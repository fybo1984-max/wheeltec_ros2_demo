// Copyright 2026 Tiago Alves
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified 2026 by WHEELTEC innovation workspace for ROS 2 Humble.

#include "semantic_costmap_plugin/mask_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_map_server/map_io.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace semantic_costmap_plugin
{

MaskLayer::MaskLayer()
: mask_cost_value_(125),
  map_loaded_(false),
  fuzzy_recalculation_needed_(true),
  task_urgency_(0),
  avoidance_level_(70.0),
  inverse_mask_resolution_(0.0),
  mask_bounds_cached_(false),
  mask_min_i_(0),
  mask_min_j_(0),
  mask_max_i_(0),
  mask_max_j_(0),
  publish_divisor_(10),
  update_count_(0),
  fuzzy_engine_(std::make_unique<FuzzyInferenceEngine>())
{
}

MaskLayer::~MaskLayer()
{
  parameter_callback_.reset();
}

void MaskLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("mask_source", rclcpp::ParameterValue("file"));
  declareParameter("map_yaml_path", rclcpp::ParameterValue(""));
  declareParameter("mask_topic", rclcpp::ParameterValue("/semantic_mask"));
  declareParameter("mask_cost_value", rclcpp::ParameterValue(125));
  declareParameter("task_urgency", rclcpp::ParameterValue(0));
  declareParameter("avoidance_level", rclcpp::ParameterValue(70.0));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.5));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(2.0));
  declareParameter("inflate_unknown", rclcpp::ParameterValue(false));

  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("semantic mask layer could not lock its lifecycle node");
  }

  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  mask_source_ = node->get_parameter(name_ + ".mask_source").as_string();
  map_yaml_path_ = node->get_parameter(name_ + ".map_yaml_path").as_string();
  mask_topic_ = node->get_parameter(name_ + ".mask_topic").as_string();
  const auto mask_cost = node->get_parameter(name_ + ".mask_cost_value").as_int();
  const auto task_urgency = node->get_parameter(name_ + ".task_urgency").as_int();
  const double avoidance_level =
    node->get_parameter(name_ + ".avoidance_level").as_double();
  const double inflation_radius =
    node->get_parameter(name_ + ".inflation_radius").as_double();
  const double scaling_factor =
    node->get_parameter(name_ + ".cost_scaling_factor").as_double();
  const bool inflate_unknown =
    node->get_parameter(name_ + ".inflate_unknown").as_bool();

  if (mask_source_ != "file" && mask_source_ != "topic") {
    throw std::runtime_error("mask_layer.mask_source must be 'file' or 'topic'");
  }
  if (mask_source_ == "file" && map_yaml_path_.empty()) {
    throw std::runtime_error(
            "mask_layer.map_yaml_path must not be empty for file source");
  }
  if (mask_source_ == "topic" && mask_topic_.empty()) {
    throw std::runtime_error(
            "mask_layer.mask_topic must not be empty for topic source");
  }
  if (mask_cost < 1 || mask_cost > nav2_costmap_2d::MAX_NON_OBSTACLE) {
    throw std::runtime_error("mask_layer.mask_cost_value must be in [1, 252]");
  }
  if (task_urgency < 0 || task_urgency > 10) {
    throw std::runtime_error("mask_layer.task_urgency must be in [0, 10]");
  }
  if (avoidance_level < 0.0 || avoidance_level > 100.0) {
    throw std::runtime_error("mask_layer.avoidance_level must be in [0, 100]");
  }
  if (inflation_radius < 0.0 || scaling_factor < 0.0) {
    throw std::runtime_error(
            "mask_layer inflation_radius and cost_scaling_factor must be nonnegative");
  }

  mask_cost_value_ = static_cast<unsigned char>(mask_cost);
  task_urgency_ = static_cast<int>(task_urgency);
  avoidance_level_ = avoidance_level;

  if (mask_source_ == "file") {
    if (!loadMask()) {
      throw std::runtime_error("failed to load semantic mask: " + map_yaml_path_);
    }
    inverse_mask_resolution_ = 1.0 / mask_.info.resolution;
  }

  matchSize();
  auto * master = layered_costmap_->getCostmap();
  inflator_ = std::make_unique<MaskInflator>(
    master->cellDistance(inflation_radius),
    scaling_factor,
    inflate_unknown,
    master->getResolution(),
    mask_cost_value_);

  fuzzy_engine_->setTaskUrgency(task_urgency_);
  fuzzy_engine_->setAvoidanceLevel(avoidance_level_);
  fuzzy_recalculation_needed_ = false;

  parameter_callback_ = node->add_on_set_parameters_callback(
    std::bind(
      &MaskLayer::onParametersChanged, this,
      std::placeholders::_1));
  cost_publisher_ = node->create_publisher<std_msgs::msg::UInt8>(
    "semantic_zone/fuzzy_cost_value", rclcpp::QoS(10));
  if (mask_source_ == "topic") {
    auto mask_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    mask_qos.reliable();
    mask_qos.transient_local();
    mask_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      mask_topic_, mask_qos,
      std::bind(&MaskLayer::maskCallback, this, std::placeholders::_1));
  }

  current_ = true;
  if (mask_source_ == "file") {
    RCLCPP_INFO(
      logger_,
      "Semantic mask layer loaded %ux%u mask from %s (enabled=%s)",
      mask_.info.width, mask_.info.height, map_yaml_path_.c_str(),
      enabled_ ? "true" : "false");
  } else {
    RCLCPP_INFO(
      logger_,
      "Semantic mask layer waiting for OccupancyGrid on %s (enabled=%s)",
      mask_topic_.c_str(), enabled_ ? "true" : "false");
  }
}

void MaskLayer::matchSize()
{
  nav2_costmap_2d::CostmapLayer::matchSize();
  mask_bounds_cached_ = false;
}

void MaskLayer::reset()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  resetMaps();
  mask_bounds_cached_ = false;
  current_ = true;
}

bool MaskLayer::loadMask()
{
  const auto node = node_.lock();
  if (!node) {
    return false;
  }

  try {
    const auto status = nav2_map_server::loadMapFromYaml(map_yaml_path_, mask_);
    if (status != nav2_map_server::LOAD_MAP_SUCCESS) {
      RCLCPP_ERROR(
        logger_, "nav2_map_server returned load status %d for %s",
        static_cast<int>(status), map_yaml_path_.c_str());
      return false;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Error loading semantic mask: %s", error.what());
    return false;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(mask_.info.width) * mask_.info.height;
  if (
    mask_.info.resolution <= 0.0 ||
    mask_.info.width == 0 ||
    mask_.info.height == 0 ||
    mask_.data.size() != expected_size)
  {
    RCLCPP_ERROR(logger_, "Semantic mask metadata or data size is invalid");
    return false;
  }

  const auto & orientation = mask_.info.origin.orientation;
  constexpr double tolerance = 1e-6;
  if (
    std::abs(orientation.x) > tolerance ||
    std::abs(orientation.y) > tolerance ||
    std::abs(orientation.z) > tolerance ||
    std::abs(std::abs(orientation.w) - 1.0) > tolerance)
  {
    RCLCPP_ERROR(logger_, "Rotated semantic masks are not supported");
    return false;
  }

  map_loaded_ = true;
  return true;
}

bool MaskLayer::validateMask(
  const nav_msgs::msg::OccupancyGrid & mask,
  std::string & reason) const
{
  const std::size_t expected_size =
    static_cast<std::size_t>(mask.info.width) * mask.info.height;
  if (
    mask.info.resolution <= 0.0 ||
    mask.info.width == 0 ||
    mask.info.height == 0 ||
    mask.data.size() != expected_size)
  {
    reason = "metadata or data size is invalid";
    return false;
  }

  const auto & orientation = mask.info.origin.orientation;
  constexpr double tolerance = 1e-6;
  if (
    std::abs(orientation.x) > tolerance ||
    std::abs(orientation.y) > tolerance ||
    std::abs(orientation.z) > tolerance ||
    std::abs(std::abs(orientation.w) - 1.0) > tolerance)
  {
    reason = "rotated masks are not supported";
    return false;
  }
  if (mask.header.frame_id != layered_costmap_->getGlobalFrameID()) {
    reason =
      "frame_id '" + mask.header.frame_id + "' does not match global frame '" +
      layered_costmap_->getGlobalFrameID() + "'";
    return false;
  }
  return true;
}

void MaskLayer::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  std::string reason;
  if (!validateMask(*message, reason)) {
    RCLCPP_ERROR(
      logger_, "Rejected semantic mask from %s: %s",
      mask_topic_.c_str(), reason.c_str());
    return;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  mask_ = *message;
  inverse_mask_resolution_ = 1.0 / mask_.info.resolution;
  map_loaded_ = true;
  mask_bounds_cached_ = false;
  current_ = true;
}

void MaskLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  if (!enabled_ || !map_loaded_) {
    return;
  }

  *min_x = -std::numeric_limits<double>::max();
  *min_y = -std::numeric_limits<double>::max();
  *max_x = std::numeric_limits<double>::max();
  *max_y = std::numeric_limits<double>::max();
}

void MaskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  if (!enabled_ || !map_loaded_) {
    return;
  }

  if (fuzzy_recalculation_needed_) {
    fuzzy_engine_->setTaskUrgency(task_urgency_);
    fuzzy_engine_->setAvoidanceLevel(avoidance_level_);
    fuzzy_recalculation_needed_ = false;
  }
  applyMask(master_grid, min_i, min_j, max_i, max_j);
}

void MaskLayer::computeMaskBounds()
{
  const auto * master = layered_costmap_->getCostmap();
  const double master_min_x = master->getOriginX();
  const double master_min_y = master->getOriginY();
  const double master_max_x =
    master_min_x + master->getSizeInCellsX() * master->getResolution();
  const double master_max_y =
    master_min_y + master->getSizeInCellsY() * master->getResolution();

  const double mask_min_x = mask_.info.origin.position.x;
  const double mask_min_y = mask_.info.origin.position.y;
  const double mask_max_x = mask_min_x + mask_.info.width * mask_.info.resolution;
  const double mask_max_y = mask_min_y + mask_.info.height * mask_.info.resolution;

  const double overlap_min_x = std::max(master_min_x, mask_min_x);
  const double overlap_min_y = std::max(master_min_y, mask_min_y);
  const double overlap_max_x = std::min(master_max_x, mask_max_x);
  const double overlap_max_y = std::min(master_max_y, mask_max_y);

  if (overlap_min_x >= overlap_max_x || overlap_min_y >= overlap_max_y) {
    mask_min_i_ = mask_min_j_ = mask_max_i_ = mask_max_j_ = 0;
  } else {
    mask_min_i_ = static_cast<int>(
      (overlap_min_x - master_min_x) / master->getResolution());
    mask_min_j_ = static_cast<int>(
      (overlap_min_y - master_min_y) / master->getResolution());
    mask_max_i_ = static_cast<int>(
      std::ceil((overlap_max_x - master_min_x) / master->getResolution()));
    mask_max_j_ = static_cast<int>(
      std::ceil((overlap_max_y - master_min_y) / master->getResolution()));
    mask_max_i_ = std::min(mask_max_i_, static_cast<int>(master->getSizeInCellsX()));
    mask_max_j_ = std::min(mask_max_j_, static_cast<int>(master->getSizeInCellsY()));
  }
  mask_bounds_cached_ = true;
}

void MaskLayer::applyMask(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!mask_bounds_cached_) {
    computeMaskBounds();
  }

  const int start_i = std::max(min_i, mask_min_i_);
  const int start_j = std::max(min_j, mask_min_j_);
  const int end_i = std::min(max_i, mask_max_i_);
  const int end_j = std::min(max_j, mask_max_j_);
  if (start_i >= end_i || start_j >= end_j) {
    return;
  }

  const double master_resolution = master_grid.getResolution();
  const double master_origin_x = master_grid.getOriginX();
  const double master_origin_y = master_grid.getOriginY();
  const double mask_origin_x = mask_.info.origin.position.x;
  const double mask_origin_y = mask_.info.origin.position.y;

  double world_y = master_origin_y + (static_cast<double>(start_j) + 0.5) *
    master_resolution;
  for (int j = start_j; j < end_j; ++j, world_y += master_resolution) {
    const int mask_j = static_cast<int>(
      (world_y - mask_origin_y) * inverse_mask_resolution_);
    double world_x = master_origin_x + (static_cast<double>(start_i) + 0.5) *
      master_resolution;
    for (int i = start_i; i < end_i; ++i, world_x += master_resolution) {
      const int mask_i = static_cast<int>(
        (world_x - mask_origin_x) * inverse_mask_resolution_);
      unsigned char layer_cost = nav2_costmap_2d::FREE_SPACE;
      if (
        mask_i >= 0 && mask_j >= 0 &&
        mask_i < static_cast<int>(mask_.info.width) &&
        mask_j < static_cast<int>(mask_.info.height))
      {
        const auto mask_index =
          static_cast<std::size_t>(mask_j) * mask_.info.width +
          static_cast<std::size_t>(mask_i);
        const int risk_value = static_cast<int>(mask_.data[mask_index]);
        if (risk_value > 0) {
          const double scaled_cost =
            static_cast<double>(mask_cost_value_) *
            std::min(risk_value, 100) / 100.0;
          layer_cost = static_cast<unsigned char>(
            std::clamp(
              static_cast<int>(std::lround(scaled_cost)),
              1,
              static_cast<int>(nav2_costmap_2d::MAX_NON_OBSTACLE)));
        }
      }
      costmap_[getIndex(i, j)] = layer_cost;
    }
  }

  const unsigned char fuzzy_cost = fuzzy_engine_->applyFuzzy(
    master_grid.getSizeInCellsX(), costmap_, start_i, start_j, end_i, end_j);
  if (fuzzy_cost != nav2_costmap_2d::FREE_SPACE) {
    inflator_->inflateCosts(
      costmap_, start_i, start_j, end_i, end_j,
      master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(), fuzzy_cost);
  }
  for (int j = start_j; j < end_j; ++j) {
    for (int i = start_i; i < end_i; ++i) {
      const unsigned char layer_cost = costmap_[getIndex(i, j)];
      const unsigned char master_cost = master_grid.getCost(i, j);
      if (
        layer_cost != nav2_costmap_2d::FREE_SPACE &&
        layer_cost != nav2_costmap_2d::NO_INFORMATION &&
        master_cost != nav2_costmap_2d::NO_INFORMATION)
      {
        master_grid.setCost(i, j, std::max(master_cost, layer_cost));
      }
    }
  }

  if (++update_count_ >= publish_divisor_) {
    std_msgs::msg::UInt8 message;
    message.data = fuzzy_cost;
    cost_publisher_->publish(message);
    update_count_ = 0;
  }
}

rcl_interfaces::msg::SetParametersResult MaskLayer::onParametersChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_ + ".enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "mask_layer.enabled must be a boolean";
        return result;
      }
    } else if (parameter.get_name() == name_ + ".task_urgency") {
      if (
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter.as_int() < 0 || parameter.as_int() > 10)
      {
        result.successful = false;
        result.reason = "mask_layer.task_urgency must be an integer in [0, 10]";
        return result;
      }
    } else if (parameter.get_name() == name_ + ".avoidance_level") {
      if (
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
        parameter.as_double() < 0.0 || parameter.as_double() > 100.0)
      {
        result.successful = false;
        result.reason = "mask_layer.avoidance_level must be a double in [0, 100]";
        return result;
      }
    }
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name_ + ".enabled") {
      enabled_ = parameter.as_bool();
    } else if (parameter.get_name() == name_ + ".task_urgency") {
      task_urgency_ = static_cast<int>(parameter.as_int());
      fuzzy_recalculation_needed_ = true;
    } else if (parameter.get_name() == name_ + ".avoidance_level") {
      avoidance_level_ = parameter.as_double();
      fuzzy_recalculation_needed_ = true;
    }
  }
  return result;
}

}  // namespace semantic_costmap_plugin

PLUGINLIB_EXPORT_CLASS(
  semantic_costmap_plugin::MaskLayer,
  nav2_costmap_2d::Layer)
