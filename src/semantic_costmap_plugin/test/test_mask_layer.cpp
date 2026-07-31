// Copyright 2026 WHEELTEC innovation workspace
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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_costmap_plugin/mask_layer.hpp"
#include "tf2_ros/buffer.h"

namespace
{

class MaskLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("mask_layer_test_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
      "map", false, false);
    layered_costmap_->resizeMap(10, 10, 0.1, 0.0, 0.0);
  }

  void initializeLayer(
    const std::string & source,
    const std::string & cost_mode = "fuzzy",
    int task_urgency = 0,
    double avoidance_level = 0.0)
  {
    const std::string name = "mask_layer";
    node_->declare_parameter(name + ".enabled", true);
    node_->declare_parameter(name + ".mask_source", source);
    node_->declare_parameter(
      name + ".map_yaml_path",
      std::string(TEST_FIXTURE_DIR) + "/test_mask.yaml");
    node_->declare_parameter(name + ".mask_topic", "/test_semantic_mask");
    node_->declare_parameter(name + ".cost_mode", cost_mode);
    node_->declare_parameter(name + ".mask_cost_value", 100);
    node_->declare_parameter(name + ".task_urgency", task_urgency);
    node_->declare_parameter(name + ".avoidance_level", avoidance_level);
    node_->declare_parameter(name + ".inflation_radius", 0.0);
    node_->declare_parameter(name + ".cost_scaling_factor", 2.0);
    node_->declare_parameter(name + ".inflate_unknown", false);

    layer_ = std::make_shared<semantic_costmap_plugin::MaskLayer>();
    layer_->initialize(
      layered_costmap_.get(), name, tf_buffer_.get(), node_, nullptr);
  }

  void TearDown() override
  {
    layer_.reset();
    layered_costmap_.reset();
    tf_buffer_.reset();
    node_.reset();
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<semantic_costmap_plugin::MaskLayer> layer_;
};

TEST_F(MaskLayerTest, AddsSemanticCostWithoutClearingUnknownOrLethalCells)
{
  initializeLayer("file");
  auto * master = layered_costmap_->getCostmap();
  std::fill_n(
    master->getCharMap(),
    master->getSizeInCellsX() * master->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
  master->setCost(0, 0, nav2_costmap_2d::NO_INFORMATION);
  master->setCost(4, 4, nav2_costmap_2d::LETHAL_OBSTACLE);

  layer_->updateCosts(*master, 0, 0, 10, 10);

  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getCost(4, 4), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(5, 4), 100);
  EXPECT_EQ(master->getCost(9, 9), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(MaskLayerTest, AcceptsRiskWeightedMaskFromTopic)
{
  initializeLayer("topic");
  auto publisher_node = std::make_shared<rclcpp::Node>("mask_publisher_test_node");
  auto publisher = publisher_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/test_semantic_mask",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  executor.add_node(publisher_node);

  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "map";
  mask.info.resolution = 0.1;
  mask.info.width = 10;
  mask.info.height = 10;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(100, 0);
  mask.data[4 * 10 + 5] = 50;

  for (
    int attempt = 0;
    attempt < 20 && publisher->get_subscription_count() == 0;
    ++attempt)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);
  publisher->publish(mask);
  for (int attempt = 0; attempt < 20; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  auto * master = layered_costmap_->getCostmap();
  std::fill_n(
    master->getCharMap(),
    master->getSizeInCellsX() * master->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
  layer_->updateCosts(*master, 0, 0, 10, 10);

  EXPECT_EQ(master->getCost(5, 4), 50);
  EXPECT_EQ(master->getCost(4, 4), nav2_costmap_2d::FREE_SPACE);
  executor.remove_node(publisher_node);
  executor.remove_node(node_->get_node_base_interface());
}

TEST_F(MaskLayerTest, FixedModeDoesNotApplyFuzzyPolicy)
{
  initializeLayer("file", "fixed", 10, 0.0);
  auto * master = layered_costmap_->getCostmap();
  std::fill_n(
    master->getCharMap(),
    master->getSizeInCellsX() * master->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);

  layer_->updateCosts(*master, 0, 0, 10, 10);

  EXPECT_EQ(master->getCost(5, 4), 100);
}

TEST_F(MaskLayerTest, LethalModeMarksRiskWithoutClearingUnknown)
{
  initializeLayer("file", "lethal");
  auto * master = layered_costmap_->getCostmap();
  std::fill_n(
    master->getCharMap(),
    master->getSizeInCellsX() * master->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
  master->setCost(0, 0, nav2_costmap_2d::NO_INFORMATION);

  layer_->updateCosts(*master, 0, 0, 10, 10);

  EXPECT_EQ(master->getCost(0, 0), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(master->getCost(5, 4), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(MaskLayerTest, RejectsInvalidCostModeUpdate)
{
  initializeLayer("file");

  const auto result = node_->set_parameter(
    rclcpp::Parameter("mask_layer.cost_mode", "unsupported"));

  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("cost_mode"), std::string::npos);
}

}  // namespace
