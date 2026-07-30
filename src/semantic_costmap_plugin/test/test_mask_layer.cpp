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
#include <memory>
#include <string>

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

    const std::string name = "mask_layer";
    node_->declare_parameter(name + ".enabled", true);
    node_->declare_parameter(
      name + ".map_yaml_path",
      std::string(TEST_FIXTURE_DIR) + "/test_mask.yaml");
    node_->declare_parameter(name + ".mask_cost_value", 100);
    node_->declare_parameter(name + ".task_urgency", 0);
    node_->declare_parameter(name + ".avoidance_level", 0.0);
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

}  // namespace
