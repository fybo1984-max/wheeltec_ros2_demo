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

#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

TEST(SemanticCostmapPlugin, IsDiscoverableAndLoadable)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");

  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "semantic_costmap_plugin::MaskLayer"),
    classes.end());

  const auto layer = loader.createSharedInstance(
    "semantic_costmap_plugin::MaskLayer");
  EXPECT_NE(layer, nullptr);
}
