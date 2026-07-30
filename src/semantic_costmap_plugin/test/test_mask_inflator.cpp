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

#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "semantic_costmap_plugin/mask_inflator.hpp"

namespace
{

TEST(MaskInflator, ProducesBoundedMonotonicSoftCosts)
{
  constexpr unsigned int width = 11;
  constexpr unsigned int height = 11;
  constexpr unsigned char base_cost = 180;
  std::vector<unsigned char> grid(
    width * height, nav2_costmap_2d::FREE_SPACE);
  grid[5 * width + 5] = base_cost;

  semantic_costmap_plugin::MaskInflator inflator(
    3, 2.0, false, 0.1, base_cost);
  inflator.inflateCosts(
    grid.data(), 0, 0, width, height, width, height, base_cost);

  EXPECT_EQ(grid[5 * width + 5], base_cost);
  EXPECT_GT(grid[5 * width + 6], grid[5 * width + 7]);
  EXPECT_GT(grid[5 * width + 7], grid[5 * width + 8]);
  EXPECT_EQ(grid[5 * width + 9], nav2_costmap_2d::FREE_SPACE);
  EXPECT_LT(grid[5 * width + 6], nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

TEST(MaskInflator, RepeatedUpdatesDoNotRetainOldSources)
{
  constexpr unsigned int width = 9;
  constexpr unsigned int height = 9;
  constexpr unsigned char base_cost = 125;
  semantic_costmap_plugin::MaskInflator inflator(
    2, 2.0, false, 0.1, base_cost);

  std::vector<unsigned char> first(
    width * height, nav2_costmap_2d::FREE_SPACE);
  first[2 * width + 2] = base_cost;
  inflator.inflateCosts(
    first.data(), 0, 0, width, height, width, height, base_cost);

  std::vector<unsigned char> second(
    width * height, nav2_costmap_2d::FREE_SPACE);
  second[6 * width + 6] = base_cost;
  inflator.inflateCosts(
    second.data(), 0, 0, width, height, width, height, base_cost);

  EXPECT_EQ(second[2 * width + 2], nav2_costmap_2d::FREE_SPACE);
  EXPECT_GT(second[6 * width + 5], nav2_costmap_2d::FREE_SPACE);
}

TEST(MaskInflator, ZeroRadiusLeavesMapUnchanged)
{
  constexpr unsigned int width = 5;
  constexpr unsigned char base_cost = 100;
  std::vector<unsigned char> grid(
    width * width, nav2_costmap_2d::FREE_SPACE);
  grid[2 * width + 2] = base_cost;
  const auto original = grid;

  semantic_costmap_plugin::MaskInflator inflator(
    0, 2.0, false, 0.05, base_cost);
  inflator.inflateCosts(
    grid.data(), 0, 0, width, width, width, width, base_cost);

  EXPECT_EQ(grid, original);
}

}  // namespace
