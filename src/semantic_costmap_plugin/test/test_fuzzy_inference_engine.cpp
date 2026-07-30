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
#include "semantic_costmap_plugin/fuzzy_inference_engine.hpp"

namespace
{

unsigned char evaluateCost(int urgency, double avoidance, unsigned char base_cost = 100)
{
  semantic_costmap_plugin::FuzzyInferenceEngine engine;
  engine.setTaskUrgency(urgency);
  engine.setAvoidanceLevel(avoidance);
  std::vector<unsigned char> grid(9, nav2_costmap_2d::FREE_SPACE);
  grid[4] = base_cost;
  return engine.applyFuzzy(3, grid.data(), 0, 0, 3, 3);
}

TEST(FuzzyInferenceEngine, UrgencyAndAvoidanceChangeSoftCost)
{
  EXPECT_EQ(evaluateCost(0, 0.0), 100);
  EXPECT_EQ(evaluateCost(10, 0.0), 20);
  EXPECT_EQ(evaluateCost(0, 100.0), 180);
}

TEST(FuzzyInferenceEngine, CostNeverBecomesLethal)
{
  EXPECT_EQ(
    evaluateCost(0, 100.0, nav2_costmap_2d::MAX_NON_OBSTACLE),
    nav2_costmap_2d::MAX_NON_OBSTACLE);
}

TEST(FuzzyInferenceEngine, OnlySemanticCellsAreChanged)
{
  semantic_costmap_plugin::FuzzyInferenceEngine engine;
  engine.setTaskUrgency(10);
  engine.setAvoidanceLevel(0.0);
  std::vector<unsigned char> grid(16, nav2_costmap_2d::FREE_SPACE);
  grid[5] = 100;
  grid[10] = 100;

  const unsigned char result = engine.applyFuzzy(4, grid.data(), 0, 0, 4, 4);

  EXPECT_EQ(result, 20);
  EXPECT_EQ(grid[5], result);
  EXPECT_EQ(grid[10], result);
  EXPECT_EQ(grid[0], nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(grid[15], nav2_costmap_2d::FREE_SPACE);
}

TEST(FuzzyInferenceEngine, NullAndFreeMapsReturnFreeSpace)
{
  semantic_costmap_plugin::FuzzyInferenceEngine engine;
  EXPECT_EQ(
    engine.applyFuzzy(3, nullptr, 0, 0, 3, 3),
    nav2_costmap_2d::FREE_SPACE);

  std::vector<unsigned char> grid(9, nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(
    engine.applyFuzzy(3, grid.data(), 0, 0, 3, 3),
    nav2_costmap_2d::FREE_SPACE);
}

}  // namespace
