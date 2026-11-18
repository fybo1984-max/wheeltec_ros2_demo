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

#include "semantic_costmap_plugin/mask_inflator.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace semantic_costmap_plugin
{

MaskInflator::MaskInflator(
  unsigned int cell_radius,
  double cost_scaling_factor,
  bool inflate_unknown,
  double resolution)
: cost_scaling_factor_(cost_scaling_factor),
  resolution_(resolution),
  cell_radius_(cell_radius),
  inflate_unknown_(inflate_unknown)
{
}

void MaskInflator::inflateCosts(
  unsigned char * costmap,
  int min_i,
  int min_j,
  int max_i,
  int max_j,
  unsigned int size_x,
  unsigned int size_y)
{
  if (costmap == nullptr || cell_radius_ == 0) {
    return;
  }

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  std::vector<std::tuple<int, int, unsigned char>> sources;
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned char cost = costmap[
        static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i)];
      if (
        cost != nav2_costmap_2d::FREE_SPACE &&
        cost != nav2_costmap_2d::NO_INFORMATION)
      {
        sources.emplace_back(i, j, cost);
      }
    }
  }

  const int radius = static_cast<int>(cell_radius_);
  const double squared_radius = static_cast<double>(cell_radius_ * cell_radius_);
  for (const auto & [source_i, source_j, source_cost] : sources) {
    const int first_i = std::max(min_i, source_i - radius);
    const int last_i = std::min(max_i - 1, source_i + radius);
    const int first_j = std::max(min_j, source_j - radius);
    const int last_j = std::min(max_j - 1, source_j + radius);
    for (int j = first_j; j <= last_j; ++j) {
      for (int i = first_i; i <= last_i; ++i) {
        const double dx = static_cast<double>(i - source_i);
        const double dy = static_cast<double>(j - source_j);
        const double squared_distance = dx * dx + dy * dy;
        if (squared_distance > squared_radius) {
          continue;
        }
        const auto index =
          static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i);
        const unsigned char old_cost = costmap[index];
        const unsigned char inflated_cost = computeCost(
          std::sqrt(squared_distance), source_cost);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
          if (inflate_unknown_ && inflated_cost > nav2_costmap_2d::FREE_SPACE) {
            costmap[index] = inflated_cost;
          }
        } else {
          costmap[index] = std::max(old_cost, inflated_cost);
        }
      }
    }
  }
}

unsigned char MaskInflator::computeCost(
  double distance,
  unsigned char source_cost) const
{
  if (distance == 0.0) {
    return source_cost;
  }
  const double factor =
    std::exp(-cost_scaling_factor_ * distance * resolution_);
  return static_cast<unsigned char>(static_cast<double>(source_cost) * factor);
}

}  // namespace semantic_costmap_plugin
