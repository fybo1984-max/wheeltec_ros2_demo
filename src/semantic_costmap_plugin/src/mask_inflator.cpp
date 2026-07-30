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

#include "nav2_costmap_2d/cost_values.hpp"

namespace semantic_costmap_plugin
{

MaskInflator::MaskInflator(
  unsigned int cell_radius,
  double cost_scaling_factor,
  bool inflate_unknown,
  double resolution,
  unsigned char base_cost)
: cost_scaling_factor_(cost_scaling_factor),
  resolution_(resolution),
  cell_radius_(cell_radius),
  cached_cell_radius_(0),
  cache_length_(0),
  base_cost_(base_cost),
  cached_base_cost_(0),
  inflate_unknown_(inflate_unknown),
  last_seen_size_(0)
{
  computeCaches();
}

void MaskInflator::inflateCosts(
  unsigned char * costmap,
  int min_i,
  int min_j,
  int max_i,
  int max_j,
  unsigned int size_x,
  unsigned int size_y,
  unsigned char base_cost)
{
  if (costmap == nullptr || cell_radius_ == 0) {
    return;
  }

  base_cost_ = base_cost;
  computeCaches();

  const std::size_t map_size = static_cast<std::size_t>(size_x) * size_y;
  if (last_seen_size_ != map_size) {
    seen_.resize(map_size);
    last_seen_size_ = map_size;
  }
  std::fill(seen_.begin(), seen_.end(), false);

  const int requested_min_i = min_i;
  const int requested_min_j = min_j;
  const int requested_max_i = max_i;
  const int requested_max_j = max_j;
  const int radius = static_cast<int>(cell_radius_);
  min_i = std::max(0, min_i - radius);
  min_j = std::max(0, min_j - radius);
  max_i = std::min(static_cast<int>(size_x), max_i + radius);
  max_j = std::min(static_cast<int>(size_y), max_j + radius);

  auto & source_cells = inflation_cells_.front();
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const auto index =
        static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i);
      if (costmap[index] == base_cost_) {
        source_cells.emplace_back(i, j, i, j);
      }
    }
  }

  for (auto & distance_bin : inflation_cells_) {
    for (std::size_t cell_index = 0; cell_index < distance_bin.size(); ++cell_index) {
      const CellData cell = distance_bin[cell_index];
      const unsigned int index = cell.y * size_x + cell.x;
      if (seen_[index]) {
        continue;
      }
      seen_[index] = true;

      const unsigned char cost =
        costLookup(cell.x, cell.y, cell.source_x, cell.source_y);
      const unsigned char old_cost = costmap[index];
      const bool inside_requested_bounds =
        static_cast<int>(cell.x) >= requested_min_i &&
        static_cast<int>(cell.y) >= requested_min_j &&
        static_cast<int>(cell.x) < requested_max_i &&
        static_cast<int>(cell.y) < requested_max_j;

      if (inside_requested_bounds) {
        if (
          old_cost == nav2_costmap_2d::NO_INFORMATION &&
          inflate_unknown_ && cost > nav2_costmap_2d::FREE_SPACE)
        {
          costmap[index] = cost;
        } else if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
          costmap[index] = std::max(old_cost, cost);
        }
      }

      if (cell.x > 0) {
        enqueue(index - 1, cell.x - 1, cell.y, cell.source_x, cell.source_y);
      }
      if (cell.y > 0) {
        enqueue(index - size_x, cell.x, cell.y - 1, cell.source_x, cell.source_y);
      }
      if (cell.x + 1 < size_x) {
        enqueue(index + 1, cell.x + 1, cell.y, cell.source_x, cell.source_y);
      }
      if (cell.y + 1 < size_y) {
        enqueue(index + size_x, cell.x, cell.y + 1, cell.source_x, cell.source_y);
      }
    }
    distance_bin.clear();
  }
}

unsigned char MaskInflator::computeCost(double distance) const
{
  if (distance == 0.0) {
    return base_cost_;
  }
  const double factor =
    std::exp(-cost_scaling_factor_ * distance * resolution_);
  return static_cast<unsigned char>(static_cast<double>(base_cost_) * factor);
}

double MaskInflator::distanceLookup(
  unsigned int x,
  unsigned int y,
  unsigned int source_x,
  unsigned int source_y) const
{
  const unsigned int dx = x > source_x ? x - source_x : source_x - x;
  const unsigned int dy = y > source_y ? y - source_y : source_y - y;
  return cached_distances_[dx * cache_length_ + dy];
}

unsigned char MaskInflator::costLookup(
  unsigned int x,
  unsigned int y,
  unsigned int source_x,
  unsigned int source_y) const
{
  const unsigned int dx = x > source_x ? x - source_x : source_x - x;
  const unsigned int dy = y > source_y ? y - source_y : source_y - y;
  return cached_costs_[dx * cache_length_ + dy];
}

void MaskInflator::enqueue(
  unsigned int index,
  unsigned int x,
  unsigned int y,
  unsigned int source_x,
  unsigned int source_y)
{
  if (seen_[index]) {
    return;
  }

  const double distance = distanceLookup(x, y, source_x, source_y);
  if (distance > static_cast<double>(cell_radius_)) {
    return;
  }

  const int cache_origin = static_cast<int>(cell_radius_) + 2;
  const int dx = static_cast<int>(x) - static_cast<int>(source_x) + cache_origin;
  const int dy = static_cast<int>(y) - static_cast<int>(source_y) + cache_origin;
  const int distance_bin = distance_matrix_[dx][dy];
  inflation_cells_[distance_bin].emplace_back(x, y, source_x, source_y);
}

void MaskInflator::computeCaches()
{
  if (cell_radius_ == 0 || (
      cell_radius_ == cached_cell_radius_ &&
      base_cost_ == cached_base_cost_))
  {
    return;
  }

  cache_length_ = cell_radius_ + 2;
  if (cell_radius_ != cached_cell_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);
    for (unsigned int x = 0; x < cache_length_; ++x) {
      for (unsigned int y = 0; y < cache_length_; ++y) {
        cached_distances_[x * cache_length_ + y] = std::hypot(x, y);
      }
    }
    cached_cell_radius_ = cell_radius_;
  }

  for (unsigned int x = 0; x < cache_length_; ++x) {
    for (unsigned int y = 0; y < cache_length_; ++y) {
      const unsigned int index = x * cache_length_ + y;
      cached_costs_[index] = computeCost(cached_distances_[index]);
    }
  }
  cached_base_cost_ = base_cost_;

  const int maximum_distance_bin = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(static_cast<std::size_t>(maximum_distance_bin + 1));
}

int MaskInflator::generateIntegerDistances()
{
  const int radius = static_cast<int>(cell_radius_) + 2;
  const int diameter = radius * 2 + 1;
  std::vector<std::pair<int, int>> points;

  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      if (x * x + y * y <= radius * radius) {
        points.emplace_back(x, y);
      }
    }
  }
  std::sort(
    points.begin(), points.end(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.first * lhs.first + lhs.second * lhs.second <
      rhs.first * rhs.first + rhs.second * rhs.second;
    });

  distance_matrix_.assign(
    static_cast<std::size_t>(diameter),
    std::vector<int>(static_cast<std::size_t>(diameter), 0));
  std::pair<int, int> previous{0, 0};
  int level = 0;
  for (const auto & point : points) {
    const int squared_distance =
      point.first * point.first + point.second * point.second;
    const int previous_squared_distance =
      previous.first * previous.first + previous.second * previous.second;
    if (squared_distance != previous_squared_distance) {
      ++level;
    }
    distance_matrix_[point.first + radius][point.second + radius] = level;
    previous = point;
  }
  return level;
}

}  // namespace semantic_costmap_plugin
