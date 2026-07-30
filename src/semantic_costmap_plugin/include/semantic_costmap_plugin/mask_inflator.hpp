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

#ifndef SEMANTIC_COSTMAP_PLUGIN__MASK_INFLATOR_HPP_
#define SEMANTIC_COSTMAP_PLUGIN__MASK_INFLATOR_HPP_

#include <cstddef>
#include <utility>
#include <vector>

namespace semantic_costmap_plugin
{

struct CellData
{
  CellData(unsigned int x, unsigned int y, unsigned int source_x, unsigned int source_y)
  : x(x), y(y), source_x(source_x), source_y(source_y) {}

  unsigned int x;
  unsigned int y;
  unsigned int source_x;
  unsigned int source_y;
};

class MaskInflator
{
public:
  MaskInflator(
    unsigned int cell_radius,
    double cost_scaling_factor,
    bool inflate_unknown,
    double resolution,
    unsigned char base_cost);

  void inflateCosts(
    unsigned char * costmap,
    int min_i,
    int min_j,
    int max_i,
    int max_j,
    unsigned int size_x,
    unsigned int size_y,
    unsigned char base_cost);

private:
  unsigned char computeCost(double distance) const;
  double distanceLookup(
    unsigned int x, unsigned int y,
    unsigned int source_x, unsigned int source_y) const;
  unsigned char costLookup(
    unsigned int x, unsigned int y,
    unsigned int source_x, unsigned int source_y) const;
  void enqueue(
    unsigned int index,
    unsigned int x,
    unsigned int y,
    unsigned int source_x,
    unsigned int source_y);
  void computeCaches();
  int generateIntegerDistances();

  double cost_scaling_factor_;
  double resolution_;
  unsigned int cell_radius_;
  unsigned int cached_cell_radius_;
  unsigned int cache_length_;
  unsigned char base_cost_;
  unsigned char cached_base_cost_;
  bool inflate_unknown_;
  std::size_t last_seen_size_;
  std::vector<std::vector<CellData>> inflation_cells_;
  std::vector<std::vector<int>> distance_matrix_;
  std::vector<bool> seen_;
  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
};

}  // namespace semantic_costmap_plugin

#endif  // SEMANTIC_COSTMAP_PLUGIN__MASK_INFLATOR_HPP_
