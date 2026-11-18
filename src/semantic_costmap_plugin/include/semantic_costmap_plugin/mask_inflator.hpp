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

namespace semantic_costmap_plugin
{

class MaskInflator
{
public:
  MaskInflator(
    unsigned int cell_radius,
    double cost_scaling_factor,
    bool inflate_unknown,
    double resolution);

  void inflateCosts(
    unsigned char * costmap,
    int min_i,
    int min_j,
    int max_i,
    int max_j,
    unsigned int size_x,
    unsigned int size_y);

private:
  unsigned char computeCost(double distance, unsigned char source_cost) const;

  double cost_scaling_factor_;
  double resolution_;
  unsigned int cell_radius_;
  bool inflate_unknown_;
};

}  // namespace semantic_costmap_plugin

#endif  // SEMANTIC_COSTMAP_PLUGIN__MASK_INFLATOR_HPP_
