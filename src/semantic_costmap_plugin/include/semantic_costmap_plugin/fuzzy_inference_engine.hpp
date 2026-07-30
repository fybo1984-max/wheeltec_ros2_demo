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

#ifndef SEMANTIC_COSTMAP_PLUGIN__FUZZY_INFERENCE_ENGINE_HPP_
#define SEMANTIC_COSTMAP_PLUGIN__FUZZY_INFERENCE_ENGINE_HPP_

#include <array>
#include <cstddef>

namespace semantic_costmap_plugin
{

struct FuzzyMembership
{
  double low;
  double medium;
  double high;
};

struct SugenoOutput
{
  double value;
  double weight;
};

constexpr std::size_t kNumberOfFuzzyRules = 6;

class FuzzyInferenceEngine
{
public:
  FuzzyInferenceEngine();

  void setTaskUrgency(int urgency);
  void setAvoidanceLevel(double avoidance_level);

  unsigned char applyFuzzy(
    unsigned int row_stride,
    unsigned char * values,
    int min_i,
    int min_j,
    int max_i,
    int max_j);

private:
  unsigned char applyFuzzyCell(unsigned char semantic_cost) const;
  FuzzyMembership urgencyMembership(int urgency) const;
  FuzzyMembership avoidanceMembership(double avoidance_level) const;
  std::array<SugenoOutput, kNumberOfFuzzyRules> evaluateRules(
    unsigned char semantic_cost) const;
  unsigned char defuzzify(
    const std::array<SugenoOutput, kNumberOfFuzzyRules> & outputs) const;

  FuzzyMembership urgency_;
  FuzzyMembership avoidance_;
};

}  // namespace semantic_costmap_plugin

#endif  // SEMANTIC_COSTMAP_PLUGIN__FUZZY_INFERENCE_ENGINE_HPP_
