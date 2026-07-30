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

#include "semantic_costmap_plugin/fuzzy_inference_engine.hpp"

#include <algorithm>

#include "nav2_costmap_2d/cost_values.hpp"

namespace semantic_costmap_plugin
{

FuzzyInferenceEngine::FuzzyInferenceEngine()
{
  setTaskUrgency(0);
  setAvoidanceLevel(0.0);
}

unsigned char FuzzyInferenceEngine::applyFuzzy(
  unsigned int row_stride,
  unsigned char * values,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (values == nullptr) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  bool cost_calculated = false;
  unsigned char fuzzy_cost = nav2_costmap_2d::FREE_SPACE;
  for (int j = min_j; j < max_j; ++j) {
    unsigned int index = static_cast<unsigned int>(j) * row_stride +
      static_cast<unsigned int>(min_i);
    for (int i = min_i; i < max_i; ++i, ++index) {
      if (values[index] == nav2_costmap_2d::FREE_SPACE) {
        continue;
      }
      if (!cost_calculated) {
        fuzzy_cost = applyFuzzyCell(values[index]);
        cost_calculated = true;
      }
      values[index] = fuzzy_cost;
    }
  }
  return fuzzy_cost;
}

unsigned char FuzzyInferenceEngine::applyFuzzyCell(unsigned char semantic_cost) const
{
  return defuzzify(evaluateRules(semantic_cost));
}

std::array<SugenoOutput, kNumberOfFuzzyRules>
FuzzyInferenceEngine::evaluateRules(unsigned char semantic_cost) const
{
  const double base_cost = static_cast<double>(semantic_cost);
  const auto fuzzy_and = [](double lhs, double rhs) {return lhs * rhs;};

  std::array<SugenoOutput, kNumberOfFuzzyRules> rules{};

  rules[0] = {
    std::min(
      base_cost * 1.8,
      static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE)),
    avoidance_.high};
  rules[1] = {
    std::min(
      base_cost * 1.5,
      static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE)),
    fuzzy_and(avoidance_.medium, urgency_.low)};
  rules[2] = {
    base_cost * 0.2,
    fuzzy_and(urgency_.high, 1.0 - avoidance_.high)};
  rules[3] = {
    base_cost * 0.5,
    fuzzy_and(urgency_.medium, avoidance_.low)};
  rules[4] = {
    base_cost,
    fuzzy_and(urgency_.medium, avoidance_.medium)};
  rules[5] = {
    base_cost,
    fuzzy_and(urgency_.low, avoidance_.low)};

  return rules;
}

unsigned char FuzzyInferenceEngine::defuzzify(
  const std::array<SugenoOutput, kNumberOfFuzzyRules> & outputs) const
{
  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  for (const auto & output : outputs) {
    weighted_sum += output.value * output.weight;
    weight_sum += output.weight;
  }

  if (weight_sum < 1e-6) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  const double result = std::clamp(
    weighted_sum / weight_sum,
    static_cast<double>(nav2_costmap_2d::FREE_SPACE),
    static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE));
  return static_cast<unsigned char>(result);
}

FuzzyMembership FuzzyInferenceEngine::urgencyMembership(int urgency) const
{
  const double value = std::clamp(static_cast<double>(urgency), 0.0, 10.0);
  FuzzyMembership membership{};

  if (value <= 1.0) {
    membership.low = 1.0;
  } else if (value < 5.0) {
    membership.low = (5.0 - value) / 4.0;
  }

  if (value > 1.0 && value < 9.0) {
    membership.medium = value <= 5.0 ?
      (value - 1.0) / 4.0 : (9.0 - value) / 4.0;
  }

  if (value >= 9.0) {
    membership.high = 1.0;
  } else if (value > 5.0) {
    membership.high = (value - 5.0) / 4.0;
  }

  return membership;
}

FuzzyMembership FuzzyInferenceEngine::avoidanceMembership(double avoidance_level) const
{
  const double value = std::clamp(avoidance_level, 0.0, 100.0);
  FuzzyMembership membership{};

  if (value <= 20.0) {
    membership.low = 1.0;
  } else if (value < 70.0) {
    membership.low = (70.0 - value) / 50.0;
  }

  if (value >= 20.0 && value <= 90.0) {
    membership.medium = value <= 70.0 ?
      (value - 20.0) / 50.0 : (90.0 - value) / 20.0;
  }

  if (value >= 90.0) {
    membership.high = 1.0;
  } else if (value >= 70.0) {
    membership.high = (value - 70.0) / 20.0;
  }

  return membership;
}

void FuzzyInferenceEngine::setTaskUrgency(int urgency)
{
  urgency_ = urgencyMembership(urgency);
}

void FuzzyInferenceEngine::setAvoidanceLevel(double avoidance_level)
{
  avoidance_ = avoidanceMembership(avoidance_level);
}

}  // namespace semantic_costmap_plugin
