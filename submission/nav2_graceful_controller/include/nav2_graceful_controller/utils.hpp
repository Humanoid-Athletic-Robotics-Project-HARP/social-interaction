// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_
#define NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_graceful_controller
{
/**
   * @brief Create a flat circle marker of radius slowdown_radius around the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target
   * @param slowdown_radius Radius of the slowdown circle
   * @return visualization_msgs::msg::Marker Slowdown marker
   */
visualization_msgs::msg::Marker createSlowdownMarker(
  const geometry_msgs::msg::PoseStamped & motion_target, const double & slowdown_radius);

// HARP (Foxy port): these three are nav2_util::geometry_utils' from Nav2 jazzy,
// which Foxy's nav2_util doesn't have.

/** @brief The element of [begin, end) with the lowest getCompareVal, the last one on ties */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp <= lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/** @brief The first pose after the path's integrated distance passes getCompareVal */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += nav2_util::geometry_utils::euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/** @brief The length of the path from start_index to its end */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += nav2_util::geometry_utils::euclidean_distance(
      path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__UTILS_HPP_
