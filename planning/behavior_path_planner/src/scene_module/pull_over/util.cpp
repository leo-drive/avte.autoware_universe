// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

namespace behavior_path_planner
{
namespace pull_over_utils
{
PathWithLaneId combineReferencePath(const PathWithLaneId & path1, const PathWithLaneId & path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

lanelet::ConstLanelets getPullOverLanes(const RouteHandler & route_handler)
{
  lanelet::ConstLanelets pull_over_lanes;
  lanelet::ConstLanelet target_shoulder_lane;
  const Pose goal_pose = route_handler.getGoalPose();

  if (route_handler::RouteHandler::getPullOverTarget(
        route_handler.getShoulderLanelets(), goal_pose, &target_shoulder_lane)) {
    constexpr double pull_over_lane_length = 200;
    pull_over_lanes = route_handler.getShoulderLaneletSequence(
      target_shoulder_lane, goal_pose, pull_over_lane_length, pull_over_lane_length);
  }
  return pull_over_lanes;
}

PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside)
{
  PredictedObjects filtered_objects;
  for (const auto & object : objects.objects) {
    const double distance =
      util::calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, object);
    if (filter_inside ? distance < distance_thresh : distance > distance_thresh) {
      filtered_objects.objects.push_back(object);
    }
  }

  return filtered_objects;
}

MarkerArray createPullOverAreaMarkerArray(
  GoalCandidates goal_candidates, const std_msgs::msg::Header & header,
  const double base_link2front, const double base_link2rear, const double vehicle_width,
  const std_msgs::msg::ColorRGBA & color)
{
  using tier4_autoware_utils::MultiPolygon2d;
  using tier4_autoware_utils::Point2d;
  using tier4_autoware_utils::Polygon2d;

  MarkerArray marker_array{};
  if (goal_candidates.empty()) {
    return marker_array;
  }

  // sort goal candidates by id
  std::sort(goal_candidates.begin(), goal_candidates.end(), [](auto const & a, auto const & b) {
    return a.first.id < b.first.id;
  });

  // classify candidates by id
  std::map<size_t, std::vector<Pose>> goal_poses_map{};
  for (const auto & candidate : goal_candidates) {
    goal_poses_map[candidate.second].push_back(candidate.first.goal_pose);
  }

  const auto appendPointToPolygon =
    [](Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point) {
      Point2d point{};
      point.x() = geom_point.x;
      point.y() = geom_point.y;
      boost::geometry::append(polygon.outer(), point);
    };

  // calculate average z of candidates
  const double average_z =
    std::accumulate(
      begin(goal_candidates), end(goal_candidates), 0,
      [](double sum, const auto & c) { return sum + c.first.goal_pose.position.z; }) /
    goal_candidates.size();

  for (const auto & id_and_poses : goal_poses_map) {
    const size_t area_id = id_and_poses.first;
    const auto & poses = id_and_poses.second;

    MultiPolygon2d unions{};
    for (const auto p : poses) {
      Polygon2d footprint{};
      const Point p_left_front = calcOffsetPose(p, base_link2front, vehicle_width / 2, 0).position;
      appendPointToPolygon(footprint, p_left_front);

      const Point p_right_front =
        calcOffsetPose(p, base_link2front, -vehicle_width / 2, 0).position;
      appendPointToPolygon(footprint, p_right_front);

      const Point p_right_back = calcOffsetPose(p, -base_link2rear, -vehicle_width / 2, 0).position;
      appendPointToPolygon(footprint, p_right_back);

      const Point p_left_back = calcOffsetPose(p, -base_link2rear, vehicle_width / 2, 0).position;
      appendPointToPolygon(footprint, p_left_back);

      appendPointToPolygon(footprint, p_left_front);

      MultiPolygon2d current_result{};
      boost::geometry::union_(footprint, unions, current_result);
      unions = current_result;
    }
    if (unions.empty()) {
      continue;
    }

    Marker marker = createDefaultMarker(
      header.frame_id, header.stamp, "pull_over_area_" + std::to_string(area_id), area_id,
      visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), color);
    for (const auto & u : unions) {
      for (const auto & p : u.outer()) {
        marker.points.push_back(createPoint(p.x(), p.y(), average_z));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = tier4_autoware_utils::createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::ARROW,
      createMarkerScale(0.5, 0.25, 0.25), color);
    marker.pose = pose;
    marker.id = i++;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.3, 0.3, 0.3), color);
    marker.pose = calcOffsetPose(pose, 0, 0, 1.0);
    marker.id = i;
    marker.text = std::to_string(i);
    msg.markers.push_back(marker);
    i++;
  }

  return msg;
}

MarkerArray createGoalCandidatesMarkerArray(
  GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color)
{
  // convert to pose vector
  std::vector<Pose> pose_vector{};
  for (const auto & goal_candidate : goal_candidates) {
    pose_vector.push_back(goal_candidate.first.goal_pose);
  }

  auto marker_array = createPosesMarkerArray(pose_vector, "goal_candidates", color);
  for (const auto & text_marker :
       createTextsMarkerArray(
         pose_vector, "goal_candidates_priority", createMarkerColor(1.0, 1.0, 1.0, 0.999))
         .markers) {
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}
}  // namespace pull_over_utils
}  // namespace behavior_path_planner
