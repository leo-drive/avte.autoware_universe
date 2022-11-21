// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/pull_over/shift_pull_over.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
ShiftPullOver::ShiftPullOver(
  rclcpp::Node & node, const PullOverParameters & parameters,
  const LaneDepartureChecker & lane_departure_checker,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
: PullOverPlannerBase{node, parameters},
  lane_departure_checker_{lane_departure_checker},
  occupancy_grid_map_{occupancy_grid_map}
{
}
boost::optional<PullOverPath> ShiftPullOver::plan(const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const double min_jerk = parameters_.minimum_lateral_jerk;
  const double max_jerk = parameters_.maximum_lateral_jerk;
  const int pull_over_sampling_num = parameters_.pull_over_sampling_num;
  const double jerk_resolution = std::abs(max_jerk - min_jerk) / pull_over_sampling_num;

  const auto road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = pull_over_utils::getPullOverLanes(*route_handler);
  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return {};
  }

  // calculate lateral distances from road lane center to goal
  lanelet::ConstLanelet goal_closest_road_lane{};
  lanelet::utils::query::getClosestLanelet(road_lanes, goal_pose, &goal_closest_road_lane);
  const auto road_center_pose =
    lanelet::utils::getClosestCenterPose(goal_closest_road_lane, goal_pose.position);
  const double shoulder_left_bound_to_road_center = util::getSignedDistanceFromShoulderLeftBoundary(
    shoulder_lanes, vehicle_footprint_, road_center_pose);
  const double shoulder_left_bound_to_goal_distance =
    util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, vehicle_footprint_, goal_pose);
  const double road_center_to_goal_distance =
    -shoulder_left_bound_to_road_center + shoulder_left_bound_to_goal_distance;

  for (double lateral_jerk = min_jerk; lateral_jerk <= max_jerk; lateral_jerk += jerk_resolution) {
    const auto pull_over_path = generatePullOverPath(
      road_lanes, shoulder_lanes, goal_pose, lateral_jerk, road_center_to_goal_distance);
    if (!pull_over_path) continue;
    return *pull_over_path;
  }

  return {};
}

PathWithLaneId ShiftPullOver::generateReferencePath(
  const lanelet::ConstLanelets & road_lanes, const Pose & end_pose) const
{
  const auto & route_handler = planner_data_->route_handler;
  const Pose & current_pose = planner_data_->self_pose->pose;
  const double backward_path_length = planner_data_->parameters.backward_path_length;
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double deceleration_interval = parameters_.deceleration_interval;

  const auto current_road_arc_coords = lanelet::utils::getArcCoordinates(road_lanes, current_pose);
  const double s_start = current_road_arc_coords.length - backward_path_length;
  const double s_end = std::max(
    lanelet::utils::getArcCoordinates(road_lanes, end_pose).length,
    s_start + std::numeric_limits<double>::epsilon());
  auto road_lane_reference_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);

  // decelerate velocity linearly to minimum pull over velocity
  // (or keep original velocity if it is lower than pull over velocity)
  for (auto & point : road_lane_reference_path.points) {
    const auto arclength = lanelet::utils::getArcCoordinates(road_lanes, point.point.pose).length;
    const double distance_to_pull_over_start =
      std::clamp(s_end - arclength, 0.0, deceleration_interval);
    const auto decelerated_velocity = static_cast<float>(
      distance_to_pull_over_start / deceleration_interval *
        (point.point.longitudinal_velocity_mps - pull_over_velocity) +
      pull_over_velocity);
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, decelerated_velocity);
  }
  return road_lane_reference_path;
}

boost::optional<PullOverPath> ShiftPullOver::generatePullOverPath(
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & goal_pose, const double lateral_jerk,
  const double road_center_to_goal_distance) const
{
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double after_pull_over_distance = parameters_.after_pull_over_distance;
  const double margin_from_boundary = parameters_.margin_from_boundary;
  // const double vehicle_width = planner_data_->parameters.vehicle_width;

  // generate road lane reference path to goal for caluculating shift start/end pose
  const auto road_lane_reference_path_to_goal = generateReferencePath(road_lanes, goal_pose);

  // calculate shift end pose on road lane
  const double before_shifted_after_pull_over_distance = calcBeforeShiftedArcLegth(
    road_lane_reference_path_to_goal, after_pull_over_distance, road_center_to_goal_distance);
  const auto shift_end_pose_road_lane = motion_utils::calcLongitudinalOffsetPose(
    road_lane_reference_path_to_goal.points, goal_pose.position,
    -before_shifted_after_pull_over_distance);
  if (!shift_end_pose_road_lane) return {};

  // calculate shift start pose on road lane
  const double pull_over_distance = PathShifter::calcLongitudinalDistFromJerk(
    road_center_to_goal_distance, lateral_jerk, pull_over_velocity);

  // generate road lane reference path to shift end
  const auto road_lane_reference_path_to_shift_end = util::resamplePathWithSpline(
    generateReferencePath(road_lanes, *shift_end_pose_road_lane), resample_interval_);
  if (road_lane_reference_path_to_shift_end.points.empty()) return {};

  // calculate shift end pose on shoulder lane
  const double shoulder_left_bound_to_shift_end_road_distance =
    util::getSignedDistanceFromShoulderLeftBoundary(
      shoulder_lanes, vehicle_footprint_, *shift_end_pose_road_lane);
  const double shift_end_road_to_target_distance =
    // -shoulder_left_bound_to_shift_end_road_distance - margin_from_boundary - vehicle_width / 2.0;
    -shoulder_left_bound_to_shift_end_road_distance - margin_from_boundary;
  const Pose shift_end_pose = tier4_autoware_utils::calcOffsetPose(
    *shift_end_pose_road_lane, 0, shift_end_road_to_target_distance, 0);

  const double before_shifted_pull_over_distance = calcBeforeShiftedArcLegth(
    road_lane_reference_path_to_shift_end, pull_over_distance, shift_end_road_to_target_distance);
  const auto shift_start_pose = motion_utils::calcLongitudinalOffsetPose(
    road_lane_reference_path_to_shift_end.points, shift_end_pose_road_lane->position,
    -before_shifted_pull_over_distance);
  if (!shift_start_pose) return {};

  // set path shifter and generate shifted path
  PathShifter path_shifter{};
  path_shifter.setPath(road_lane_reference_path_to_shift_end);
  ShiftLine shift_line{};
  shift_line.start = *shift_start_pose;
  shift_line.end = shift_end_pose;
  shift_line.end_shift_length = shift_end_road_to_target_distance;
  path_shifter.addShiftLine(shift_line);
  ShiftedPath shifted_path{};
  const bool offset_back = true;  // offset front side from reference path
  if (!path_shifter.generate(&shifted_path, offset_back)) return {};

  // interpolate between shift end pose to goal pose
  std::vector<Pose> interpolated_poses =
    interpolatePose(shifted_path.path.points.back().point.pose, goal_pose, resample_interval_);
  for (const auto & pose : interpolated_poses) {
    PathPointWithLaneId p = shifted_path.path.points.back();
    p.point.pose = pose;
    shifted_path.path.points.push_back(p);
  }

  // set goal pose with velocity 0
  {
    PathPointWithLaneId p{};
    p.point.longitudinal_velocity_mps = 0.0;
    p.point.pose = goal_pose;
    p.lane_ids = shifted_path.path.points.back().lane_ids;
    for (const auto & lane : shoulder_lanes) {
      p.lane_ids.push_back(lane.id());
    }
    shifted_path.path.points.push_back(p);

    // insert a pose imediately around to the goal_pose to keep it's orientation after resampling
    PathPointWithLaneId p_next = p;
    p_next.point.pose = tier4_autoware_utils::calcOffsetPose(goal_pose, 0.1, 0, 0);
    shifted_path.path.points.push_back(p_next);
  }

  // check lane departure with road and shoulder lanes
  lanelet::ConstLanelets lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());
  if (lane_departure_checker_.checkPathWillLeaveLane(lanes, shifted_path.path)) return {};

  // check collision
  if (!isSafePath(shifted_path.path)) return {};

  // set lane_id and velocity to shifted_path
  for (size_t i = 0; i < shifted_path.path.points.size() - 1; ++i) {
    auto & point = shifted_path.path.points.at(i);
    // add road lane_ids if not found
    for (const auto id : shifted_path.path.points.back().lane_ids) {
      if (std::find(point.lane_ids.begin(), point.lane_ids.end(), id) == point.lane_ids.end()) {
        point.lane_ids.push_back(id);
      }
    }
    // add shoulder lane_id if not found
    for (const auto & lane : shoulder_lanes) {
      if (
        std::find(point.lane_ids.begin(), point.lane_ids.end(), lane.id()) ==
        point.lane_ids.end()) {
        point.lane_ids.push_back(lane.id());
      }
    }
    // set velocity
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(pull_over_velocity));
  }

  // set pull over path
  PullOverPath pull_over_path{};
  pull_over_path.path = shifted_path.path;
  pull_over_path.partial_paths.push_back(pull_over_path.path);
  pull_over_path.start_pose = path_shifter.getShiftLines().front().start;
  pull_over_path.end_pose = path_shifter.getShiftLines().front().end;
  pull_over_path.debug_poses.push_back(
    road_lane_reference_path_to_goal.points.back().point.pose);     // goal pose on road lane
  pull_over_path.debug_poses.push_back(*shift_end_pose_road_lane);  // shift end pose on road lane

  // check enough distance
  if (!hasEnoughDistance(
        pull_over_path.path, road_lanes, pull_over_path.start_pose, goal_pose,
        pull_over_distance)) {
    return {};
  }

  return pull_over_path;
}

bool ShiftPullOver::hasEnoughDistance(
  const PathWithLaneId & path, const lanelet::ConstLanelets & road_lanes, const Pose & start_pose,
  const Pose & goal_pose, const double pull_over_distance) const
{
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & common_params = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  const size_t ego_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, current_pose, common_params.ego_nearest_dist_threshold,
    common_params.ego_nearest_yaw_threshold);
  const size_t start_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, start_pose, common_params.ego_nearest_dist_threshold,
    common_params.ego_nearest_yaw_threshold);
  const double dist_to_start_pose = motion_utils::calcSignedArcLength(
    path.points, current_pose.position, ego_segment_idx, start_pose.position, start_segment_idx);

  // once stopped, it cannot start again if start_pose is close.
  // so need enough distance to restart
  constexpr double eps_vel = 0.01;
  // dist to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const double dist_to_restart = parameters_.decide_path_distance / 2;
  if (std::abs(current_vel) < eps_vel && dist_to_start_pose < dist_to_restart) {
    return false;
  }
  const double current_to_stop_distance =
    std::pow(current_vel, 2) / parameters_.maximum_deceleration / 2;
  if (dist_to_start_pose < current_to_stop_distance) {
    return false;
  }

  const double road_lane_dist_to_goal = dist_to_start_pose + pull_over_distance;
  if (road_lane_dist_to_goal > util::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  const bool is_in_goal_route_section =
    planner_data_->route_handler->isInGoalRouteSection(road_lanes.back());
  if (
    is_in_goal_route_section &&
    road_lane_dist_to_goal > util::getSignedDistance(current_pose, goal_pose, road_lanes)) {
    return false;
  }

  return true;
}

bool ShiftPullOver::isSafePath(const PathWithLaneId & path) const
{
  if (parameters_.use_occupancy_grid || !occupancy_grid_map_) {
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->hasObstacleOnPath(path, check_out_of_range)) {
      return false;
    }
  }

  if (parameters_.use_object_recognition) {
    if (util::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, path, *(planner_data_->dynamic_object),
          parameters_.object_recognition_collision_check_margin)) {
      return false;
    }
  }

  return true;
}

double ShiftPullOver::calcBeforeShiftedArcLegth(
  const PathWithLaneId & path, const double target_after_arc_length, const double dr)
{
  PathWithLaneId reversed_path{};
  std::reverse_copy(
    path.points.begin(), path.points.end(), std::back_inserter(reversed_path.points));

  double before_arc_length{0.0};
  double after_arc_length{0.0};
  for (const auto & [curvature, segment_length] :
       motion_utils::calcCurvatureAndArcLength(reversed_path.points)) {
    if (after_arc_length + segment_length > target_after_arc_length) {
      const double offset = target_after_arc_length - after_arc_length;
      before_arc_length += offset / (1 + curvature * dr);
      break;
    }

    before_arc_length += segment_length / (1 + curvature * dr);
    after_arc_length += segment_length;
  }

  return before_arc_length;
}

// only two points is supported
std::vector<double> ShiftPullOver::splineTwoPoints(
  std::vector<double> base_s, std::vector<double> base_x, const double begin_diff,
  const double end_diff, std::vector<double> new_s)
{
  const double h = base_s.at(1) - base_s.at(0);

  const double c = begin_diff;
  const double d = base_x.at(0);
  const double a = (end_diff * h - 2 * base_x.at(1) + c * h + 2 * d) / std::pow(h, 3);
  const double b = (3 * base_x.at(1) - end_diff * h - 2 * c * h - 3 * d) / std::pow(h, 2);

  std::vector<double> res;
  for (const auto & s : new_s) {
    const double ds = s - base_s.at(0);
    res.push_back(d + (c + (b + a * ds) * ds) * ds);
  }

  return res;
}

std::vector<Pose> ShiftPullOver::interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval)
{
  std::vector<Pose> interpolated_poses{};  // output

  const std::vector<double> base_s{
    0, tier4_autoware_utils::calcDistance2d(start_pose.position, end_pose.position)};
  const std::vector<double> base_x{start_pose.position.x, end_pose.position.x};
  const std::vector<double> base_y{start_pose.position.y, end_pose.position.y};
  std::vector<double> new_s;

  constexpr double eps = 0.01;
  for (double s = eps; s < base_s.back() - eps; s += resample_interval) {
    new_s.push_back(s);
  }

  const std::vector<double> interpolated_x = splineTwoPoints(
    base_s, base_x, std::cos(tf2::getYaw(start_pose.orientation)),
    std::cos(tf2::getYaw(end_pose.orientation)), new_s);
  const std::vector<double> interpolated_y = splineTwoPoints(
    base_s, base_y, std::sin(tf2::getYaw(start_pose.orientation)),
    std::sin(tf2::getYaw(end_pose.orientation)), new_s);
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    Pose pose{};
    pose = util::lerpByPose(end_pose, start_pose, (base_s.back() - new_s.at(i)) / base_s.back());
    pose.position.x = interpolated_x.at(i);
    pose.position.y = interpolated_y.at(i);
    pose.position.z = end_pose.position.z;
    interpolated_poses.push_back(pose);
  }

  return interpolated_poses;
}
}  // namespace behavior_path_planner
