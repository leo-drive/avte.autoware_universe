// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/util/pull_over/freespace_pull_over.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/util/pull_over/util.hpp"

#include <memory>
#include <vector>

namespace behavior_path_planner
{
FreespacePullOver::FreespacePullOver(
  rclcpp::Node & node, const PullOverParameters & parameters,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PullOverPlannerBase{node, parameters}
{
  velocity_ = node.declare_parameter("pull_over.freespace_parking.velocity", 1.0);

  const double vehicle_shape_margin =
    node.declare_parameter("pull_over.freespace_parking.vehicle_shape_margin", 1.0);
  freespace_planning_algorithms::VehicleShape vehicle_shape(vehicle_info, vehicle_shape_margin);

  const auto algorithm =
    node.declare_parameter("pull_over.freespace_parking.planning_algorithm", "astar");
  if (algorithm == "astar") {
    const auto astar_param = getAstarParam(node);
    use_back_ = astar_param.use_back;
    planner_ = std::make_unique<AstarSearch>(getCommonParam(node), vehicle_shape, astar_param);
  } else if (algorithm == "rrtstar") {
    use_back_ = true;  // no opition for disabling back in rrtstar
    planner_ =
      std::make_unique<RRTStar>(getCommonParam(node), vehicle_shape, getRRTStarParam(node));
  }
}

PlannerCommonParam FreespacePullOver::getCommonParam(rclcpp::Node & node) const
{
  const auto dp = [&node](const std::string & str, auto def_val) {
    std::string name = "pull_over.freespace_parking." + str;
    return node.declare_parameter(name, def_val);
  };

  PlannerCommonParam p{};

  // search configs
  p.time_limit = dp("time_limit", 5000.0);
  p.minimum_turning_radius = dp("minimum_turning_radius", 0.5);
  p.maximum_turning_radius = dp("maximum_turning_radius", 6.0);
  p.turning_radius_size = dp("turning_radius_size", 11);
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = dp("theta_size", 48);
  p.angle_goal_range = dp("angle_goal_range", 6.0);

  p.curve_weight = dp("curve_weight", 1.2);
  p.reverse_weight = dp("reverse_weight", 2.00);
  p.lateral_goal_range = dp("lateral_goal_range", 0.5);
  p.longitudinal_goal_range = dp("longitudinal_goal_range", 2.0);

  // costmap configs
  p.obstacle_threshold = dp("obstacle_threshold", 100);

  return p;
}

AstarParam FreespacePullOver::getAstarParam(rclcpp::Node & node) const
{
  const auto dp = [&node](const std::string & str, auto def_val) {
    std::string name = "pull_over.freespace_parking.astar." + str;
    return node.declare_parameter(name, def_val);
  };

  AstarParam p{};

  p.only_behind_solutions = dp("only_behind_solutions", false);
  p.use_back = dp("use_back", true);
  p.distance_heuristic_weight = dp("distance_heuristic_weight", 1.0);

  return p;
}

RRTStarParam FreespacePullOver::getRRTStarParam(rclcpp::Node & node) const
{
  const auto dp = [&node](const std::string & str, auto def_val) {
    std::string name = "pull_over.freespace_parking.rrtstar." + str;
    return node.declare_parameter(name, def_val);
  };

  RRTStarParam p;

  p.enable_update = dp("enable_update", true);
  p.use_informed_sampling = dp("use_informed_sampling", true);
  p.max_planning_time = dp("max_planning_time", 150.0);
  p.neighbour_radius = dp("neighbour_radius", 8.0);
  p.margin = dp("margin", 0.1);

  return p;
}

boost::optional<PullOverPath> FreespacePullOver::plan(const Pose & goal_pose)
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  planner_->setMap(*planner_data_->costmap);

  // offset goal pose to make staright path near goal for improving parking precision
  // todo: support straight path when using back
  constexpr double straight_distance = 1.0;
  const Pose end_pose =
    use_back_ ? goal_pose
              : tier4_autoware_utils::calcOffsetPose(goal_pose, -straight_distance, 0.0, 0.0);
  const bool found_path = planner_->makePlan(current_pose, end_pose);
  if (!found_path) {
    return {};
  }

  PathWithLaneId path = util::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_);
  const auto reverse_indices = util::getReversingIndices(path);
  std::vector<PathWithLaneId> partial_paths = util::dividePath(path, reverse_indices);

  // remove points which are near the goal
  PathWithLaneId & last_path = partial_paths.back();
  const double th_goal_disntace = 1.0;
  for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
    size_t index = std::distance(last_path.points.begin(), it);
    if (index == 0) continue;
    const double distance =
      tier4_autoware_utils::calcDistance2d(end_pose.position, it->point.pose.position);
    if (distance < th_goal_disntace) {
      last_path.points.erase(it, last_path.points.end());
      break;
    }
  }

  // add PathPointWithLaneId to last path
  auto addPose = [&last_path](const Pose & pose) {
    PathPointWithLaneId p = last_path.points.back();
    p.point.pose = pose;
    last_path.points.push_back(p);
  };

  if (use_back_) {
    addPose(end_pose);
  } else {
    // add interpolated poses
    auto addInterpolatedPoses = [&addPose](const Pose & pose1, const Pose & pose2) {
      constexpr double interval = 0.5;
      std::vector<Pose> interpolated_poses = util::interpolatePose(pose1, pose2, interval);
      for (const auto & pose : interpolated_poses) {
        addPose(pose);
      }
    };
    addInterpolatedPoses(last_path.points.back().point.pose, end_pose);
    addPose(end_pose);
    addInterpolatedPoses(end_pose, goal_pose);
    addPose(goal_pose);
  }

  util::correctDividedPathVelocity(partial_paths);

  const double drivable_area_margin = planner_data_->parameters.vehicle_width;
  for (auto & path : partial_paths) {
    const auto is_driving_forward = motion_utils::isDrivingForward(path.points);
    if (!is_driving_forward) {
      // path points is less than 2
      return {};
    }
    util::generateDrivableArea(
      path, planner_data_->parameters.vehicle_length, planner_data_->parameters.vehicle_width,
      drivable_area_margin, *is_driving_forward);
  }

  PullOverPath pull_over_path{};
  pull_over_path.partial_paths = partial_paths;
  pull_over_path.start_pose = current_pose;
  pull_over_path.end_pose = goal_pose;
  pull_over_path.type = getPlannerType();

  return pull_over_path;
}
}  // namespace behavior_path_planner
