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

#include "autoware/behavior_path_start_planner_module/freespace_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

// StartPlannerModule::onFreespacePlannerTimer()
//   -> StartPlannerModule::planFreespacePath()
//     -> FreespacePullOut::plan()
//       -> planner_->makePlan(start_pose, end_pose)
//         -> AstarSearch::makePlan()  // 如果配置选择 astar

namespace autoware::behavior_path_planner
{
FreespacePullOut::FreespacePullOut(rclcpp::Node & node, const StartPlannerParameters & parameters)
: PullOutPlannerBase{node, parameters}, velocity_{parameters.freespace_planner_velocity}
{
  autoware::freespace_planning_algorithms::VehicleShape vehicle_shape(
    vehicle_info_, parameters.vehicle_shape_margin);
  if (parameters.freespace_planner_algorithm == "astar") {
    use_back_ = parameters.astar_parameters.use_back;
    planner_ = std::make_unique<AstarSearch>(
      parameters.freespace_planner_common_parameters, vehicle_shape, parameters.astar_parameters,
      node.get_clock());
  } else if (parameters.freespace_planner_algorithm == "rrtstar") {
    use_back_ = true;  // no option for disabling back in rrtstar
    planner_ = std::make_unique<RRTStar>(
      parameters.freespace_planner_common_parameters, vehicle_shape, parameters.rrt_star_parameters,
      node.get_clock());
  }
}

/// @brief 使用自由空间规划器生成起步驶出路径，并把自由空间段拼接回道路中心线。
/// @param start_pose 自车当前位姿，也就是自由空间搜索的起点。
/// @param end_pose 道路中心线上的候选回归位姿，也就是自由空间搜索的终点。
/// @param planner_data 行为路径规划共享数据，至少需要 route_handler、costmap 和路径长度参数。
/// @param planner_debug_data 调试信息输出容器，用于记录本次规划成功或失败原因。
/// @return 成功时返回 PullOutPath；失败时返回 std::nullopt。
///
/// 输出的 PullOutPath 主要包含：
/// - partial_paths: 按前进/倒车方向切分后的路径片段。
/// - start_pose: 本次自由空间规划的起点。
/// - end_pose: 本次自由空间规划接回道路中心线的目标点。
///
/// 这个函数只负责把自由空间规划结果整理成 Start Planner 可执行的 PullOutPath。
/// 真正的 Hybrid A* / RRT* 搜索发生在 planner_->makePlan(start_pose, end_pose) 内部。
std::optional<PullOutPath> FreespacePullOut::plan(
  const Pose & start_pose, const Pose & end_pose,
  const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
{
  // route_handler 负责从 lanelet 路由中取车道中心线和全局目标点。
  const auto & route_handler = planner_data->route_handler;
  // backward_path_length/forward_path_length 决定后续可参考车道和拼接中心线的范围。
  const double backward_path_length = planner_data->parameters.backward_path_length;
  const double forward_path_length = planner_data->parameters.forward_path_length;

  // 将最新 costmap 交给自由空间规划器，Hybrid A* 会在这张栅格地图上做碰撞检查。
  planner_->setMap(*planner_data->costmap);

  try {
    // 从 start_pose 搜索到 end_pose。若配置为 astar，这里会进入 AstarSearch::makePlan()。
    if (!planner_->makePlan(start_pose, end_pose)) {
      planner_debug_data.conditions_evaluation.emplace_back("no path found");
      return {};
    }
  } catch (const std::exception & e) {
    // 起点/终点碰撞、搜索超时或内部规划失败都可能通过异常返回到这里。
    return {};
  }

  // 取得当前路线上的道路车道，后面用于给自由空间路径点补 lane_id。
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  // 取得 pull out 相关车道，并与 road_lanes 合并，保证路边起步段也能关联到车道语义。
  const auto pull_out_lanes =
    start_planner_utils::getPullOutLanes(planner_data, backward_path_length);
  const auto lanes = utils::combineLanelets(road_lanes, pull_out_lanes);

  // 将自由空间规划器输出的几何 waypoint 转成 Autoware 使用的 PathWithLaneId。
  // velocity_ 是自由空间低速驶出的目标速度；lanes 用来为路径点补 lane_id。
  const PathWithLaneId path =
    utils::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_, lanes);

  // 自由空间路径可能包含倒车。先找到前进/倒车切换点，再切成多个 partial path。
  const auto reverse_indices = utils::getReversingIndices(path);
  std::vector<PathWithLaneId> partial_paths = utils::dividePath(path, reverse_indices);

  // 去掉最后一段中已经接近 end_pose 之后的自由空间尾巴，避免与后面中心线路径重复。
  PathWithLaneId & last_path = partial_paths.back();
  const double th_end_distance = 1.0;
  for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
    const size_t index = std::distance(last_path.points.begin(), it);
    if (index == 0) continue;
    const double distance =
      autoware_utils::calc_distance2d(end_pose.position, it->point.pose.position);
    if (distance < th_end_distance) {
      last_path.points.erase(it, last_path.points.end());
      break;
    }
  }

  // 自由空间段只负责到达 end_pose 附近；到达后还需要继续沿道路中心线向前行驶。
  const Pose goal_pose = route_handler->getGoalPose();
  // 从 end_pose 前方 1m 开始取中心线，减少自由空间段和中心线段在拼接处的重叠。
  constexpr double offset_from_end_pose = 1.0;
  // 将 end_pose 投影到当前道路中心线，得到它在道路弧长坐标中的位置。
  const auto arc_position_end = lanelet::utils::getArcCoordinatesOnEgoCenterline(
    road_lanes, end_pose, route_handler->getLaneletMapPtr());
  const double s_start = std::max(arc_position_end.length + offset_from_end_pose, 0.0);
  // 计算中心线拼接的终点弧长；同时判断这段路径终点是否已经是全局 route goal。
  const auto path_end_info =
    autoware::behavior_path_planner::utils::parking_departure::calcEndArcLength(
      s_start, forward_path_length, road_lanes, goal_pose);
  const double s_end = path_end_info.first;
  const bool path_terminal_is_goal = path_end_info.second;

  // 取出 end_pose 之后的道路中心线路径，拼接到最后一个 partial path 上并重新采样。
  const auto road_center_line_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);
  last_path = utils::resamplePathWithSpline(
    utils::combinePath(last_path, road_center_line_path), parameters_.center_line_path_interval);

  // correctDividedPathVelocity 会调整各片段末端速度，先保存原末端速度以便必要时恢复。
  const double original_terminal_velocity = last_path.points.back().point.longitudinal_velocity_mps;
  // 修正切分路径的速度：换挡点/片段末端通常需要停车，保证前进/倒车切换可执行。
  utils::correctDividedPathVelocity(partial_paths);
  if (!path_terminal_is_goal) {
    // 如果末端不是全局目标点，就不需要在当前生成路径末端强制停车。
    last_path.points.back().point.longitudinal_velocity_mps = original_terminal_velocity;
  }

  // 组装对外输出：路径片段 + 本次自由空间起终点。
  PullOutPath pull_out_path{};
  pull_out_path.partial_paths = partial_paths;
  pull_out_path.start_pose = start_pose;
  pull_out_path.end_pose = end_pose;

  // 记录调试结果，供 RViz 或调试表格展示本规划器成功。
  planner_debug_data.conditions_evaluation.emplace_back("success");
  return pull_out_path;
}
}  // namespace autoware::behavior_path_planner
