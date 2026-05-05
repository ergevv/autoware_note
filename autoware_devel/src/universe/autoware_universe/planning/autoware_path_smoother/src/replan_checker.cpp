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

#include "autoware/path_smoother/replan_checker.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_smoother/utils/trajectory_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <memory>
#include <vector>

namespace autoware::path_smoother
{
ReplanChecker::ReplanChecker(rclcpp::Node * node, const EgoNearestParam & ego_nearest_param)
: ego_nearest_param_(ego_nearest_param), logger_(node->get_logger().get_child("replan_checker"))
{
  enable_ = node->declare_parameter<bool>("replan.enable");
  max_path_shape_around_ego_lat_dist_ =
    node->declare_parameter<double>("replan.max_path_shape_around_ego_lat_dist");
  max_path_shape_forward_lat_dist_ =
    node->declare_parameter<double>("replan.max_path_shape_forward_lat_dist");
  max_path_shape_forward_lon_dist_ =
    node->declare_parameter<double>("replan.max_path_shape_forward_lon_dist");
  max_ego_moving_dist_ = node->declare_parameter<double>("replan.max_ego_moving_dist");
  max_goal_moving_dist_ = node->declare_parameter<double>("replan.max_goal_moving_dist");
  max_delta_time_sec_ = node->declare_parameter<double>("replan.max_delta_time_sec");
}

void ReplanChecker::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  update_param<bool>(parameters, "replan.enable", enable_);
  update_param<double>(
    parameters, "replan.max_path_shape_around_ego_lat_dist", max_path_shape_around_ego_lat_dist_);
  update_param<double>(
    parameters, "replan.max_path_shape_forward_lat_dist", max_path_shape_forward_lat_dist_);
  update_param<double>(
    parameters, "replan.max_path_shape_forward_lon_dist", max_path_shape_forward_lon_dist_);
  update_param<double>(parameters, "replan.max_ego_moving_dist", max_ego_moving_dist_);
  update_param<double>(parameters, "replan.max_goal_moving_dist", max_goal_moving_dist_);
  update_param<double>(parameters, "replan.max_delta_time_sec", max_delta_time_sec_);
}

/**
 * @brief 检查是否需要重置路径优化器状态
 * 
 * 该函数通过比较当前规划数据与之前保存的轨迹和位姿信息，判断是否发生了以下情况：
 * 1. 自车周围的路径形状发生变化
 * 2. 路径目标点发生变化
 * 3. 自车位姿发生较大变化（超过设定的最大移动距离）
 * 
 * 如果上述任一条件满足，则需要重置优化器状态以重新进行路径规划。
 * 
 * @param planner_data 当前规划器的数据，包含自车位姿、轨迹点等信息
 * @return true 如果需要重置优化器状态
 * @return false 如果不需要重置优化器状态
 */
bool ReplanChecker::isResetRequired(const PlannerData & planner_data) const
{
  const auto & p = planner_data;

  const bool reset_required = [&]() {
    // 检查前一次轨迹点和自车位姿指针是否有效，无效则必须重置
    if (!prev_traj_points_ptr_ || !prev_ego_pose_ptr_) {
      return true;
    }
    const auto & prev_traj_points = *prev_traj_points_ptr_;

    // 检查自车周围的路径形状是否发生变化，比较自车在之前轨迹和当前轨迹上的横向偏移量差异
    if (isPathAroundEgoChanged(planner_data, prev_traj_points)) {
      RCLCPP_DEBUG(
        logger_, "Replan with resetting optimization since path shape around ego changed.");
      return true;
    }

    // 检查路径目标点是否发生变化，两次轨迹的目标点距离是不是较大
    if (isPathGoalChanged(planner_data, prev_traj_points)) {
      RCLCPP_DEBUG(logger_, "Replan with resetting optimization since path goal changed.");
      return true;
    }

    // 检查自车位姿是否丢失或在仿真中被重新指定（当前位置与上次位置距离过大）
    const double delta_dist =
      autoware_utils::calc_distance2d(p.ego_pose, prev_ego_pose_ptr_->position);
    if (max_ego_moving_dist_ < delta_dist) {
      RCLCPP_DEBUG(
        logger_,
        "Replan with resetting optimization since current ego pose is far from previous ego pose.");
      return true;
    }

    return false;
  }();

  return reset_required;
}

/**
 * @brief 检查是否需要进行路径重新规划
 * 
 * 该函数通过以下条件判断是否需要触发重新规划：
 * 1. 功能是否启用（未启用时始终返回true）
 * 2. 之前保存的重新规划时间和轨迹点数据是否有效
 * 3. 自上次重新规划以来经过的时间是否超过阈值
 * 4. 路径前方形状是否发生显著变化
 * 
 * @param planner_data 当前规划器的数据，包含自车位姿、轨迹点等信息
 * @param current_time 当前时间戳，用于计算与上次重新规划的时间间隔
 * @return true 如果需要进行重新规划
 * @return false 如果不需要进行重新规划
 */
bool ReplanChecker::isReplanRequired(
  const PlannerData & planner_data, const rclcpp::Time & current_time) const
{
  if (!enable_) return true;

  // 验证之前保存的重新规划时间和轨迹点数据的有效性
  if (!prev_replanned_time_ptr_ || !prev_traj_points_ptr_) return true;
  const auto & prev_traj_points = *prev_traj_points_ptr_;

  // 检查自上次重新规划以来经过的时间是否超过最大允许时间间隔
  const double delta_time_sec = (current_time - *prev_replanned_time_ptr_).seconds();
  if (max_delta_time_sec_ < delta_time_sec) return true;

  // 检查路径前方形状是否发生显著变化
  if (isPathForwardChanged(planner_data, prev_traj_points)) {
    RCLCPP_INFO(logger_, "Replan since path forward shape changed.");
    return true;
  }

  return false;
}

void ReplanChecker::updateData(
  const PlannerData & planner_data, const bool is_replan_required,
  const rclcpp::Time & current_time)
{
  const auto & p = planner_data;

  // update previous information required in this function
  prev_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(p.traj_points);
  prev_ego_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(p.ego_pose);

  // update previous information required in this function
  if (is_replan_required) {
    prev_replanned_time_ptr_ = std::make_shared<rclcpp::Time>(current_time);
  }
}

/**
 * @brief 检查自车周围的路径形状是否发生显著变化
 * 
 * 通过比较自车在之前轨迹和当前轨迹上的横向偏移量差异，
 * 判断路径在自车附近的形状是否发生了需要重新规划的变化。
 * 
 * @param planner_data 规划器数据，包含当前轨迹点和自车位姿信息
 * @param prev_traj_points 之前的轨迹点序列
 * @return true 如果自车周围的横向偏移变化超过阈值，需要重新规划
 * @return false 如果横向偏移变化在允许范围内，不需要重新规划
 */
bool ReplanChecker::isPathAroundEgoChanged(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const
{
  const auto & p = planner_data;

  // 计算自车相对于之前轨迹的横向偏移量
  const auto prev_ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(prev_traj_points, p.ego_pose, ego_nearest_param_);
  const double prev_ego_lat_offset = autoware::motion_utils::calcLateralOffset(
    prev_traj_points, p.ego_pose.position, prev_ego_seg_idx);

  // 计算自车相对于当前轨迹的横向偏移量
  const auto ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(p.traj_points, p.ego_pose, ego_nearest_param_);
  const double ego_lat_offset =
    autoware::motion_utils::calcLateralOffset(p.traj_points, p.ego_pose.position, ego_seg_idx);

  // 判断横向偏移量的变化是否超过阈值
  const double diff_ego_lat_offset = prev_ego_lat_offset - ego_lat_offset;
  if (std::abs(diff_ego_lat_offset) < max_path_shape_around_ego_lat_dist_) {
    return false;
  }

  return true;
}

/**
 * @brief 检查路径是否向前发生了显著变化
 * 
 * 该函数通过比较当前轨迹点和前一次轨迹点在前方多个纵向距离点处的横向偏移，
 * 判断路径形状是否发生了需要重新规划的变化。如果在任意检查点处的横向偏移
 * 超过阈值，则认为路径发生了显著变化。
 * 
 * @param planner_data 规划器数据，包含当前轨迹点和自车姿态等信息
 * @param prev_traj_points 前一次的轨迹点序列
 * @return true 如果路径向前发生了显著变化（横向偏移超过阈值）
 * @return false 如果路径未发生显著变化
 */
bool ReplanChecker::isPathForwardChanged(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const
{
  const auto & p = planner_data;

  // 计算前一次轨迹中自车所在的路段索引
  const size_t prev_ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(prev_traj_points, p.ego_pose, ego_nearest_param_);

  // 在多个纵向距离点上检查横向偏移是否超过阈值
  constexpr double lon_dist_interval = 10.0;
  for (double lon_dist = lon_dist_interval; lon_dist <= max_path_shape_forward_lon_dist_;
       lon_dist += lon_dist_interval) {
    const auto prev_forward_point = autoware::motion_utils::calcLongitudinalOffsetPoint(
      prev_traj_points, prev_ego_seg_idx, lon_dist);
    if (!prev_forward_point) {
      continue;
    }

    // 计算当前轨迹点相对于前向参考点的横向偏移
    const auto forward_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(p.traj_points, *prev_forward_point);
    const double forward_lat_offset = autoware::motion_utils::calcLateralOffset(
      p.traj_points, *prev_forward_point, forward_seg_idx);
    if (max_path_shape_forward_lat_dist_ < std::abs(forward_lat_offset)) {
      return true;
    }
  }

  return false;
}

bool ReplanChecker::isPathGoalChanged(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & prev_traj_points) const
{
  const auto & p = planner_data;

  // check if the vehicle is stopping
  constexpr double min_vel = 1e-3;
  if (min_vel < std::abs(p.ego_vel)) {
    return false;
  }

  const double goal_moving_dist =
    autoware_utils::calc_distance2d(p.traj_points.back(), prev_traj_points.back());
  if (goal_moving_dist < max_goal_moving_dist_) {
    return false;
  }

  return true;
}
}  // namespace autoware::path_smoother
