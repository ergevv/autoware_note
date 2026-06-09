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

#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"
#include "tf2/utils.h"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
namespace
{
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByUniformCircle(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num,
  const double radius_ratio)
{
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = std::hypot(
                          vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
                          vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
                        radius_ratio;
  const std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_info.rear_overhang_m);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByBicycleModel(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num,
  const double rear_radius_ratio, const double front_radius_ratio)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  // 1st circle (rear wheel)
  const double rear_radius =
    vehicle_info.vehicle_width_m / 2.0 + lateral_offset * rear_radius_ratio;
  const double rear_lon_offset = 0.0;

  // 2nd circle (front wheel)
  const double front_radius =
    std::hypot(
      vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
      vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
    front_radius_ratio;

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  const double front_lon_offset =
    unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_info.rear_overhang_m;

  return {{rear_radius, front_radius}, {rear_lon_offset, front_lon_offset}};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByFittingUniformCircle(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = vehicle_info.vehicle_width_m / 2.0 + lateral_offset;
  std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length =
    vehicle_info.vehicle_length_m / static_cast<double>(circle_num - 1);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(unit_lon_length * i - vehicle_info.rear_overhang_m);
    radiuses.push_back(radius);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> extractBounds(
  const std::vector<ReferencePoint> & ref_points, const size_t l_idx, const double offset)
{
  Eigen::VectorXd ub_vec(ref_points.size());
  Eigen::VectorXd lb_vec(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ub_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).upper_bound + offset;
    lb_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).lower_bound - offset;
  }
  return {ub_vec, lb_vec};
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

bool isLeft(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & target_pos)
{
  const double base_theta = tf2::getYaw(pose.orientation);
  const double target_theta = autoware_utils::calc_azimuth_angle(pose.position, target_pos);
  const double diff_theta = autoware_utils::normalize_radian(target_theta - base_theta);
  return diff_theta > 0;
}

// NOTE: Regarding boundary's sign, left is positive, and right is negative
double calcLateralDistToBounds(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & bound,
  const double additional_offset, const bool is_left_bound = true)
{
  constexpr double max_lat_offset_for_left = 5.0;
  constexpr double min_lat_offset_for_left = -5.0;

  const double max_lat_offset = is_left_bound ? max_lat_offset_for_left : -max_lat_offset_for_left;
  const double min_lat_offset = is_left_bound ? min_lat_offset_for_left : -min_lat_offset_for_left;
  const auto max_lat_offset_point =
    autoware_utils::calc_offset_pose(pose, 0.0, max_lat_offset, 0.0).position;
  const auto min_lat_offset_point =
    autoware_utils::calc_offset_pose(pose, 0.0, min_lat_offset, 0.0).position;

  double closest_dist_to_bound = max_lat_offset;
  for (size_t i = 0; i < bound.size() - 1; ++i) {
    const auto intersect_point = autoware_utils::intersect(
      min_lat_offset_point, max_lat_offset_point, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      const bool is_point_left = isLeft(pose, *intersect_point);
      const double dist_to_bound =
        autoware_utils::calc_distance2d(pose.position, *intersect_point) *
        (is_point_left ? 1.0 : -1.0);

      // the bound which is closest to the centerline will be chosen
      const double tmp_dist_to_bound =
        is_left_bound ? dist_to_bound - additional_offset : dist_to_bound + additional_offset;
      if (std::abs(tmp_dist_to_bound) < std::abs(closest_dist_to_bound)) {
        closest_dist_to_bound = tmp_dist_to_bound;
      }
    }
  }

  return closest_dist_to_bound;
}
}  // namespace

MPTOptimizer::MPTParam::MPTParam(
  rclcpp::Node * node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  {  // option
    steer_limit_constraint = node->declare_parameter<bool>("mpt.option.steer_limit_constraint");
    enable_warm_start = node->declare_parameter<bool>("mpt.option.enable_warm_start");
    enable_manual_warm_start = node->declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    enable_optimization_validation =
      node->declare_parameter<bool>("mpt.option.enable_optimization_validation");
    mpt_visualize_sampling_num = node->declare_parameter<int>("mpt.option.visualize_sampling_num");
  }

  {  // common
    num_points = node->declare_parameter<int>("mpt.common.num_points");
    delta_arc_length = node->declare_parameter<double>("mpt.common.delta_arc_length");
  }

  // kinematics
  max_steer_rad = vehicle_info.max_steer_angle_rad;

  // NOTE: By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
  //       The 0.8 scale is adopted as it performed the best.
  // optimization_center_offset 是从参考位姿向车辆前方平移的距离，目标函数会在这个前向点
  // 评价横向误差。值越大，同样的 yaw_error 会造成越大的前方横向偏移：
  //   lat_error + offset * yaw_error
  constexpr double default_wheel_base_ratio = 0.8;
  optimization_center_offset = node->declare_parameter<double>(
    "mpt.kinematics.optimization_center_offset",
    vehicle_info.wheel_base_m * default_wheel_base_ratio);

  {  // clearance
    hard_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.hard_clearance_from_road");
    soft_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.soft_clearance_from_road");
  }

  {  // weight
    soft_collision_free_weight =
      node->declare_parameter<double>("mpt.weight.soft_collision_free_weight");

    lat_error_weight = node->declare_parameter<double>("mpt.weight.lat_error_weight");
    yaw_error_weight = node->declare_parameter<double>("mpt.weight.yaw_error_weight");
    yaw_error_rate_weight = node->declare_parameter<double>("mpt.weight.yaw_error_rate_weight");
    steer_input_weight = node->declare_parameter<double>("mpt.weight.steer_input_weight");
    steer_rate_weight = node->declare_parameter<double>("mpt.weight.steer_rate_weight");

    terminal_lat_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_lat_error_weight");
    terminal_yaw_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_yaw_error_weight");
    goal_lat_error_weight = node->declare_parameter<double>("mpt.weight.goal_lat_error_weight");
    goal_yaw_error_weight = node->declare_parameter<double>("mpt.weight.goal_yaw_error_weight");
  }

  {  // avoidance
    max_longitudinal_margin_for_bound_violation =
      node->declare_parameter<double>("mpt.avoidance.max_longitudinal_margin_for_bound_violation");
    max_bound_fixing_time = node->declare_parameter<double>("mpt.avoidance.max_bound_fixing_time");
    max_avoidance_cost = node->declare_parameter<double>("mpt.avoidance.max_avoidance_cost");
    avoidance_cost_margin = node->declare_parameter<double>("mpt.avoidance.avoidance_cost_margin");
    avoidance_cost_band_length =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_band_length");
    avoidance_cost_decrease_rate =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_decrease_rate");
    min_drivable_width = node->declare_parameter<double>("mpt.avoidance.min_drivable_width");

    avoidance_lat_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.lat_error_weight");
    avoidance_yaw_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.yaw_error_weight");
    avoidance_steer_input_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.steer_input_weight");
  }

  {  // collision free constraints
    l_inf_norm = node->declare_parameter<bool>("mpt.collision_free_constraints.option.l_inf_norm");
    soft_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.soft_constraint");
    hard_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.hard_constraint");
  }

  {  // vehicle_circles
    // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
    vehicle_circles_method =
      node->declare_parameter<std::string>("mpt.collision_free_constraints.vehicle_circles.method");

    // uniform circles
    vehicle_circles_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
    vehicle_circles_uniform_circle_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio");

    // bicycle model
    vehicle_circles_bicycle_model_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_"
      "calculation");
    vehicle_circles_bicycle_model_rear_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.rear_radius_ratio");
    vehicle_circles_bicycle_model_front_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.front_radius_ratio");

    // fitting uniform circles
    vehicle_circles_fitting_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num");
  }

  {  // validation
    max_validation_lat_error = node->declare_parameter<double>("mpt.validation.max_lat_error");
    max_validation_yaw_error = node->declare_parameter<double>("mpt.validation.max_yaw_error");
  }
}

void MPTOptimizer::MPTParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  {  // option
    update_param<bool>(parameters, "mpt.option.steer_limit_constraint", steer_limit_constraint);
    update_param<bool>(parameters, "mpt.option.enable_warm_start", enable_warm_start);
    update_param<bool>(parameters, "mpt.option.enable_manual_warm_start", enable_manual_warm_start);
    update_param<bool>(
      parameters, "mpt.option.enable_optimization_validation", enable_optimization_validation);
    update_param<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num);
  }

  // common
  update_param<int>(parameters, "mpt.common.num_points", num_points);
  update_param<double>(parameters, "mpt.common.delta_arc_length", delta_arc_length);

  // kinematics
  update_param<double>(
    parameters, "mpt.kinematics.optimization_center_offset", optimization_center_offset);

  // collision_free_constraints
  update_param<bool>(parameters, "mpt.collision_free_constraints.option.l_inf_norm", l_inf_norm);
  update_param<bool>(
    parameters, "mpt.collision_free_constraints.option.soft_constraint", soft_constraint);
  update_param<bool>(
    parameters, "mpt.collision_free_constraints.option.hard_constraint", hard_constraint);

  {  // vehicle_circles
    update_param<std::string>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.method", vehicle_circles_method);

    // uniform circles
    update_param<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
      vehicle_circles_uniform_circle_num);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
      vehicle_circles_uniform_circle_radius_ratio);

    // bicycle model
    update_param<int>(
      parameters,
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation",
      vehicle_circles_bicycle_model_num);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio",
      vehicle_circles_bicycle_model_rear_radius_ratio);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.front_radius_ratio",
      vehicle_circles_bicycle_model_front_radius_ratio);

    // fitting uniform circles
    update_param<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num",
      vehicle_circles_fitting_uniform_circle_num);
  }

  {  // clearance
    update_param<double>(
      parameters, "mpt.clearance.hard_clearance_from_road", hard_clearance_from_road);
    update_param<double>(
      parameters, "mpt.clearance.soft_clearance_from_road", soft_clearance_from_road);
  }

  {  // weight
    update_param<double>(
      parameters, "mpt.weight.soft_collision_free_weight", soft_collision_free_weight);

    update_param<double>(parameters, "mpt.weight.lat_error_weight", lat_error_weight);
    update_param<double>(parameters, "mpt.weight.yaw_error_weight", yaw_error_weight);
    update_param<double>(parameters, "mpt.weight.yaw_error_rate_weight", yaw_error_rate_weight);
    update_param<double>(parameters, "mpt.weight.steer_input_weight", steer_input_weight);
    update_param<double>(parameters, "mpt.weight.steer_rate_weight", steer_rate_weight);

    update_param<double>(
      parameters, "mpt.weight.terminal_lat_error_weight", terminal_lat_error_weight);
    update_param<double>(
      parameters, "mpt.weight.terminal_yaw_error_weight", terminal_yaw_error_weight);
    update_param<double>(parameters, "mpt.weight.goal_lat_error_weight", goal_lat_error_weight);
    update_param<double>(parameters, "mpt.weight.goal_yaw_error_weight", goal_yaw_error_weight);
  }

  {  // avoidance
    update_param<double>(
      parameters, "mpt.avoidance.max_longitudinal_margin_for_bound_violation",
      max_longitudinal_margin_for_bound_violation);
    update_param<double>(parameters, "mpt.avoidance.max_bound_fixing_time", max_bound_fixing_time);
    update_param<double>(parameters, "mpt.avoidance.min_drivable_width", min_drivable_width);
    update_param<double>(parameters, "mpt.avoidance.max_avoidance_cost", max_avoidance_cost);
    update_param<double>(parameters, "mpt.avoidance.avoidance_cost_margin", avoidance_cost_margin);
    update_param<double>(
      parameters, "mpt.avoidance.avoidance_cost_band_length", avoidance_cost_band_length);
    update_param<double>(
      parameters, "mpt.avoidance.avoidance_cost_decrease_rate", avoidance_cost_decrease_rate);

    update_param<double>(
      parameters, "mpt.avoidance.weight.lat_error_weight", avoidance_lat_error_weight);
    update_param<double>(
      parameters, "mpt.avoidance.weight.yaw_error_weight", avoidance_yaw_error_weight);
    update_param<double>(
      parameters, "mpt.avoidance.weight.steer_input_weight", avoidance_steer_input_weight);
  }

  {  // validation
    update_param<double>(parameters, "mpt.validation.max_lat_error", max_validation_lat_error);
    update_param<double>(parameters, "mpt.validation.max_yaw_error", max_validation_yaw_error);
  }
}

MPTOptimizer::MPTOptimizer(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  vehicle_info_(vehicle_info),
  traj_param_(traj_param),
  debug_data_ptr_(debug_data_ptr),
  time_keeper_(time_keeper),
  logger_(node->get_logger().get_child("mpt_optimizer"))
{
  // initialize mpt param
  mpt_param_ = MPTParam(node, vehicle_info);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;

  // state equation generator
  state_equation_generator_ =
    StateEquationGenerator(vehicle_info_.wheel_base_m, mpt_param_.max_steer_rad, time_keeper_);

  // osqp solver
  osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(osqp_epsilon_);

  // publisher
  debug_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_ref_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_traj", 1);
}

void MPTOptimizer::updateVehicleCircles()
{
  const auto & p = mpt_param_;

  if (p.vehicle_circles_method == "uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByUniformCircle(
        vehicle_info_, p.vehicle_circles_uniform_circle_num,
        p.vehicle_circles_uniform_circle_radius_ratio);
  } else if (p.vehicle_circles_method == "bicycle_model") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByBicycleModel(
        vehicle_info_, p.vehicle_circles_bicycle_model_num,
        p.vehicle_circles_bicycle_model_rear_radius_ratio,
        p.vehicle_circles_bicycle_model_front_radius_ratio);
  } else if (p.vehicle_circles_method == "fitting_uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByFittingUniformCircle(
        vehicle_info_, p.vehicle_circles_fitting_uniform_circle_num);
  } else {
    throw std::invalid_argument("mpt_param_.vehicle_circles_method is invalid.");
  }

  debug_data_ptr_->vehicle_circle_radiuses = vehicle_circle_radiuses_;
  debug_data_ptr_->vehicle_circle_longitudinal_offsets = vehicle_circle_longitudinal_offsets_;
}

void MPTOptimizer::initialize(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void MPTOptimizer::resetPreviousData()
{
  prev_ref_points_ptr_ = nullptr;
  prev_optimized_traj_points_ptr_ = nullptr;
}

void MPTOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  mpt_param_.onParam(parameters);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;
}
/**
 * @brief 使用模型预测控制(MPT)优化轨迹
 *
 * 该函数基于参考点和规划数据,通过求解二次规划(QP)问题来优化车辆的转向角,
 * 生成平滑且安全的轨迹。优化过程考虑了车辆运动学约束、碰撞避免和轨迹平滑性。
 *
 * 优化流程:
 * 1. 计算参考点序列
 * 2. 构建状态方程矩阵 (B, W)
 * 3. 计算代价函数权重矩阵 (Q, R)
 * 4. 构建目标函数矩阵
 * 5. 构建约束条件矩阵
 * 6. 求解QP问题得到最优转向角
 * 7. 将优化结果转换为轨迹点并进行验证
 * 8. 发布调试轨迹并更新内部状态
 * 
 * @param planner_data 规划器数据,包含轨迹点、车辆状态、环境信息等
 * @return std::optional<std::vector<TrajectoryPoint>> 优化后的轨迹点序列
 *         - 成功时返回优化后的轨迹点向量
 *         - 失败时返回std::nullopt,可能原因包括:
 *           * 参考点数量不足(< 2)
 *           * QP问题求解失败
 *           * 横向或航向误差过大导致验证失败
 */
std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::optimizeTrajectory(
  const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = planner_data;
  const auto & traj_points = p.traj_points;

  // 计算参考点序列,作为优化的基准路径
  auto ref_points = calcReferencePoints(planner_data, traj_points);
  if (ref_points.size() < 2) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since ref_points size is less than 2.");
    return std::nullopt;
  }

  // 构建状态方程矩阵 B 和 W,其中状态变量 x = B*u + W
  // u 为控制输入(转向角),该方程描述系统状态随控制输入的变化关系
  const auto mpt_mat = state_equation_generator_.calcMatrix(ref_points);

  // 计算代价函数的权重矩阵 Q 和 R
  // 目标函数 J(x,u) = x^T*Q*x + u^T*R*u,用于平衡跟踪精度和控制平滑性
  const auto val_mat = calcValueMatrix(ref_points, traj_points);

  // 根据状态方程矩阵和权重矩阵构建最终的目标函数矩阵
  const auto obj_mat = calcObjectiveMatrix(mpt_mat, val_mat, ref_points);

  // 构建约束条件矩阵,包括车辆运动学约束、碰撞避免约束等
  const auto const_mat = calcConstraintMatrix(mpt_mat, ref_points);

  // 求解二次规划问题,得到最优的转向角序列
  const auto optimized_variables = calcOptimizedSteerAngles(ref_points, obj_mat, const_mat);
  if (!optimized_variables) {
    RCLCPP_WARN(logger_, "return std::nullopt since could not solve qp");

    return std::nullopt;
  }

  // 将优化得到的控制变量转换为轨迹点,并验证横向误差和航向误差是否在允许范围内
  auto mpt_traj_points = calcMPTPoints(ref_points, *optimized_variables, mpt_mat);
  if (!mpt_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since lateral or yaw error is too large.");
    return std::nullopt;
  }

  // 发布调试用的轨迹信息,用于可视化和调试
  publishDebugTrajectories(p.header, ref_points, *mpt_traj_points);

  debug_data_ptr_->ref_points = ref_points;
  prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(ref_points);
  prev_optimized_traj_points_ptr_ =
    std::make_shared<std::vector<TrajectoryPoint>>(*mpt_traj_points);

  return mpt_traj_points;
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::getPrevOptimizedTrajectoryPoints() const
{
  if (prev_optimized_traj_points_ptr_) {
    return *prev_optimized_traj_points_ptr_;
  }
  return std::nullopt;
}

/**
 * @brief 计算参考点序列，用于MPT（Model Predictive Trajectory）优化器
 * 
 * 该函数基于平滑后的轨迹点和规划数据，生成一系列参考点。这些参考点包含了位置、方向、曲率、
 * 边界等完整信息，作为后续路径优化的基础。处理流程包括重采样、裁剪、插值、以及各类属性的计算。
 * 
 * 主要处理步骤：
 * 1. 对平滑轨迹点进行重采样并转换为ReferencePoint类型
 * 2. 前后向裁剪轨迹并添加余量，进行样条插值
 * 3. 计算每个参考点的方向和曲率
 * 4. 后向裁剪到指定长度
 * 5. 更新固定点并重新采样
 * 6. 更新道路边界和车辆圆约束信息（包括 beta 和 bounds_on_constraints）
 * 7. 更新弧长间隔
 * 8. 更新额外信息（alpha 和避障代价）
 * 9. 前向裁剪到目标点数
 * 
 * @param planner_data 规划器数据，包含自车位姿、速度、左右边界等关键信息
 * @param smoothed_points 经过平滑处理的轨迹点序列，作为参考路径的输入
 * 
 * @return std::vector<ReferencePoint> 计算完成的参考点序列，包含完整的几何和约束信息
 */
std::vector<ReferencePoint> MPTOptimizer::calcReferencePoints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = planner_data;

  // MPT 只优化自车附近的一段局部窗口。
  // forward_traj_length 是前向优化视野长度；backward_traj_length 用来保留自车后方点，
  // 使输出轨迹能和车后历史轨迹平滑衔接。
  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  // 1. 按 MPT 的离散间隔重采样输入轨迹，并转换成 ReferencePoint。
  // ReferencePoint 不只是普通轨迹点，它还会承载曲率、边界、固定点状态、alpha/beta、
  // 上一帧优化结果等后续优化需要的附加信息。
  time_keeper_->start_track("resampleReferencePoints");
  auto ref_points = [&]() {
    const auto resampled_smoothed_points =
      trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
        smoothed_points, mpt_param_.delta_arc_length);
    return trajectory_utils::convertToReferencePoints(resampled_smoothed_points);
  }();
  time_keeper_->end_track("resampleReferencePoints");

  // 2. 先裁剪一个带前后余量的临时窗口。
  // 这段余量不属于最终优化视野，而是给样条插值提供足够邻域点，避免在真实裁剪边界附近
  // 计算 yaw/curvature 时出现端点误差。cropPoints 可能在自车附近生成新的起点，因此每次
  // 裁剪后都要重新计算 ego segment 和样条。
  constexpr double tmp_margin = 10.0;
  size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);
  ref_points = autoware::motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length + tmp_margin);

  // 构造样条前先去除重复点。重复位置会导致样条插值和曲率计算不稳定。
  ref_points = trajectory_utils::sanitizePoints(ref_points);
  autoware::interpolation::SplineInterpolationPoints2d ref_points_spline(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 3. 基于样条重新计算参考 yaw 和曲率，而不是直接使用输入点姿态。
  // MPT 的车辆模型、转向限制和调试信息都会依赖这些几何量。
  updateOrientation(ref_points, ref_points_spline);
  updateCurvature(ref_points, ref_points_spline);

  // 4. yaw/curvature 计算完成后，去掉临时后向余量。
  // 前向余量暂时保留，因为 updateFixedPoint 可能会插入/重采样前部固定点，
  // updateVehicleBounds 也需要查询车辆圆纵向偏移位置处的样条位姿。
  ref_points = autoware::motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length);
  ref_points_spline = autoware::interpolation::SplineInterpolationPoints2d(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 5. 用上一帧优化结果约束当前帧前部点，保证时间连续性。
  // updateFixedPoint 可能会用上一帧优化状态替换/插入前部点，然后重新采样。
  // 这一步必须在后向裁剪之后执行，因为固定点定义在当前优化窗口的前端。
  updateFixedPoint(ref_points);
  ref_points = trajectory_utils::sanitizePoints(ref_points);
  ref_points_spline = autoware::interpolation::SplineInterpolationPoints2d(ref_points);

  // 6. 为每个参考点附加可行驶区域约束。
  // updateBounds 计算参考点中心处的左右道路/障碍边界。
  // updateVehicleBounds 会把这些边界插值到每个车辆碰撞圆的位置，并计算 beta；
  // beta 用于后续圆心横向位置的线性化约束。
  // 注意：此后不能再重采样，因为 bounds_on_constraints 是逐点绑定的，重采样函数不会插值这些边界。
  updateBounds(ref_points, p.left_bound, p.right_bound, p.ego_pose, p.ego_vel);
  updateVehicleBounds(ref_points, ref_points_spline);

  // 7. 记录每个点到下一个点的实际弧长间隔。状态方程会把该值作为空间离散步长 ds。
  updateDeltaArcLength(ref_points);

  // 8. 计算 alpha 和避障代价。
  // alpha 用于把横向误差评估点前移到优化中心；避障代价由边界宽度/障碍约束推导，
  // 再沿路径扩散，用来调整跟踪和转向权重。因此必须在 bounds 和 delta_arc_length 之后执行。
  updateExtraPoints(ref_points);

  // 9. 去掉临时前向余量，只保留 MPT 需要的目标点数。
  // 后续优化矩阵维度都由这个最终点数决定。
  // ref_points = autoware::motion_utils::cropForwardPoints(
  //   ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length);
  if (static_cast<size_t>(mpt_param_.num_points) < ref_points.size()) {
    ref_points.resize(mpt_param_.num_points);
  }

  return ref_points;
}

void MPTOptimizer::updateOrientation(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto yaw_vec = ref_points_spline.getSplineInterpolatedYaws();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw_vec.at(i));
  }
}

void MPTOptimizer::updateCurvature(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto curvature_vec = ref_points_spline.getSplineInterpolatedCurvatures();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).curvature = curvature_vec.at(i);
  }
}

/**
 * @brief 用上一帧优化结果设置当前优化窗口前端的固定点
 *
 * 用上一帧 MPT 的优化结果，固定当前帧优化窗口最前面的 1 个或 2 个点，保证当前优化轨迹和上一帧轨迹连续，避免自车附近轨迹抖动。
 *
 * 固定点是指 `fixed_kinematic_state` 被设置的 ReferencePoint。后续
 * calcConstraintMatrix() 会为这些点添加等式约束：
 *
 *   X_i = fixed_kinematic_state_i = [lat_i, yaw_i]
 *
 * 因此它们的横向误差和航向误差不会被当前帧优化改变。这样可以让当前帧轨迹从上一帧
 * 已发布/已优化的轨迹平滑接上，避免自车附近轨迹每帧重新优化后产生跳变。
 *
 * 哪些点可以成为固定点：
 * - 必须存在上一帧参考点 `prev_ref_points_ptr_`，否则没有可继承的固定点。
 * - 使用当前 ref_points.front() 在上一帧参考点中寻找对应的前端点 `idx`。
 * - 若找到的上一帧点与当前窗口前端足够接近，则当前首点继承该上一帧点。
 * - 若 `idx != 0`，为了连同“由两个前端点决定的方向”也固定住，会额外插入上一帧
 *   `idx - 1` 点，最终固定当前窗口的前两个点。
 *
 * 注意：这里固定的是优化状态 `[lat, yaw]`，不是直接把整条轨迹都锁死。固定点之外的
 * 后续点仍会参与 MPT 优化。
 */
void MPTOptimizer::updateFixedPoint(std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!prev_ref_points_ptr_) {
    // 没有上一帧优化结果，当前帧不能生成固定点。
    return;
  }

  // 在上一帧参考点中寻找与当前窗口前端 ref_points.front() 对应的点。
  // updateFrontPointForFix() 会根据这个上一帧点更新当前窗口前端：
  // - 如果两者距离小于 delta_arc_length，只替换当前首点；
  // - 如果距离较大，则把上一帧点插入到当前序列最前面；
  // 返回值 idx 是被选中的上一帧固定点索引。
  const auto idx = trajectory_utils::updateFrontPointForFix(
    ref_points, *prev_ref_points_ptr_, mpt_param_.delta_arc_length, ego_nearest_param_);

  // 如果 idx 为空，说明上一帧点相对当前窗口位置关系异常，updateFrontPointForFix()
  // 没有插入固定点。此时保留当前首点作为“弱固定候选”，后面只固定当前首点。
  // 若 idx 有效，则 front_point 是 updateFrontPointForFix() 处理后的窗口首点。
  // 该点需要在重采样后恢复，因为重采样只插值 pose/velocity/curvature，会丢失精确固定状态。
  const auto front_point = ref_points.front();

  if (idx && *idx != 0) {
    // idx != 0 时，上一帧固定点前面还有一个点。
    // 只固定一个点只能固定位置，无法稳定窗口前端由相邻点决定的方向。
    // 因此额外插入上一帧 idx - 1 点，并在重采样后固定前两个点：
    //   ref_points[0] <- prev_ref_points[idx - 1]
    //   ref_points[1] <- prev_ref_points[idx]
    ref_points.insert(ref_points.begin(), prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1));

    // 重新采样，使参考点间隔恢复为 delta_arc_length。
    // NOTE: resampleReferencePoints 只插值 pose/velocity/curvature，不会保留
    // fixed_kinematic_state，所以后面必须显式写回固定点状态。
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    // 写回上一帧的精确 pose 和 optimized_kinematic_state。
    // 当前输入路径和上一帧优化结果之间可能存在横向差异，若使用重采样后的 pose，会把固定点
    // 悄悄移动到当前路径上，削弱“继承上一帧轨迹”的目的。因此固定点 pose 也恢复为上一帧值。
    const auto & prev_ref_front_point = prev_ref_points_ptr_->at(*idx);
    const auto & prev_ref_prev_front_point = prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1);

    ref_points.front().pose = prev_ref_prev_front_point.pose;
    ref_points.front().fixed_kinematic_state = prev_ref_prev_front_point.optimized_kinematic_state;
    ref_points.at(1).pose = prev_ref_front_point.pose;
    ref_points.at(1).fixed_kinematic_state = prev_ref_front_point.optimized_kinematic_state;
  } else {
    // idx 为空或 idx == 0 时，只能固定当前窗口首点。
    // idx == 0 表示上一帧没有更前一个点可用于固定方向；idx 为空表示未能安全匹配上一帧点。
    // 仍然需要重采样来恢复固定间隔，随后再把首点 pose/curvature/固定状态写回。
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    ref_points.front().pose = front_point.pose;
    ref_points.front().curvature = front_point.curvature;
    ref_points.front().fixed_kinematic_state = front_point.optimized_kinematic_state;
  }
}

/**
 * @brief 更新参考点的弧长间隔
 * 
 * 该函数计算每个参考点到下一个参考点的距离，并将其存储为delta_arc_length。
 * 最后一个参考点的delta_arc_length设置为0.0，因为它后面没有下一个点。
 * 
 * @param ref_points 参考点序列的引用，函数会直接修改其中每个点的delta_arc_length字段
 */
void MPTOptimizer::updateDeltaArcLength(std::vector<ReferencePoint> & ref_points) const
{
  for (size_t i = 0; i < ref_points.size(); i++) {
    ref_points.at(i).delta_arc_length =
      (i == ref_points.size() - 1)
        ? 0.0
        : autoware_utils::calc_distance2d(ref_points.at(i + 1), ref_points.at(i));
  }
}

/**
 * @brief 更新参考点的额外信息，包括 alpha 和避障成本
 * 
 * 该函数主要完成以下任务：
 * 1. 计算每个参考点的 alpha。alpha 是当前参考点 yaw 与“沿参考路径向前约一个轴距处”
 *    的弦方向之间的夹角，用于在弯道上修正优化中心的横向误差评价方向。
 * 2. 计算并传播避障成本，包括：
 *    - 基于障碍物检测计算归一化避障成本
 *    - 沿纵向传播避障成本形成避障带
 *    - 对避障成本进行扩散处理，使成本在邻域内平滑衰减
 *    - 继承上一帧的避障成本以保证时序连续性
 *
 * 注意：alpha 使用 wheel_base_m 作为前向查询距离，而目标函数中的优化中心距离使用
 * optimization_center_offset。两者表达的量不同：
 * - wheel_base_m 用来估计车辆尺度上的路径弯曲方向，接近“后轴到前轴”这一物理长度，
 *   因此 alpha 更像是前轮尺度的路径方向修正。
 * - optimization_center_offset 用来决定目标函数实际惩罚哪个前向点的横向误差。
 *   它是可调参数，默认 0.8 * wheel_base_m，用来在“抑制车头摆动”和“避免过度敏感”
 *   之间折中。
 * 这样解耦后，调节优化中心前移距离不会同时改变弯道方向估计的尺度。
 * 
 * @param ref_points 参考点序列的引用，函数会直接修改其中的alpha和normalized_avoidance_cost字段
 */
void MPTOptimizer::updateExtraPoints(std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 计算每个参考点的 alpha。
  // alpha = 从当前参考点指向“前方约一个轴距处参考点”的弦方向 - 当前参考点 yaw。
  //
  // 这里使用 wheel_base_m，而不是 optimization_center_offset，是因为 alpha 只负责估计
  // 车辆尺度上的路径弯曲方向；optimization_center_offset 只负责目标函数实际前移多远评价
  // lateral error。二者解耦后，调优化中心距离不会改变弯道方向修正的尺度。
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto front_wheel_pos =
      trajectory_utils::getNearestPosition(ref_points, i, vehicle_info_.wheel_base_m);

    const bool are_too_close_points =
      autoware_utils::calc_distance2d(front_wheel_pos, ref_points.at(i).pose.position) < 1e-03;
    const auto front_wheel_yaw =
      are_too_close_points
        ? ref_points.at(i).getYaw()
        : autoware_utils::calc_azimuth_angle(ref_points.at(i).pose.position, front_wheel_pos);
    ref_points.at(i).alpha =
      autoware_utils::normalize_radian(front_wheel_yaw - ref_points.at(i).getYaw());
  }

  {  // 避障成本计算与传播
    // 计算单步避障成本并沿纵向传播形成避障带
    for (size_t i = 0; i < ref_points.size(); ++i) {
      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(i));
      if (normalized_avoidance_cost) { //当检测到某个位置有障碍物时，不仅在该点设置避障成本，还在其前后一定范围内都设置相同的成本，形成一个"避障带"。
        const int max_length_idx =
          std::floor(mpt_param_.avoidance_cost_band_length / mpt_param_.delta_arc_length);
        for (int j = -max_length_idx; j <= max_length_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            ref_points.at(k).normalized_avoidance_cost = *normalized_avoidance_cost;
          }
        }
      }
    }

    /*
    // update avoidance cost between longitudinally close obstacles
    constexpr double max_longitudinal_length_to_fill_drivable_area = 50;
    const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
    mpt_param_.delta_arc_length / 2); const auto copied_ref_points = ref_points; for (size_t i = 0;
    i < ref_points.size(); ++i) { const double base_normalized_avoidance_cost =
    ref_points.at(i).normalized_avoidance_cost; for (int j = -edge_fill_index; j <= edge_fill_index;
    ++j) { const int k = i + j; if (k < 0 || ref_points.size() - 1 <= k) { continue;
        }
        ref_points.at(i).normalized_avoidance_cost =
    std::max(ref_points.at(i).normalized_avoidance_cost,
    copied_ref_points.at(k).normalized_avoidance_cost);
      }
    }
    */

    // 对避障成本进行扩散处理，使成本在邻域内按衰减率平滑过渡
    for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
      const double base_normalized_avoidance_cost = ref_points.at(i).normalized_avoidance_cost;
      if (0 < base_normalized_avoidance_cost) {
        const int edge_decrease_idx = std::floor(
          ref_points.at(i).normalized_avoidance_cost / mpt_param_.avoidance_cost_decrease_rate);
        for (int 4j = -edge_decrease_idx; j <= edge_decrease_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            const double normalized_avoidance_cost = std::max(
              base_normalized_avoidance_cost -
                std::abs(j) * mpt_param_.avoidance_cost_decrease_rate,
              ref_points.at(k).normalized_avoidance_cost);
            ref_points.at(k).normalized_avoidance_cost =
              std::clamp(normalized_avoidance_cost, 0.0, 1.0);
          }
        }
      }
    }

    // 继承上一帧的避障成本，保证时序连续性和稳定性,将上一帧的成本与当前帧的成本比较，取较大者这样可以保持避障成本的持续性，避免障碍物检测的抖动
    const double max_dist_threshold = mpt_param_.delta_arc_length / 2.0;
    if (prev_ref_points_ptr_ && !prev_ref_points_ptr_->empty()) {
      for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
        const size_t prev_idx = trajectory_utils::findEgoIndex(
          *prev_ref_points_ptr_, autoware_utils::get_pose(ref_points.at(i)), ego_nearest_param_);

        const double dist_to_prev =
          autoware_utils::calc_distance2d(ref_points.at(i), prev_ref_points_ptr_->at(prev_idx));
        if (max_dist_threshold < dist_to_prev) {
          continue;
        }

        ref_points.at(i).normalized_avoidance_cost = std::max(
          prev_ref_points_ptr_->at(prev_idx).normalized_avoidance_cost,
          ref_points.at(i).normalized_avoidance_cost);
      }
    }
  }
}

// ✅ 边界不是固定值
// ✅ left_bound 和 right_bound 是点序列（不是单个数值）
// ✅ 对每个参考点，都根据其位置动态计算边界距离
// ✅ 最终每个参考点的 bounds 字段都包含不同的左右边界距离
void MPTOptimizer::updateBounds(
  std::vector<ReferencePoint> & ref_points,
  const std::vector<geometry_msgs::msg::Point> & left_bound,  // 这个边界是多个点了，每个点都有
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const double soft_road_clearance =
    mpt_param_.soft_clearance_from_road + vehicle_info_.vehicle_width_m / 2.0;

  // calculate distance to left/right bound on each reference point
  // NOTE: Reference points is sometimes not fully covered by the drivable area.
  //       In some edge cases like U-turn, the wrong bound may be found. To avoid finding the wrong
  //       bound, some beginning bounds are copied from the following one.
  const size_t min_ref_point_index = std::min(
    static_cast<size_t>(std::ceil(1.0 / mpt_param_.delta_arc_length)), ref_points.size() - 1);
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto ref_point_for_bound_search = ref_points.at(std::max(min_ref_point_index, i));
    const double dist_to_left_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, left_bound, soft_road_clearance, true);
    const double dist_to_right_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, right_bound, soft_road_clearance, false);
    ref_points.at(i).bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
  }

  // keep vehicle width + margin
  // NOTE: The drivable area's width is sometimes narrower than the vehicle width which means
  // infeasible to run especially when obstacles are extracted from the drivable area.
  //       In this case, the drivable area's width is forced to be wider.
  keepMinimumBoundsWidth(ref_points);

  // extend violated bounds, where the input path is outside the drivable area
  ref_points = extendViolatedBounds(ref_points);

  // keep previous boundary's width around ego to avoid sudden steering
  avoidSuddenSteering(ref_points, ego_pose, ego_vel);

  /*
  // TODO(murooka) deal with filling data between obstacles
  // fill between obstacles
  constexpr double max_longitudinal_length_to_fill_drivable_area = 20;
  const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
  mpt_param_.delta_arc_length / 2); for (int i = 0; i < ref_points.size(); ++i) { for (int j =
  -edge_fill_index; j <= edge_fill_index; ++j) { const int k = i + j; if (k < 0 || ref_points.size()
  - 1 <= k) { continue;
      }

      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(k));
      if (normalized_avoidance_cost) {
      }
    }
  }
  */
  return;
}

void MPTOptimizer::keepMinimumBoundsWidth(std::vector<ReferencePoint> & ref_points) const
{
  // calculate drivable area width considering the curvature
  std::vector<double> min_dynamic_drivable_width_vec;
  for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
    double curvature = std::abs(ref_points.at(i).curvature);
    if (i != static_cast<int>(ref_points.size()) - 1) {
      curvature = std::max(curvature, std::abs(ref_points.at(i + 1).curvature));
    }
    if (i != 0) {
      curvature = std::max(curvature, std::abs(ref_points.at(i - 1).curvature));
    }

    const double max_longitudinal_length = std::max(
      std::abs(vehicle_info_.max_longitudinal_offset_m),
      std::abs(vehicle_info_.min_longitudinal_offset_m));
    const double turning_radius = 1.0 / curvature;
    const double additional_drivable_width_by_curvature =
      std::hypot(max_longitudinal_length, turning_radius + vehicle_info_.vehicle_width_m / 2.0) -
      turning_radius - vehicle_info_.vehicle_width_m / 2.0;
    min_dynamic_drivable_width_vec.push_back(
      mpt_param_.min_drivable_width + additional_drivable_width_by_curvature);
  }

  // 1. calculate start and end sections which are out of bounds
  std::vector<std::pair<size_t, size_t>> out_of_upper_bound_sections;
  std::vector<std::pair<size_t, size_t>> out_of_lower_bound_sections;
  std::optional<size_t> out_of_upper_bound_start_idx = std::nullopt;
  std::optional<size_t> out_of_lower_bound_start_idx = std::nullopt;
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & b = ref_points.at(i).bounds;

    // const double drivable_width = b.upper_bound - b.lower_bound;
    // const bool is_infeasible_to_drive = drivable_width < min_dynamic_drivable_width

    // NOTE: The following condition should be uncommented to see obstacles outside the path.
    //       However, on a narrow road, the ego may go outside the road border with this condition.
    //       Currently, we cannot distinguish obstacles and road border
    if (/*is_infeasible_to_drive ||*/ b.upper_bound < 0.0) {  // out of upper bound
      if (!out_of_upper_bound_start_idx) {
        out_of_upper_bound_start_idx = i;
      }
    } else {
      if (out_of_upper_bound_start_idx) {
        out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, i - 1});
        out_of_upper_bound_start_idx = std::nullopt;
      }
    }
    if (/*is_infeasible_to_drive ||*/ 0.0 < b.lower_bound) {  // out of lower bound
      if (!out_of_lower_bound_start_idx) {
        out_of_lower_bound_start_idx = i;
      }
    } else {
      if (out_of_lower_bound_start_idx) {
        out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, i - 1});
        out_of_lower_bound_start_idx = std::nullopt;
      }
    }
  }
  if (out_of_upper_bound_start_idx) {
    out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, ref_points.size() - 1});
  }
  if (out_of_lower_bound_start_idx) {
    out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, ref_points.size() - 1});
  }

  auto original_ref_points = ref_points;
  const auto is_inside_sections = [&](const size_t target_idx, const auto & sections) {
    for (const auto & section : sections) {
      if (section.first <= target_idx && target_idx <= section.second) {
        return true;
      }
    }
    return false;
  };

  // lower bound
  for (const auto & out_of_lower_bound_section : out_of_lower_bound_sections) {
    std::optional<size_t> upper_bound_start_idx = std::nullopt;
    std::optional<size_t> upper_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_lower_bound_section.first;
         p_idx <= out_of_lower_bound_section.second; ++p_idx) {
      const bool is_out_of_upper_bound = is_inside_sections(p_idx, out_of_upper_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_upper_bound) {
        if (!upper_bound_start_idx) {
          upper_bound_start_idx = p_idx;
        }
        upper_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Lower bound is cut out. Widen the bounds towards the lower bound since cut out too
      // much.
      b.lower_bound =
        std::min(b.lower_bound, original_b.upper_bound - min_dynamic_drivable_width_vec.at(p_idx));
    }
    // extend longitudinal if it overlaps out_of_upper_bound_sections
    if (upper_bound_start_idx) {
      for (size_t p_idx = out_of_lower_bound_section.first; p_idx < *upper_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_start_idx).bounds.lower_bound);
      }
    }
    if (upper_bound_end_idx) {
      for (size_t p_idx = *upper_bound_end_idx + 1; p_idx <= out_of_lower_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_end_idx).bounds.lower_bound);
      }
    }
  }

  // upper bound
  for (const auto & out_of_upper_bound_section : out_of_upper_bound_sections) {
    std::optional<size_t> lower_bound_start_idx = std::nullopt;
    std::optional<size_t> lower_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_upper_bound_section.first;
         p_idx <= out_of_upper_bound_section.second; ++p_idx) {
      const bool is_out_of_lower_bound = is_inside_sections(p_idx, out_of_lower_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_lower_bound) {
        if (!lower_bound_start_idx) {
          lower_bound_start_idx = p_idx;
        }
        lower_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Upper bound is cut out. Widen the bounds towards the upper bound since cut out too
      // much.
      b.upper_bound =
        std::max(b.upper_bound, original_b.lower_bound + min_dynamic_drivable_width_vec.at(p_idx));
    }
    // extend longitudinal if it overlaps out_of_lower_bound_sections
    if (lower_bound_start_idx) {
      for (size_t p_idx = out_of_upper_bound_section.first; p_idx < *lower_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_start_idx).bounds.upper_bound);
      }
    }
    if (lower_bound_end_idx) {
      for (size_t p_idx = *lower_bound_end_idx + 1; p_idx <= out_of_upper_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_end_idx).bounds.upper_bound);
      }
    }
  }
}

std::vector<ReferencePoint> MPTOptimizer::extendViolatedBounds(
  const std::vector<ReferencePoint> & ref_points) const
{
  auto extended_ref_points = ref_points;
  const int max_length_idx = std::floor(
    mpt_param_.max_longitudinal_margin_for_bound_violation / mpt_param_.delta_arc_length);
  for (int i = 0; i < static_cast<int>(ref_points.size()) - 1; ++i) {
    // before violation
    if (
      ref_points.at(i).bounds.lower_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.lower_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i + 1).bounds.lower_bound;
      }
    }

    if (
      0.0 <= ref_points.at(i).bounds.upper_bound &&
      ref_points.at(i + 1).bounds.upper_bound <= 0.0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i + 1).bounds.upper_bound;
      }
    }

    // after violation
    if (0 <= ref_points.at(i).bounds.lower_bound && ref_points.at(i + 1).bounds.lower_bound <= 0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i).bounds.lower_bound;
      }
    }

    if (
      ref_points.at(i).bounds.upper_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.upper_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i).bounds.upper_bound;
      }
    }
  }

  return extended_ref_points;
}

void MPTOptimizer::avoidSuddenSteering(
  std::vector<ReferencePoint> & ref_points, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_vel) const
{
  if (!prev_ref_points_ptr_) {
    return;
  }
  const size_t prev_ego_idx = trajectory_utils::findEgoIndex(
    *prev_ref_points_ptr_, autoware_utils::get_pose(ref_points.front()), ego_nearest_param_);

  const double max_bound_fixing_length = ego_vel * mpt_param_.max_bound_fixing_time;
  const int max_bound_fixing_idx =
    std::floor(max_bound_fixing_length / mpt_param_.delta_arc_length);

  const size_t ego_idx = trajectory_utils::findEgoIndex(ref_points, ego_pose, ego_nearest_param_);
  const size_t max_fixed_bound_idx =
    std::min(ego_idx + static_cast<size_t>(max_bound_fixing_idx), ref_points.size());

  for (size_t i = 0; i < max_fixed_bound_idx; ++i) {
    const size_t prev_idx = std::min(
      prev_ego_idx + i, static_cast<size_t>(static_cast<int>(prev_ref_points_ptr_->size()) - 1));
    const auto & prev_bounds = prev_ref_points_ptr_->at(prev_idx).bounds;

    ref_points.at(i).bounds.upper_bound = prev_bounds.upper_bound;
    ref_points.at(i).bounds.lower_bound = prev_bounds.lower_bound;
  }
}

/**
 * @brief 为每个参考点、每个车辆碰撞圆计算边界约束用的几何信息
 * 
 * MPT 的碰撞/道路边界约束不是只检查 base_link 一个点，而是把车辆近似成多个圆。
 * 对于参考点 i 和车辆圆 l，本函数会计算：
 *
 * - beta[i][l]：当前参考点 yaw 与圆心所在路径位置 yaw 的差值。
 *   后续约束会用它把圆心横向位置线性化为
 *     y_circle ~= cos(beta) * lat + lon_offset * cos(beta) * yaw
 *                 + lon_offset * sin(beta)
 *
 * - bounds_on_constraints[i][l]：第 l 个圆心允许出现的横向边界。
 *   它由圆心所在路径位置附近的 bounds 线性插值得到，并用 offset_y 修正到
 *   上面线性化公式的坐标原点。
 *
 * - pose_on_constraints[i][l]：调试/可视化用的边界参考位姿。
 *
 * 这些信息后续会在 calcConstraintMatrix() 中组成 collision-free 约束。
 * 
 * @param ref_points 参考点向量，用于存储计算后的边界约束信息
 * @param ref_points_spline 参考点的样条插值对象，用于获取任意位置的位姿和弧长信息
 */
void MPTOptimizer::updateVehicleBounds(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (size_t p_idx = 0; p_idx < ref_points.size(); ++p_idx) {
    const auto & ref_point = ref_points.at(p_idx);
    // ReferencePoint 可能来自上一轮处理或重采样，内部仍残留旧的车辆圆约束信息。
    // 因此每帧重新计算前都要清空。
    ref_points.at(p_idx).bounds_on_constraints.clear();
    ref_points.at(p_idx).beta.clear();

    for (const double lon_offset : vehicle_circle_longitudinal_offsets_) {
      // lon_offset 是当前车辆圆心相对 base_link 的纵向偏移。
      // 在参考路径弧长 s(p_idx) + lon_offset 处查询样条位姿，作为该车辆圆的边界检查坐标系。
      const auto collision_check_pose =
        ref_points_spline.getSplineInterpolatedPose(p_idx, lon_offset);
      const double collision_check_yaw = tf2::getYaw(collision_check_pose.orientation);

      // beta 是当前参考点 yaw 与车辆圆所在路径位置 yaw 的差值。
      // 后续会用它线性化车辆圆心的横向位置：
      //   y_circle ~= cos(beta) * lat + lon_offset * cos(beta) * yaw
      //               + lon_offset * sin(beta)
      const double beta = ref_point.getYaw() - collision_check_yaw;
      ref_points.at(p_idx).beta.push_back(beta);

      // 计算 ref_point 在 collision_check_pose 坐标系下的横向坐标 offset_y。
      //
      // 车辆圆心相对 collision_check_pose 的精确横向位置中包含常数项：
      //   n_collision^T * (ref_point.position - collision_check_pose.position).
      // 该常数项就是 offset_y。代码没有把它并入 C_vec，而是在下面从插值边界中减掉，
      // 这样最终约束仍可写成：
      //   bounds.lower <= C * X + C_vec <= bounds.upper
      const double tmp_yaw = std::atan2(
        collision_check_pose.position.y - ref_point.pose.position.y,
        collision_check_pose.position.x - ref_point.pose.position.x);
      const double offset_y = -autoware_utils::calc_distance2d(ref_point, collision_check_pose) *
                              std::sin(tmp_yaw - collision_check_yaw);

      // 将车辆圆检查位姿横向平移 offset_y，得到该圆约束实际使用的边界参考位姿。
      // 该位姿主要用于调试可视化。
      const auto vehicle_bounds_pose =
        autoware_utils::calc_offset_pose(collision_check_pose, 0.0, offset_y, 0.0);

      // 在车辆圆所在弧长位置插值道路/障碍边界。
      const auto bounds = [&]() {
        const double collision_check_s = ref_points_spline.getAccumulatedLength(p_idx) + lon_offset;
        const size_t collision_check_idx = ref_points_spline.getOffsetIndex(p_idx, lon_offset);

        // 使用包围 collision_check_s 的两个参考点，对它们已经计算好的中心线边界做线性插值。
        const size_t prev_idx = std::clamp(
          collision_check_idx - 1, static_cast<size_t>(0),
          static_cast<size_t>(ref_points_spline.getSize() - 2));
        const size_t next_idx = prev_idx + 1;

        const auto & prev_bounds = ref_points.at(prev_idx).bounds;
        const auto & next_bounds = ref_points.at(next_idx).bounds;

        const double prev_s = ref_points_spline.getAccumulatedLength(prev_idx);
        const double next_s = ref_points_spline.getAccumulatedLength(next_idx);

        const double ratio = std::clamp((collision_check_s - prev_s) / (next_s - prev_s), 0.0, 1.0);

        auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);

        // 插值得到的 bounds 原本表达在 collision_check_pose 坐标系下，会约束：
        //   offset_y + C * X + C_vec.
        // 通过 translate(offset_y) 将 offset_y 从左侧表达式移到边界上，使后续约束只需处理：
        //   lower - offset_y <= C * X + C_vec <= upper - offset_y.
        // Bounds::translate(offset) 的实现就是 lower -= offset, upper -= offset。
        bounds.translate(offset_y);
        return bounds;
      }();

      // 保持与 vehicle_circle_longitudinal_offsets_ 相同的顺序：
      // 后续 calcConstraintMatrix() 中的 l_idx 会同时索引 beta 和 bounds_on_constraints。
      ref_points.at(p_idx).bounds_on_constraints.push_back(bounds);
      ref_points.at(p_idx).pose_on_constraints.push_back(vehicle_bounds_pose);
    }
  }
}

// cost function: J = x' Q x + u' R u
/**
 * @brief 计算MPT优化器的权重矩阵(Q矩阵和R矩阵)
 * 
 * 该函数为二次规划目标函数准备状态权重和控制权重。忽略优化中心偏移时，
 * 可以把代价理解为：
 *
 *   J = X^T Q X + U^T R U
 *
 * 其中：
 * - X = [x_0, x_1, ..., x_{N-1}]^T 是整段状态序列
 * - x_i = [lat_error_i, yaw_error_i]^T
 * - U = [u_0, u_1, ..., u_{N-2}]^T 是整段转向角序列
 * - Q矩阵: 状态误差权重矩阵，包含横向误差和航向误差的权重
 * - R矩阵: 控制输入权重矩阵，包含转向角本身和转向变化率的权重
 *
 * 实际的 Hessian 会在 calcObjectiveMatrix() 中进一步结合优化中心变换
 * Z = T * X + T_vec，得到状态项 (T * X + T_vec)^T Q (T * X + T_vec)。
 *
 * Q和R只描述“希望优化器偏好什么”，不描述车辆运动学约束。车辆运动学约束由
 * calcConstraintMatrix() 中的状态方程约束给出。
 * 
 * 权重会根据以下情况自适应调整：
 * 1. 终端点：如果参考点最后一个点就是全局轨迹终点，使用goal权重；否则使用terminal权重
 * 2. 避障区域：根据归一化避障成本线性插值调整权重
 * 3. 正常区域：使用默认权重参数
 *
 * 避障区域的插值形式为：
 *
 *   weight = lerp(normal_weight, avoidance_weight, normalized_avoidance_cost)
 *
 * 当 normalized_avoidance_cost = 0 时使用普通权重；
 * 当 normalized_avoidance_cost = 1 时使用避障权重。
 * 
 * @param ref_points 参考点序列，包含位置、方向、边界信息和避障成本等
 * @param traj_points 轨迹点序列，用于判断目标点是否被包含
 * 
 * @return ValueMatrix 包含Q矩阵和R矩阵的结构体
 *         - Q: N_x × N_x 稀疏矩阵，N_x = N_ref × D_x
 *         - R: N_u × N_u 稀疏矩阵，N_u = (N_ref - 1) × D_u
 */
MPTOptimizer::ValueMatrix MPTOptimizer::calcValueMatrix(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // D_x 是单个参考点的状态维度。当前自行车模型中 D_x = 2：
  //   x_i = [lat_error_i, yaw_error_i]^T
  // D_u 是单段控制输入维度。当前模型中 D_u = 1：
  //   u_i = steer_i
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  // N_ref 个参考点对应 N_ref 个状态块，因此总状态维度为 N_ref * D_x。
  // 控制输入作用在相邻参考点之间，因此只有 N_ref - 1 个控制块。
  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // 判断参考点末端是否已经到达输入轨迹的终点。
  // 如果已经到达真正的goal，就使用更强的goal权重；否则只是优化窗口末端，使用terminal权重。
  const bool is_goal_contained = geometry_utils::isSamePoint(ref_points.back(), traj_points.back());

  // 构建Q矩阵的三元组列表。
  // Q是对角矩阵，每个参考点占两个对角元素：
  //   Q(i*D_x,     i*D_x)     -> lat_error_i 的权重
  //   Q(i*D_x + 1, i*D_x + 1) -> yaw_error_i 的权重
  // 权重越大，优化器越不愿意在该维度产生误差。
  std::vector<Eigen::Triplet<double>> Q_triplet_vec;
  for (size_t i = 0; i < N_ref; ++i) {
    // 根据参考点类型自适应计算该点的横向误差和航向误差权重。
    const auto adaptive_error_weight = [&]() -> std::array<double, 2> {
      // 最后一个参考点需要额外约束轨迹收敛。
      // 如果这个点是真正的轨迹终点，使用goal权重；否则只是当前优化范围末端，使用terminal权重。
      if (i == N_ref - 1) {
        if (is_goal_contained) {
          return {mpt_param_.goal_lat_error_weight, mpt_param_.goal_yaw_error_weight};
        }
        return {mpt_param_.terminal_lat_error_weight, mpt_param_.terminal_yaw_error_weight};
      }

      // 避障区域使用普通权重和避障权重之间的线性插值。
      // 这允许优化器在障碍物附近改变“贴近参考线”和“保持平顺/可绕行”之间的偏好。
      if (0 < ref_points.at(i).normalized_avoidance_cost) {
        const double lat_error_weight = autoware::interpolation::lerp(
          mpt_param_.lat_error_weight, mpt_param_.avoidance_lat_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        const double yaw_error_weight = autoware::interpolation::lerp(
          mpt_param_.yaw_error_weight, mpt_param_.avoidance_yaw_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        return {lat_error_weight, yaw_error_weight};
      }

      // 普通区域使用默认横向误差和航向误差权重。
      return {mpt_param_.lat_error_weight, mpt_param_.yaw_error_weight};
    }();

    const double adaptive_lat_error_weight = adaptive_error_weight.at(0);
    const double adaptive_yaw_error_weight = adaptive_error_weight.at(1);

    // 只填充对角项，不在不同参考点之间引入交叉误差项。
    // 因此每个点的 lat/yaw 误差代价是：
    //   q_lat_i * lat_error_i^2 + q_yaw_i * yaw_error_i^2
    Q_triplet_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, adaptive_lat_error_weight));
    Q_triplet_vec.push_back(
      Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, adaptive_yaw_error_weight));
  }
  Eigen::SparseMatrix<double> Q_sparse_mat(N_x, N_x);
  Q_sparse_mat.setFromTriplets(Q_triplet_vec.begin(), Q_triplet_vec.end());

  // 构建R矩阵的三元组列表。
  // R的基础对角项惩罚转向角本身：
  //   r_i * steer_i^2
  // 在避障区域，转向权重也会向 avoidance_steer_input_weight 插值。
  std::vector<Eigen::Triplet<double>> R_triplet_vec;
  for (size_t i = 0; i < N_ref - 1; ++i) {
    // 根据避障成本自适应调整该段转向角权重。
    const double adaptive_steer_weight = autoware::interpolation::lerp(
      mpt_param_.steer_input_weight, mpt_param_.avoidance_steer_input_weight,
      ref_points.at(i).normalized_avoidance_cost);
    R_triplet_vec.push_back(Eigen::Triplet<double>(D_u * i, D_u * i, adaptive_steer_weight));
  }
  Eigen::SparseMatrix<double> R_sparse_mat(N_u, N_u);

  // 额外加入转向变化率代价：
  //   steer_rate_weight * (u_i - u_{i-1})^2
  // 这会在R中产生相邻控制输入之间的非对角项，让优化结果的转向序列更平滑。
  addSteerWeightR(R_triplet_vec, ref_points);

  R_sparse_mat.setFromTriplets(R_triplet_vec.begin(), R_triplet_vec.end());

  return ValueMatrix{Q_sparse_mat, R_sparse_mat};
}

/**
 * @brief 计算MPT优化器的目标矩阵（Hessian矩阵和梯度向量）
 * 
 * 该函数把 calcValueMatrix() 中得到的 Q/R 权重转换成 OSQP 使用的目标函数矩阵。
 * 决策变量整体排列为：
 *
 *   v = [X, U, S]^T
 *
 * 其中：
 * - X = [x_0, x_1, ..., x_{N-1}]^T，x_i = [lat_error_i, yaw_error_i]^T
 * - U = [u_0, u_1, ..., u_{N-2}]^T，u_i 为第 i 段转向角
 * - S 为软碰撞约束的松弛变量，只有开启 soft_constraint 时才有维度
 *
 * 目标函数可以理解为：
 *
 *   min  Z^T Q Z + U^T R U + soft_collision_free_weight * sum(S)
 *
 * 这里的 Z 不是原始状态 X，而是把横向误差评价点从参考点附近前移到
 * optimization_center_offset 后的误差：
 *
 *   Z = T * X + T_vec
 *
 * 对单个参考点 i，记：
 * - lat_i: 参考点处的横向误差
 * - yaw_i: 参考点处的航向误差
 * - offset: 优化中心沿车辆朝向前移的距离，即 optimization_center_offset
 * - alpha_i: 参考点 yaw 到“前方约一个轴距处参考点”的弦方向夹角
 *
 * 引入“优化中心”的核心原因是：不要只让参考点本身贴着路径，而是让车辆前方某个
 * 更有代表性的点也贴着路径。直线道路上如果 lat_i = 0 但 yaw_i != 0，车辆前方点
 * 仍然会产生 offset * yaw_i 的横向偏移；惩罚优化中心可以更直接抑制车头摆动。
 *
 * alpha_i 和 offset 的距离来源故意不同：
 * - alpha_i 用 wheel_base_m 计算，因为它是路径弯曲方向的几何估计，使用车辆轴距这个
 *   物理尺度更稳定，也接近前轮/车身尺度的方向变化。
 * - offset 使用 optimization_center_offset，因为它是目标函数的调参量，决定实际惩罚
 *   多靠前的点。默认值是 0.8 * wheel_base_m，经验上比完整轴距更不容易过度放大
 *   yaw_error。
 * 换句话说，alpha_i 决定“往哪个方向投影横向误差”，offset 决定“前方多远的点被惩罚”。
 *
 * z_lat_i 的几何推导如下。以当前参考点为局部坐标系原点，x 轴沿当前参考 yaw，
 * y 轴沿左法向。令：
 *
 *   l = lat_i, theta = yaw_i, d = offset, alpha = alpha_i
 *
 * 车辆基准点有横向误差 l，车辆前方 d 处的优化中心近似为：
 *
 *   p_vehicle = [0, l]^T + d * [cos(theta), sin(theta)]^T
 *             = [d * cos(theta), l + d * sin(theta)]^T
 *
 * 前方参考路径方向相对当前 yaw 偏 alpha，因此前方参考点和其左法向为：
 *
 *   p_ref = d * [cos(alpha), sin(alpha)]^T
 *   n_alpha = [-sin(alpha), cos(alpha)]^T
 *
 * 优化中心的横向误差就是二者差值投影到 n_alpha 上：
 *
 *   z_lat = n_alpha^T * (p_vehicle - p_ref)
 *         = cos(alpha) * l + d * sin(theta - alpha)
 *
 * 为了保持 QP 为二次规划，z_lat 必须对优化变量 l/theta 线性。因此在 theta = 0
 * 附近线性化：
 *
 *   sin(theta - alpha) ~= sin(-alpha) + theta * cos(-alpha)
 *                       = -sin(alpha) + theta * cos(alpha)
 *
 * 代回即可得到下面代码使用的线性近似。
 *
 * 则小角度近似下，优化中心的横向误差为：
 *
 *   z_lat_i = cos(alpha_i) * lat_i
 *             + offset * cos(alpha_i) * yaw_i
 *             - offset * sin(alpha_i)
 *
 * 航向误差仍直接使用：
 *
 *   z_yaw_i = yaw_i
 *
 * 因此每个点对应的局部变换是：
 *
 *   [z_lat_i] = [cos(alpha_i), offset * cos(alpha_i)] [lat_i] + [-offset * sin(alpha_i)]
 *   [z_yaw_i]   [0,            1                  ] [yaw_i]   [0                    ]
 *
 * 直线道路 alpha_i = 0 时，上式退化为：
 *
 *   z_lat_i = lat_i + offset * yaw_i
 *
 * 也就是：车辆后轴/参考点处有一点航向误差时，前方优化中心会额外产生
 * offset * yaw_i 的横向偏移。这比只惩罚参考点本身的 lat_error 更能抑制车头摆动。
 *
 * 展开状态代价：
 *
 *   (T X + T_vec)^T Q (T X + T_vec)
 *     = X^T T^T Q T X + 2 * T_vec^T Q T X + const
 *
 * 常数项不影响最优解，因此不进入优化器。代码按本模块已有的 QP 系数约定生成：
 * - H_x = T^T Q T
 * - g_x = T_vec^T Q T
 * 
 * 控制输入部分直接使用 R；松弛变量部分用线性项惩罚 violation，让优化器只有在
 * 硬性可行性较差时才愿意放松碰撞约束。
 * 
 * @param mpt_mat 状态方程生成器的矩阵（当前未使用，保留用于未来扩展）
 * @param val_mat 价值矩阵，包含状态权重矩阵Q和控制权重矩阵R
 * @param ref_points 参考点序列，包含每个离散点的位姿、速度、曲率等信息
 * @return ObjectiveMatrix 包含Hessian矩阵和梯度向量的目标矩阵结构体
 */
MPTOptimizer::ObjectiveMatrix MPTOptimizer::calcObjectiveMatrix(
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 单点状态维度 D_x 当前为 2：[lat_error, yaw_error]。
  // 单段控制维度 D_u 当前为 1：[steer]。
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();

  // 每个参考点最多分配 N_slack 个松弛变量。
  // 如果没有开启 soft_constraint，getNumberOfSlackVariables() 返回 0。
  const size_t N_slack = getNumberOfSlackVariables();

  // 决策变量维度：
  //   N_x: 所有状态变量 X 的维度
  //   N_u: 所有转向输入 U 的维度
  //   N_s: 所有松弛变量 S 的维度
  //   N_v: OSQP 决策变量 v = [X, U, S] 的总维度
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_s = N_ref * N_slack;

  const size_t N_v = N_x + N_u + N_s;

  // 构建稀疏坐标变换矩阵 T 和常数偏移 T_vec：
  //   Z = T * X + T_vec
  //
  // X 是参考点处的误差状态；Z 是优化中心处的误差状态。
  // 这里主要是把 lateral error 的评价点从参考点位置前移 offset，
  // yaw error 仍然保持原状态 yaw_i。
  std::vector<Eigen::Triplet<double>> triplet_T_vec;
  Eigen::VectorXd T_vec = Eigen::VectorXd::Zero(N_x);
  const double offset = mpt_param_.optimization_center_offset;
  for (size_t i = 0; i < N_ref; ++i) {
    // alpha 是参考点 yaw 与“从该点指向前方约一个轴距处参考点”的方向差。
    // 它只负责修正弯道上的投影方向；真正被惩罚的前向距离由 optimization_center_offset
    // 决定。这样可以单独调节优化中心距离，而不改变 alpha 对路径曲率的估计尺度。
    const double alpha = ref_points.at(i).alpha;

    // z_lat_i = cos(alpha) * lat_i + offset * cos(alpha) * yaw_i - offset * sin(alpha)
    // z_yaw_i = yaw_i
    //
    // 因此 T 的第 i 个状态块为：
    //   [cos(alpha), offset * cos(alpha)]
    //   [0,          1                  ]
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));

    // T_vec 只影响横向误差项，表示弯道上参考前向点本身相对当前参考点切线方向的侧向偏移。
    T_vec(i * D_x) = -offset * std::sin(alpha);
  }
  Eigen::SparseMatrix<double> sparse_T_mat(N_x, N_x);
  sparse_T_mat.setFromTriplets(triplet_T_vec.begin(), triplet_T_vec.end());

  // 状态代价的二次项：
  //   Z^T Q Z = (T X + T_vec)^T Q (T X + T_vec)
  // 其中与 X 二次相关的部分为：
  //   X^T T^T Q T X
  // 所以状态变量对应的 Hessian 子块是 H_x = T^T Q T。
  // 结果理论上是对称矩阵，这里先填上三角，再拷贝到下三角，避免数值计算带来的轻微非对称。
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(N_x, N_x);
  H_x.triangularView<Eigen::Upper>() =
    Eigen::MatrixXd(sparse_T_mat.transpose() * val_mat.Q * sparse_T_mat);
  H_x.triangularView<Eigen::Lower>() = H_x.transpose();

  // 组装完整 Hessian。
  // v = [X, U, S]，因此 H 的块结构为：
  //
  //   H = [H_x, 0, 0]
  //       [0,   R, 0]
  //       [0,   0, 0]
  //
  // slack 只通过线性项惩罚，不放入二次项。
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N_v, N_v);
  H.block(0, 0, N_x, N_x) = H_x;
  H.block(N_x, N_x, N_u, N_u) = val_mat.R;

  // 组装线性项 g。
  // 状态部分来自展开式中的 T_vec^T Q T X；常数项 T_vec^T Q T_vec 与优化变量无关，丢弃。
  // slack 部分为 soft_collision_free_weight * sum(S)，用于惩罚软约束违反量。
  Eigen::VectorXd g = Eigen::VectorXd::Zero(N_v);
  g.segment(0, N_x) = T_vec.transpose() * val_mat.Q * sparse_T_mat;
  g.segment(N_x + N_u, N_s) = mpt_param_.soft_collision_free_weight * Eigen::VectorXd::Ones(N_s);

  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = H;
  obj_matrix.gradient = g;

  return obj_matrix;
}

// Constraint: lb <= A * v <= ub
// decision variable:
//   v := [X, U, S]
//   X: stacked kinematic states [lat_error, yaw_error]
//   U: stacked steer angles
//   S: slack variables for soft collision-free constraints
/**
 * @brief 计算MPT(Model Predictive Trajectory)优化器的约束矩阵
 * 
 * 该函数构建 OSQP 使用的线性约束：
 *
 *   lb <= A * v <= ub
 *
 * 其中决策变量按以下顺序排列：
 *
 *   v = [X, U, S]^T
 *
 * - X: 所有参考点的运动学状态，x_i = [lat_i, yaw_i]^T
 * - U: 相邻参考点之间的转向角输入，u_i = steer_i
 * - S: 软碰撞约束的松弛变量，仅在 soft_constraint 开启时存在
 *
 * 本函数会依次写入四类约束：
 * 1. 状态方程约束：保证 X 和 U 满足离散车辆运动学模型
 * 2. 碰撞/道路边界约束：保证车辆圆心落在允许边界内，可选软约束或硬约束
 * 3. 固定点约束：把当前窗口前端点锁定到上一帧优化结果，保证时间连续性
 * 4. 转向角限制约束：限制 U 落在参考曲率转角附近的可行范围
 *
 * 这里的 A_rows_end 是“当前已经写入到 A/lb/ub 的行数”，每写完一类约束块就向后推进。
 * 
 * @param mpt_mat 状态方程生成器计算的矩阵,包含A、B、W矩阵,用于描述系统动力学
 * @param ref_points 参考点序列,包含轨迹上的位姿、曲率、边界等信息
 * @return ConstraintMatrix 包含线性约束矩阵A、下界lb、上界ub的结构体
 * 
 * 约束矩阵的结构:
 * - 状态变量维度: N_x = N_ref * D_x (参考点数 × 状态维度)
 * - 控制变量维度: N_u = (N_ref-1) * D_u ((参考点数-1) × 控制维度)
 * - Slack变量维度: N_ref * N_slack (仅在软约束时存在)
 * - 总变量数: N_v = N_x + N_u + slack变量数
 * 
 * 约束类型及行数:
 * 1. 状态方程约束: N_x行
 * 2. 碰撞避免约束: 
 *    - 软约束: 3 * N_ref * N_collision_check行 (上界、下界、非负约束)
 *    - 硬约束: N_ref * N_collision_check行
 * 3. 固定点约束: fixed_points数量 * D_x行
 * 4. 转向角限制约束: N_u行 (如果启用)
 */
MPTOptimizer::ConstraintMatrix MPTOptimizer::calcConstraintMatrix(
  const StateEquationGenerator::Matrix & mpt_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 当前自行车模型中：
  //   D_x = 2，对应 [lat_error, yaw_error]
  //   D_u = 1，对应 steer
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  // N_ref 个参考点对应 N_ref 个状态块；转向输入作用在两点之间，因此只有 N_ref - 1 个。
  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // 单个参考点拥有的 slack 变量个数：
  // - soft_constraint=false: 0
  // - soft_constraint=true && l_inf_norm=true: 1，所有车辆圆共享同一个 violation 上界
  // - soft_constraint=true && l_inf_norm=false: N_collision_check，每个车辆圆一个 slack
  // 总 slack 数量为 N_ref * N_slack。
  const size_t N_slack = getNumberOfSlackVariables();

  // 只有开启软约束时，决策变量中才包含 S。
  const size_t N_v = N_x + N_u + (mpt_param_.soft_constraint ? N_ref * N_slack : 0);

  // 每个纵向偏移对应一个车辆圆检查点。
  const size_t N_collision_check = vehicle_circle_longitudinal_offsets_.size();

  // 收集需要添加固定点约束的参考点索引。
  // updateFixedPoint() 会把 fixed_kinematic_state 写入这些 ReferencePoint。
  std::vector<size_t> fixed_points_indices;
  for (size_t i = 0; i < N_ref; ++i) {
    if (ref_points.at(i).fixed_kinematic_state) {
      fixed_points_indices.push_back(i);
    }
  }

  // 先统计总约束行数，再一次性分配 A/lb/ub。
  // 这样后续只需要按块写入，避免频繁 resize。
  size_t A_rows = 0;
  A_rows += N_x;  // 状态方程等式约束
  if (mpt_param_.soft_constraint) {
    // 对每个车辆圆、每个参考点写三行：
    // 1. 下边界可违反约束
    // 2. 上边界可违反约束
    // 3. slack 非负约束
    A_rows += 3 * N_ref * N_collision_check;
  }
  if (mpt_param_.hard_constraint) {
    A_rows += N_ref * N_collision_check;  // 每个车辆圆、每个参考点一行双边界约束
  }
  A_rows += fixed_points_indices.size() * D_x;  // 每个固定点固定 lat/yaw 两个状态
  if (mpt_param_.steer_limit_constraint) {
    A_rows += N_u;  // 每个转向输入一行上下界约束
  }

  // NOTE: The following takes 1 [ms]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_rows, N_v);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(A_rows, -autoware::osqp_interface::INF);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(A_rows, autoware::osqp_interface::INF);
  size_t A_rows_end = 0;

  /**
   * 1. 构建状态方程约束
   * 
   * StateEquationGenerator::calcMatrix() 返回的是整段递推关系：
   *
   *   X = mpt_mat.A * X + mpt_mat.B * U + mpt_mat.W
   *
   * 这里左右两边都写成 X，是因为 X 表示“整段轨迹的状态堆叠向量”，而不是单个时刻：
   *
   *   X = [x_0, x_1, ..., x_{N-1}]^T
   *
   * 等式按块展开后，第 i 行状态块实际表示：
   *
   *   x_i = Ad_{i-1} * x_{i-1} + Bd_{i-1} * u_{i-1} + Wd_{i-1}
   *
   * 也就是说，左边的 X_i 是当前点状态；右边的 mpt_mat.A * X 只通过块矩阵取到
   * 前一个状态 x_{i-1}。两个 X 是同一个优化变量向量的不同分量，不是同一时刻状态相等。
   *
   * 移项后得到 OSQP 的等式约束：
   *
   *   (I - mpt_mat.A) * X - mpt_mat.B * U = mpt_mat.W
   *
   * 因此这一块写成：
   *
   *   [I - A_model | -B_model | 0] * [X, U, S]^T = W
   *
   * 通过 lb = ub = W 表达等式约束。
   */
  A.block(0, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x, N_x) - mpt_mat.A;
  A.block(0, N_x, N_x, N_u) = -mpt_mat.B;
  lb.segment(0, N_x) = mpt_mat.W;
  ub.segment(0, N_x) = mpt_mat.W;
  A_rows_end += N_x;

  /**
   * 2. 构建碰撞避免约束
   * 
   * 车辆形状用多个圆近似，每个圆都有：
   * - lon_offset: 圆心相对 base_link 的纵向偏移
   * - radius: 圆半径
   *
   * 对第 l 个车辆圆，约束的是该圆心在“圆心对应路径点坐标系”下的横向位置
   * y_circle 是否位于道路/障碍边界内。推导如下。
   *
   * 记第 i 个参考点的路径切向和左法向为：
   *
   *   t_ref = [cos(psi_ref), sin(psi_ref)]^T
   *   n_ref = [-sin(psi_ref), cos(psi_ref)]^T
   *
   * 车辆圆所在路径点的左法向为：
   *
   *   n_circle = [-sin(psi_circle), cos(psi_circle)]^T
   *
   * 代码中：
   *
   *   beta_i = psi_ref - psi_circle
   *
   * 优化状态为：
   *
   *   x_i = [lat_i, yaw_i]^T
   *
   * 其中 lat_i 表示 base_link 相对参考点沿 n_ref 的横向偏移，yaw_i 表示车辆
   * 航向相对 psi_ref 的偏差。若第 l 个车辆圆相对 base_link 的纵向偏移为
   * lon_offset，则该圆心相对参考点的近似位移为：
   *
   *   p_circle_rel = lat_i * n_ref
   *                  + lon_offset * [cos(psi_ref + yaw_i), sin(psi_ref + yaw_i)]^T
   *
   * 将它投影到圆心路径点的左法向 n_circle 上：
   *
   *   y_circle_i = n_circle^T * p_circle_rel
   *              = lat_i * n_circle^T n_ref
   *                + lon_offset * n_circle^T [cos(psi_ref + yaw_i), sin(psi_ref + yaw_i)]^T
   *
   * 由于：
   *
   *   n_circle^T n_ref = cos(psi_ref - psi_circle) = cos(beta_i)
   *   n_circle^T [cos(psi_ref + yaw_i), sin(psi_ref + yaw_i)]^T
   *     = sin(psi_ref + yaw_i - psi_circle)
   *     = sin(beta_i + yaw_i)
   *
   * 所以精确到三角函数形式为：
   *
   *   y_circle_i = cos(beta_i) * lat_i + lon_offset * sin(beta_i + yaw_i)
   *
   * 为了保持约束对优化变量线性，在 yaw_i = 0 附近一阶展开：
   *
   *   sin(beta_i + yaw_i) ~= sin(beta_i) + yaw_i * cos(beta_i)
   *
   * 因此得到代码使用的小角度线性化：
   *
   *   y_circle_i ~= cos(beta_i) * lat_i
   *                 + lon_offset * cos(beta_i) * yaw_i
   *                 + lon_offset * sin(beta_i)
   *
   * 记：
   *
   *   C_i     = [cos(beta_i), lon_offset * cos(beta_i)]
   *   C_vec_i = lon_offset * sin(beta_i)
   *
   * 则所有参考点堆叠后：
   *
   *   y_circle = C * X + C_vec
   *
   * 硬约束形式：
   *
   *   lower <= C * X + C_vec <= upper
   *
   * 即：
   *
   *   lower - C_vec <= C * X <= upper - C_vec
   *
   * 软约束会引入 slack，使违反边界仍可求解，但会被目标函数惩罚。
   */
  for (size_t l_idx = 0; l_idx < N_collision_check; ++l_idx) {
    // C_sparse_mat 的每一行只作用于对应参考点的 lat/yaw 两个状态。
    // C_vec 存储圆心横向位置线性化中的常数项。
    Eigen::SparseMatrix<double> C_sparse_mat(N_ref, N_x);
    std::vector<Eigen::Triplet<double>> C_triplet_vec;
    Eigen::VectorXd C_vec = Eigen::VectorXd::Zero(N_ref);

    // 逐点构造车辆圆心横向位置的线性表达：
    //   y_circle_i = C_i * x_i + C_vec_i
    //
    // 其中：
    //   C_i     = [cos(beta), lon_offset * cos(beta)]
    //   C_vec_i = lon_offset * sin(beta)
    for (size_t i = 0; i < N_ref; ++i) {
      // beta 是当前参考点 yaw 与该车辆圆对应弧长所在路径位置 yaw 的差值。
      // 在弯道上，base_link 处参考方向和前/后方圆心处参考方向不同，因此需要 beta 修正投影。
      const double beta = ref_points.at(i).beta.at(l_idx);
      const double lon_offset = vehicle_circle_longitudinal_offsets_.at(l_idx);

      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x, 1.0 * std::cos(beta)));
      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x + 1, lon_offset * std::cos(beta)));
      C_vec(i) = lon_offset * std::sin(beta);
    }
    C_sparse_mat.setFromTriplets(C_triplet_vec.begin(), C_triplet_vec.end());

    // 计算当前车辆圆的可行边界。
    //
    // ref_points[*].bounds_on_constraints[l_idx] 是车辆中心线/圆心参考位姿处的道路边界。
    // 为了保证“圆的外缘”不越界，需要把可行范围向内收缩 circle_radius。
    // 由于 updateBounds() 里的基础边界已经包含 vehicle_width / 2 的车辆半宽裕度，
    // 这里先加回半宽，再扣掉当前圆半径：
    //   bounds_offset = vehicle_width / 2 - circle_radius
    //
    // extractBounds() 中：
    //   upper += bounds_offset
    //   lower -= bounds_offset
    // 若 circle_radius 大于半宽，bounds_offset 为负，可行范围会相应收紧。
    const double bounds_offset =
      vehicle_info_.vehicle_width_m / 2.0 - vehicle_circle_radiuses_.at(l_idx);
    const auto & [part_ub, part_lb] = extractBounds(ref_points, l_idx, bounds_offset);

    /**
     * 软约束处理：引入 slack 变量使约束可违反但有惩罚。
     * 
     * 对每个车辆圆构造 3*N_ref 行，只设置 lower bound，upper bound 维持 +INF：
     *
     * 1. 下边界：
     *      C*X + s >= lower - C_vec
     *    等价于：
     *      C*X >= lower - C_vec - s
     *
     * 2. 上边界：
     *     -C*X + s >= C_vec - upper
     *    等价于：
     *      C*X <= upper - C_vec + s
     *
     * 3. slack 非负：
     *      s >= 0
     *
     * 如果 l_inf_norm=true，所有车辆圆共享每个参考点的同一个 slack；
     * 否则每个车辆圆都有自己的一组 N_ref 个 slack。
     */
    if (mpt_param_.soft_constraint) {
      const size_t A_blk_rows = 3 * N_ref;

      // 当前车辆圆的软约束块：
      //   [ C | 0 | I]
      //   [-C | 0 | I]
      //   [ 0 | 0 | I]
      // 中间的 0 对应控制变量 U；I 对应当前圆使用的 slack 列。
      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_x) = C_sparse_mat;
      A_blk.block(N_ref, 0, N_ref, N_x) = -C_sparse_mat;

      // slack 变量在 v = [X, U, S] 中从 N_x + N_u 开始。
      // l_inf_norm=false 时，不同车辆圆使用不同 slack 段，因此偏移 N_ref * l_idx。
      // l_inf_norm=true 时，所有车辆圆共用同一段 slack，因此偏移为 0。
      const size_t local_A_offset_cols = N_x + N_u + (!mpt_param_.l_inf_norm ? N_ref * l_idx : 0);
      A_blk.block(0, local_A_offset_cols, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(2 * N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);

      // 只设置下界，这里存在两个下界，因为把上界取反，成下界了：
      //   [ lower - C_vec ]
      //   [ C_vec - upper ]
      //   [       0       ]
      // 上界保持初始化时的 +INF。
      Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(A_blk_rows);
      lb_blk.segment(0, N_ref) = -C_vec + part_lb;
      lb_blk.segment(N_ref, N_ref) = C_vec - part_ub;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = lb_blk;

      A_rows_end += A_blk_rows;
    }

    /**
     * 硬约束处理：严格满足碰撞避免约束，不允许 violation。
     * 
     * 构造 N_ref 行双边界约束：
     *
     *   lower - C_vec <= C*X <= upper - C_vec
     *
     * 对应矩阵块：
     *
     *   [C | 0 | 0] * [X, U, S]^T
     */
    if (mpt_param_.hard_constraint) {
      const size_t A_blk_rows = N_ref;

      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_x) = C_sparse_mat;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = part_lb - C_vec;
      ub.segment(A_rows_end, A_blk_rows) = part_ub - C_vec;

      A_rows_end += A_blk_rows;
    }
  }

  /**
   * 3. 构建固定点约束
   * 
   * 对于标记为固定的参考点，其优化状态必须等于上一帧记录的状态：
   *
   *   x_i = fixed_kinematic_state_i = [lat_i, yaw_i]^T
   * 
   * 在矩阵中表示为:
   *
   *   [0 ... I_i ... 0 | 0 | 0] * [X, U, S]^T = fixed_kinematic_state_i
   *
   * 通过 lb = ub 表达等式。固定点通常位于当前优化窗口前端，用于连接上一帧优化轨迹。
   */
  for (const size_t i : fixed_points_indices) {
    A.block(A_rows_end, D_x * i, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

    lb.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();
    ub.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();

    A_rows_end += D_x;
  }

  /**
   * 4. 构建转向角限制约束
   * 
   * 限制控制输入 U 中每个转向角落在参考曲率对应转角附近：
   * 
   *   ref_steer_angle - max_steer <= u_i <= ref_steer_angle + max_steer
   *
   * 其中：
   *
   *   ref_steer_angle = atan(wheel_base * curvature)
   * 
   * 在矩阵中表示为:
   *
   *   [0 | I | 0] * [X, U, S]^T = U
   *
   * 注意：状态方程当前暂时没有使用 curvature 前馈，但转向限制仍围绕参考曲率转角设置。
   */
  if (mpt_param_.steer_limit_constraint) {
    A.block(A_rows_end, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u, N_u);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // lb.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, -mpt_param_.max_steer_rad);
    // ub.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, mpt_param_.max_steer_rad);

    for (size_t i = 0; i < N_u; ++i) {
      const double ref_steer_angle =
        std::atan2(vehicle_info_.wheel_base_m * ref_points.at(i).curvature, 1.0);
      lb(A_rows_end + i) = ref_steer_angle - mpt_param_.max_steer_rad;
      ub(A_rows_end + i) = ref_steer_angle + mpt_param_.max_steer_rad;
    }

    // cppcheck-suppress unreadVariable
    A_rows_end += N_u;
  }

  return ConstraintMatrix{A, lb, ub};
}

void MPTOptimizer::addSteerWeightR(
  std::vector<Eigen::Triplet<double>> & R_triplet_vec,
  const std::vector<ReferencePoint> & ref_points) const
{
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;

  // add steering rate : weight for (u(i) - u(i-1))^2
  for (size_t i = 0; i < N_u - 1; ++i) {
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i, mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i + 1, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i + 1, mpt_param_.steer_rate_weight));
  }
}

/**
 * @brief 求解 MPT 的二次规划问题，并返回完整的优化变量向量。
 *
 * 虽然函数名中写的是 SteerAngles，但 OSQP 求解的变量不是单独的转角序列，而是完整决策向量:
 *
 *   v = [X, U, S]^T
 *
 * 其中:
 *   X = [lat_0, yaw_0, lat_1, yaw_1, ..., lat_{N-1}, yaw_{N-1}]^T
 *       表示每个参考点上的横向误差和航向误差。
 *   U = [steer_0, steer_1, ..., steer_{N-2}]^T
 *       表示相邻参考点区间上的控制输入，也就是优化出的转向角。
 *   S 表示软碰撞约束的 slack variables。如果没有软约束，N_slack 为 0。
 *
 * 输入的 obj_mat/const_mat 已经把 MPT 问题整理成 OSQP 标准形式:
 *
 *   min_v  1/2 * v^T H v + f^T v
 *   s.t.   lb <= A v <= ub
 *
 * 这里的 H/f 来自目标函数，A/lb/ub 来自状态方程、碰撞边界、固定点和转向限制。
 *
 * 本函数内部有两层 warm start 概念:
 *   1. manual warm start:
 *      从上一帧轨迹构造一个完整初始解 u0，并令 v = u0 + delta_v。
 *      代入目标函数可得:
 *
 *        1/2 (u0 + delta_v)^T H (u0 + delta_v) + f^T (u0 + delta_v)
 *      = 1/2 delta_v^T H delta_v + (f + H u0)^T delta_v + const
 *
 *      常数项不影响最优解，所以只需要把梯度更新为 f' = f + H u0。
 *      约束同理:
 *
 *        lb <= A (u0 + delta_v) <= ub
 *      => lb - A u0 <= A delta_v <= ub - A u0
 *
 *      OSQP 求出来的是 delta_v，函数返回前再加回 u0，得到真正的 v。
 *
 *   2. OSQP warm start:
 *      如果上一帧求解成功且矩阵尺寸不变，就复用 solver 对象，只更新 H/f/A/bounds。
 *      这主要减少初始化开销，与上面的变量平移不是同一个概念。
 */
std::optional<Eigen::VectorXd> MPTOptimizer::calcOptimizedSteerAngles(
  const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
  const ConstraintMatrix & const_mat)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t N_v = N_x + N_u + N_ref * N_slack;

  // 手动 warm start 使用上一帧优化结果构造 u0，u0 必须和完整决策变量 v=[X,U,S] 同维度。
  const auto u0 = [&]() -> std::optional<Eigen::VectorXd> {
    if (mpt_param_.enable_manual_warm_start) {
      if (prev_ref_points_ptr_ && 1 < prev_ref_points_ptr_->size()) {
        return calcInitialSolutionForManualWarmStart(ref_points, *prev_ref_points_ptr_);
      }
    }
    return std::nullopt;
  }();

  // 如果有 u0，就把原问题 v 改写成增量问题 delta_v，并对应平移目标函数和约束边界。
  const auto [updated_obj_mat, updated_const_mat] =
    updateMatrixForManualWarmStart(obj_mat, const_mat, u0);

  // OSQP 接收的标准 QP:
  //   min 1/2 v^T H v + f^T v
  //   s.t. lower_bound <= A v <= upper_bound
  const Eigen::MatrixXd & H = updated_obj_mat.hessian;
  const Eigen::MatrixXd & A = updated_const_mat.linear;
  const auto f = toStdVector(updated_obj_mat.gradient);
  const auto upper_bound = toStdVector(updated_const_mat.upper_bound);
  const auto lower_bound = toStdVector(updated_const_mat.lower_bound);

  // 初始化或更新 OSQP。H 是对称矩阵，OSQP 的 P 矩阵只需要上三角 CSC 表示。
  time_keeper_->start_track("initOsqp");

  const autoware::osqp_interface::CSC_Matrix P_csc =
    autoware::osqp_interface::calCSCMatrixTrapezoidal(H);
  const autoware::osqp_interface::CSC_Matrix A_csc = autoware::osqp_interface::calCSCMatrix(A);
  if (
    // OSQP warm start: 上一帧求解成功且矩阵尺寸不变时，复用 solver 并只刷新数值。
    prev_solution_status_ == 1 && mpt_param_.enable_warm_start && prev_mat_n_ == H.rows() &&
    prev_mat_m_ == A.rows()) {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "warm start");
    osqp_solver_ptr_->updateCscP(P_csc);
    osqp_solver_ptr_->updateQ(f);
    osqp_solver_ptr_->updateCscA(A_csc);
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  } else {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "no warm start");
    osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(
      P_csc, A_csc, f, lower_bound, upper_bound, osqp_epsilon_);
  }
  prev_mat_n_ = H.rows();
  prev_mat_m_ = A.rows();
  time_keeper_->end_track("initOsqp");

  // 求解 QP，得到 primal_solution。若启用了 manual warm start，此处的解是 delta_v。
  time_keeper_->start_track("solveOsqp");
  const autoware::osqp_interface::OSQPResult osqp_result = osqp_solver_ptr_->optimize();
  time_keeper_->end_track("solveOsqp");

  // OSQP solution_status == 1 表示成功求得最优解。
  const int solution_status = osqp_result.solution_status;
  prev_solution_status_ = solution_status;
  if (solution_status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[MPT]");
    return std::nullopt;
  }

  // print iteration
  const int iteration_status = osqp_result.iteration_status;
  RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "iteration: %d", iteration_status);

  // 将 std::vector 形式的 OSQP 结果映射回 Eigen 向量，布局仍然是 [X,U,S]。
  auto optimization_result =
    osqp_result.primal_solution;  // NOTE: const cannot be added due to the next operation.

  const auto has_nan = std::any_of(
    optimization_result.begin(), optimization_result.end(),
    [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    return std::nullopt;
  }
  const Eigen::VectorXd optimized_variables =
    Eigen::Map<Eigen::VectorXd>(&optimization_result[0], N_v);

  // manual warm start 求解的是 delta_v，所以需要 v = u0 + delta_v 还原真正的优化变量。
  if (u0) {
    return static_cast<Eigen::VectorXd>(optimized_variables + *u0);
  }
  return optimized_variables;
}

/**
 * @brief 根据上一帧优化结果构造 manual warm start 的初始解 u0。
 *
 * u0 的布局必须与 OSQP 决策变量完全一致:
 *
 *   u0 = [X0, U0, S0]^T
 *
 * 当前帧参考点会相对上一帧向前移动，因此先在上一帧参考点中寻找当前帧第一个参考点
 * 对应的 nearest_idx，再从该位置开始拷贝上一帧的状态、转角和 slack variables。
 * u0 不要求满足当前帧的约束，它只是变量平移 v = u0 + delta_v 的展开中心。
 */
Eigen::VectorXd MPTOptimizer::calcInitialSolutionForManualWarmStart(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<ReferencePoint> & prev_ref_points) const
{
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t N_v = N_x + N_u + N_ref * N_slack;

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(N_v);

  // 将当前帧第一个参考点对齐到上一帧轨迹，避免直接从上一帧起点拷贝造成时间错位。
  const size_t nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    prev_ref_points, ref_points.front().pose, ego_nearest_param_.dist_threshold,
    ego_nearest_param_.yaw_threshold);

  // 拷贝上一帧的横向误差和航向误差，作为当前帧每个状态变量 X_i 的展开中心。
  for (size_t i = 0; i < N_ref; ++i) {
    const size_t prev_target_idx = std::min(nearest_idx + i, prev_ref_points.size() - 1);
    const auto & prev_state = prev_ref_points.at(prev_target_idx).optimized_kinematic_state;
    u0(i * D_x) = prev_state.lat;
    u0(i * D_x + 1) = prev_state.yaw;
  }

  // 拷贝上一帧的转向角输入，写入完整变量向量中的 U 区间。
  for (size_t i = 0; i < N_u; ++i) {
    const size_t prev_target_idx = std::min(nearest_idx + i, prev_ref_points.size() - 1);
    u0(N_x + i) = prev_ref_points.at(prev_target_idx).optimized_input;
  }

  // slack variables 描述上一轮软约束违反量，也从上一帧对应点读取并写入 S 区间。
  for (size_t i = 0; i < N_ref; ++i) {
    const size_t prev_target_idx = std::min(nearest_idx + i, prev_ref_points.size() - 1);
    const auto & slack_variables = prev_ref_points.at(prev_target_idx).slack_variables;
    if (slack_variables) {
      for (size_t j = 0; j < std::min(slack_variables->size(), N_slack); ++j) {
        u0(N_x + N_u + i * N_slack + j) = slack_variables->at(j);
      }
    }
  }

  return u0;
}

std::pair<MPTOptimizer::ObjectiveMatrix, MPTOptimizer::ConstraintMatrix>
MPTOptimizer::updateMatrixForManualWarmStart(
  const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
  const std::optional<Eigen::VectorXd> & u0) const
{
  if (!u0) {
    // 没有 manual warm start 时，直接使用原始 QP 矩阵。
    return {obj_mat, const_mat};
  }

  const Eigen::MatrixXd & H = obj_mat.hessian;
  const Eigen::MatrixXd & A = const_mat.linear;

  auto updated_obj_mat = obj_mat;
  auto updated_const_mat = const_mat;

  Eigen::VectorXd & f = updated_obj_mat.gradient;
  Eigen::VectorXd & ub = updated_const_mat.upper_bound;
  Eigen::VectorXd & lb = updated_const_mat.lower_bound;

  // 变量平移 v = u0 + delta_v 后:
  //   1/2 v^T H v + f^T v
  // = 1/2 delta_v^T H delta_v + (f + H u0)^T delta_v + const
  // 常数项不影响优化，因此只需要更新线性项。
  f += H * *u0;

  // 约束平移:
  //   lb <= A (u0 + delta_v) <= ub
  // => lb - A u0 <= A delta_v <= ub - A u0
  const Eigen::VectorXd A_times_u0 = A * *u0;
  ub -= A_times_u0;
  lb -= A_times_u0;

  return {updated_obj_mat, updated_const_mat};
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::calcMPTPoints(
  std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & optimized_variables,
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();

  const Eigen::VectorXd states = optimized_variables.segment(0, N_x);
  const Eigen::VectorXd steer_angles = optimized_variables.segment(N_x, N_u);
  const Eigen::VectorXd slack_variables = optimized_variables.segment(N_x + N_u, N_ref * N_slack);

  // calculate trajectory points from optimization result
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < N_ref; ++i) {
    auto & ref_point = ref_points.at(i);

    const double lat_error = states(i * D_x);
    const double yaw_error = states(i * D_x + 1);

    // validate optimization result
    if (mpt_param_.enable_optimization_validation) {
      if (
        mpt_param_.max_validation_lat_error < std::abs(lat_error) ||
        mpt_param_.max_validation_yaw_error < std::abs(yaw_error)) {
        return std::nullopt;
      }
    }

    // memorize optimization result (optimized_kinematic_state and optimized_input)
    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};
    if (i == N_ref - 1) {
      ref_point.optimized_input = 0.0;
    } else {
      ref_point.optimized_input = steer_angles(i * D_u);
    }

    std::vector<double> tmp_slack_variables;
    for (size_t j = 0; j < N_slack; ++j) {
      tmp_slack_variables.push_back(slack_variables(i * N_slack + j));
    }
    ref_point.slack_variables = tmp_slack_variables;

    // update pose and velocity
    TrajectoryPoint traj_point;
    traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
    traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;

    traj_points.push_back(traj_point);
  }

  return traj_points;
}

void MPTOptimizer::publishDebugTrajectories(
  const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & mpt_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // reference points
  const auto ref_traj = autoware::motion_utils::convertToTrajectory(
    trajectory_utils::convertToTrajectoryPoints(ref_points), header);
  debug_ref_traj_pub_->publish(ref_traj);

  // fixed reference points
  const auto fixed_traj_points = extractFixedPoints(ref_points);
  const auto fixed_traj = autoware::motion_utils::convertToTrajectory(fixed_traj_points, header);
  debug_fixed_traj_pub_->publish(fixed_traj);

  // mpt points
  const auto mpt_traj = autoware::motion_utils::convertToTrajectory(mpt_traj_points, header);
  debug_mpt_traj_pub_->publish(mpt_traj);
}

std::vector<TrajectoryPoint> MPTOptimizer::extractFixedPoints(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> fixed_traj_points;
  for (const auto & ref_point : ref_points) {
    if (ref_point.fixed_kinematic_state) {
      TrajectoryPoint fixed_traj_point;
      fixed_traj_point.pose = ref_point.offsetDeviation(
        ref_point.fixed_kinematic_state->lat, ref_point.fixed_kinematic_state->yaw);
      fixed_traj_points.push_back(fixed_traj_point);
    }
  }

  return fixed_traj_points;
}

double MPTOptimizer::getTrajectoryLength() const
{
  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;
  return forward_traj_length + backward_traj_length;
}

double MPTOptimizer::getDeltaArcLength() const
{
  return mpt_param_.delta_arc_length;
}

int MPTOptimizer::getNumberOfPoints() const
{
  return mpt_param_.num_points;
}

size_t MPTOptimizer::getNumberOfSlackVariables() const
{
  if (mpt_param_.soft_constraint) {
    if (mpt_param_.l_inf_norm) {
      // 如果使用 L∞ 范数，只返回 1 个松弛变量。L∞ 范数的特点是取所有约束 violation 的最大值，因此只需要一个松弛变量来表示最大的约束违反程度。
      return 1;
    }
    return vehicle_circle_longitudinal_offsets_.size(); //存储了用于近似车辆形状的多个圆形的纵向偏移量。每个圆形都需要一个独立的松弛变量来处理其对应的碰撞约束。
  }
  return 0;
}

/**
 * @brief 计算归一化的避让成本
 * 
 * 该函数根据参考点的边界约束和参数配置，计算一个归一化到 [0, 1] 范围的避让成本值。
 * 当车辆偏离期望路径时，该成本用于在优化过程中惩罚过大的横向偏移。
 * 
 * 计算逻辑：
 * 1. 首先计算负向避让成本，取左右边界的较小值（考虑安全裕度）
 * 2. 如果负向避让成本 >= 0，说明当前点在安全范围内，无需避让，返回空值
 * 3. 否则将负向避让成本归一化到 [0, 1] 范围并返回
 * 
 * @param ref_point 参考点，包含边界信息（上下界）
 * @return std::optional<double> 归一化的避让成本值 [0, 1]，如果不需要避让则返回空值
 */
std::optional<double> MPTOptimizer::calcNormalizedAvoidanceCost(
  const ReferencePoint & ref_point) const
{
  // 计算负向避让成本：取左右边界距离的最小值（减去安全裕度后取负）
  const double negative_avoidance_cost = std::min(
    -ref_point.bounds.lower_bound - mpt_param_.avoidance_cost_margin,
    ref_point.bounds.upper_bound - mpt_param_.avoidance_cost_margin);
  
  // 如果负向避让成本 >= 0，说明车辆在安全边界内，不需要避让
  if (0 <= negative_avoidance_cost) {
    return {};
  }
  
  // 将负向避让成本归一化到 [0, 1] 范围
  return std::clamp(-negative_avoidance_cost / mpt_param_.max_avoidance_cost, 0.0, 1.0);
}
}  // namespace autoware::path_optimizer
