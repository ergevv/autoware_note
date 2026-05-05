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

#include "autoware/path_optimizer/node.hpp"

#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_optimizer/debug_marker.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"
#include "rclcpp/time.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

[[maybe_unused]] StringStamped createStringStamped(
  const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

[[maybe_unused]] Float64Stamped createFloat64Stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

void setZeroVelocityAfterStopPoint(std::vector<TrajectoryPoint> & traj_points)
{
  const auto opt_zero_vel_idx = autoware::motion_utils::searchZeroVelocityIndex(traj_points);
  if (opt_zero_vel_idx) {
    for (size_t i = opt_zero_vel_idx.value(); i < traj_points.size(); ++i) {
      traj_points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }
}

bool hasZeroVelocity(const TrajectoryPoint & traj_point)
{
  constexpr double zero_vel = 0.0001;
  return std::abs(traj_point.longitudinal_velocity_mps) < zero_vel;
}

std::vector<double> calcSegmentLengthVector(const std::vector<TrajectoryPoint> & points)
{
  std::vector<double> segment_length_vector;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const double segment_length = autoware_utils::calc_distance2d(points.at(i), points.at(i + 1));
    segment_length_vector.push_back(segment_length);
  }
  return segment_length_vector;
}
}  // namespace

/**
 * @brief PathOptimizer节点的构造函数
 * 
 * 初始化路径优化器节点，包括：
 * - ROS2接口（发布器和订阅器）的创建
 * - 参数声明和加载
 * - 核心算法模块的初始化
 * - 诊断工具的注册
 * - 调试功能的配置
 * 
 * @param node_options ROS2节点选项，包含节点配置参数
 */
PathOptimizer::PathOptimizer(const rclcpp::NodeOptions & node_options)
: Node("path_optimizer", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>()),
  conditional_timer_(std::make_shared<ConditionalTimer>())
{
  // 创建输出轨迹和虚拟墙的发布器
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
  virtual_wall_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

  // 创建输入路径的订阅器
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&PathOptimizer::onPath, this, std::placeholders::_1));

  // 创建调试信息发布器：扩展轨迹、标记、计算时间和处理时间详情
  debug_extended_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_traj", 1);
  debug_markers_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_calculation_time_str_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);
  debug_calculation_time_float_pub_ =
    create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  debug_processing_time_detail_pub_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail_ms", 1);

  {  // 声明和加载所有配置参数
    // 功能选项参数
    enable_outside_drivable_area_stop_ =
      declare_parameter<bool>("option.enable_outside_drivable_area_stop");
    enable_skip_optimization_ = declare_parameter<bool>("option.enable_skip_optimization");
    enable_reset_prev_optimization_ =
      declare_parameter<bool>("option.enable_reset_prev_optimization");
    use_footprint_polygon_for_outside_drivable_area_check_ =
      declare_parameter<bool>("option.use_footprint_polygon_for_outside_drivable_area_check");

    // 调试标记发布选项
    enable_pub_debug_marker_ = declare_parameter<bool>("option.debug.enable_pub_debug_marker");
    enable_pub_extra_debug_marker_ =
      declare_parameter<bool>("option.debug.enable_pub_extra_debug_marker");

    // 调试信息启用选项
    enable_debug_info_ = declare_parameter<bool>("option.debug.enable_debug_info");

    // 可行驶区域外的停车边距
    vehicle_stop_margin_outside_drivable_area_ =
      declare_parameter<double>("common.vehicle_stop_margin_outside_drivable_area");

    // 自车最近点搜索参数
    ego_nearest_param_ = EgoNearestParam(this);

    // 轨迹相关参数
    traj_param_ = TrajectoryParam(this);

    // 注册诊断检查器
    {
      updater_.setHardwareID("path_optimizer");
      updater_.add(
        "path_optimizer_emergency_stop", this, &PathOptimizer::onCheckPathOptimizationValid);
    }
  }

  // 创建时间性能追踪器
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);

  // 初始化核心算法模块：重规划检查器和MPT优化器
  replan_checker_ptr_ = std::make_shared<ReplanChecker>(this, ego_nearest_param_);
  mpt_optimizer_ptr_ = std::make_shared<MPTOptimizer>(
    this, enable_debug_info_, ego_nearest_param_, vehicle_info_, traj_param_, debug_data_ptr_,
    time_keeper_);

  // 重置规划器状态（必须在核心算法初始化后调用）
  initializePlanning();

  // 设置参数回调函数（必须在核心算法初始化后调用）
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PathOptimizer::onParam, this, std::placeholders::_1));

  // 初始化日志级别配置和发布时间发布器
  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

rcl_interfaces::msg::SetParametersResult PathOptimizer::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  // 功能选项参数
  update_param<bool>(
    parameters, "option.enable_outside_drivable_area_stop", enable_outside_drivable_area_stop_);
  update_param<bool>(parameters, "option.enable_skip_optimization", enable_skip_optimization_);
  update_param<bool>(
    parameters, "option.enable_reset_prev_optimization", enable_reset_prev_optimization_);
  update_param<bool>(
    parameters, "option.use_footprint_polygon_for_outside_drivable_area_check",
    use_footprint_polygon_for_outside_drivable_area_check_);

  // 调试标记发布选项
  update_param<bool>(parameters, "option.debug.enable_pub_debug_marker", enable_pub_debug_marker_);
  update_param<bool>(
    parameters, "option.debug.enable_pub_extra_debug_marker", enable_pub_extra_debug_marker_);

  // 调试信息启用选项
  update_param<bool>(parameters, "option.debug.enable_debug_info", enable_debug_info_);

  update_param<double>(
    parameters, "common.vehicle_stop_margin_outside_drivable_area",
    vehicle_stop_margin_outside_drivable_area_);

  // 自车最近点搜索参数
  ego_nearest_param_.onParam(parameters);

  // 轨迹相关参数
  traj_param_.onParam(parameters);

  // 核心算法模块参数
  replan_checker_ptr_->onParam(parameters);
  mpt_optimizer_ptr_->onParam(parameters);

  // 重置规划器状态
  initializePlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PathOptimizer::initializePlanning()
{
  RCLCPP_DEBUG(get_logger(), "Initialize planning");

  mpt_optimizer_ptr_->initialize(enable_debug_info_, traj_param_);

  resetPreviousData();
}

void PathOptimizer::resetPreviousData()
{
  mpt_optimizer_ptr_->resetPreviousData();
}

/**
 * @brief 处理接收到的路径消息并进行轨迹优化
 * 
 * 这是路径优化器的核心回调函数，负责将输入的路径转换为优化的轨迹。
 * 主要流程包括：
 * 1. 验证输入路径和自车里程计数据的有效性
 * 2. 检查路径行驶方向（暂不支持后退路径）
 * 3. 创建规划器所需的数据结构
 * 4. 生成优化后的轨迹点
 * 5. 扩展轨迹以平滑连接优化轨迹和后续路径
 * 6. 在停止点后设置零速度
 * 7. 发布调试数据和最终轨迹
 * 
 * @param path_ptr 输入路径的智能指针，包含待优化的路径点信息
 * 
 * @return void 无返回值，结果通过轨迹发布器发布
 * 
 * @note 如果输入路径无效、里程计数据缺失或路径为后退方向，函数会提前返回
 * @note 后退路径目前不被支持，会直接转换为轨迹并发布警告信息
 * @note 计算时间会在所有处理完成后记录并发布
 */
void PathOptimizer::onPath(const Path::ConstSharedPtr path_ptr)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_.tic();

  // 验证输入路径的有效性
  if (!checkInputPath(*path_ptr, *get_clock())) {
    return;
  }

  // 获取并验证自车里程计数据
  const auto ego_odom_ptr = ego_odom_sub_.take_data();
  if (!ego_odom_ptr) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for ego pose and twist.");
    return;
  }

  // 检查路径是否为后退方向，后退路径暂不支持
  const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
  if (!is_driving_forward) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");

    const auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    const auto output_traj_msg =
      autoware::motion_utils::convertToTrajectory(traj_points, path_ptr->header);
    traj_pub_->publish(output_traj_msg);
    published_time_publisher_->publish_if_subscribed(traj_pub_, output_traj_msg.header.stamp);
    return;
  }

  // 创建规划器数据结构，整合路径和自车状态信息
  const auto planner_data = createPlannerData(*path_ptr, ego_odom_ptr);

  // 基于MPT算法生成优化轨迹
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

  // 扩展轨迹以确保与后续路径的平滑连接
  auto full_traj_points = extendTrajectory(planner_data.traj_points, optimized_traj_points);

  // 在检测到停止点后将后续所有轨迹点的速度设为零
  setZeroVelocityAfterStopPoint(full_traj_points);

  // 发布调试数据和更新诊断信息
  publishDebugData(planner_data.header);
  updater_.force_update();

  // 发布计算耗时（必须在测量完成后调用）
  debug_calculation_time_float_pub_->publish(createFloat64Stamped(now(), stop_watch_.toc()));

  const auto output_traj_msg =
    autoware::motion_utils::convertToTrajectory(full_traj_points, path_ptr->header);
  traj_pub_->publish(output_traj_msg);
  published_time_publisher_->publish_if_subscribed(traj_pub_, output_traj_msg.header.stamp);
}

bool PathOptimizer::checkInputPath(const Path & path, rclcpp::Clock clock) const
{
  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

void PathOptimizer::onCheckPathOptimizationValid(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (is_optimization_failed_) {
    const std::string error_msg =
      "[Path Optimizer]: Emergency Brake due to prolonged MPT Optimizer failure";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
  } else {
    const std::string error_msg = "[Path Optimizer]: MPT Optimizer successful";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }
}

PlannerData PathOptimizer::createPlannerData(
  const Path & path, const Odometry::ConstSharedPtr ego_odom_ptr) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.ego_pose = ego_odom_ptr->pose.pose;
  planner_data.ego_vel = ego_odom_ptr->twist.twist.linear.x;

  debug_data_ptr_->ego_pose = planner_data.ego_pose;
  return planner_data;
}

std::vector<TrajectoryPoint> PathOptimizer::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. calculate trajectory with MPT
  //    NOTE: This function may return previously optimized trajectory points.
  //          Also, velocity on some points will not be updated for a logic purpose.
  auto optimized_traj_points = optimizeTrajectory(planner_data);

  // 2. insert zero velocity when trajectory is over drivable area
  insertZeroVelocityOutsideDrivableArea(planner_data, optimized_traj_points);

  // 3. publish debug marker
  publishDebugMarkerOfOptimization(optimized_traj_points);

  return optimized_traj_points;
}

/**
 * @brief 优化轨迹，使其满足运动学可行性和无碰撞约束
 * 
 * 该函数是路径优化器的核心方法，负责根据规划数据生成优化的轨迹。
 * 主要功能包括：
 * 1. 判断是否需要重新规划（基于历史数据和当前状态）
 * 2. 使用模型预测轨迹(MPT)优化器生成运动学可行且无碰撞的轨迹
 * 3. 处理优化失败的情况（超时或使用历史轨迹）
 * 4. 更新轨迹点的速度信息
 * 
 * @param planner_data 规划器数据，包含当前车辆状态、路径点、自车位姿等信息
 * @return std::vector<TrajectoryPoint> 优化后的轨迹点序列
 *         - 如果不需要重规划且存在历史优化轨迹，返回历史优化轨迹
 *         - 如果跳过优化，返回输入的路径点
 *         - 如果MPT优化成功，返回优化后的轨迹
 *         - 如果优化失败但超时，返回输入路径点
 *         - 如果优化失败但未超时，返回历史优化轨迹
 */
std::vector<TrajectoryPoint> PathOptimizer::optimizeTrajectory(const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  is_optimization_failed_ = false;
  const auto & p = planner_data;

  // 判断是否需要重新规划
  // 如果需要重置之前的优化或检测到需要重规划的条件，则执行完整优化
  // 否则复用之前的优化结果以提高效率
  const bool is_replan_required = [&]() {
    const bool reset_prev_optimization = replan_checker_ptr_->isResetRequired(planner_data);
    if (enable_reset_prev_optimization_ || reset_prev_optimization) {
      // NOTE: always replan when resetting previous optimization
      resetPreviousData();
      return true;
    }
    // check replan when not resetting previous optimization
    return replan_checker_ptr_->isReplanRequired(planner_data, now());
  }();
  replan_checker_ptr_->updateData(planner_data, is_replan_required, now());
  if (!is_replan_required) {
    return getPrevOptimizedTrajectory(p.traj_points);
  }

  if (enable_skip_optimization_) {
    return p.traj_points;
  }

  // 使用模型预测轨迹(MPT)优化器进行轨迹优化
  // MPT优化器确保轨迹满足运动学约束且在可行驶区域内无碰撞
  const auto mpt_traj = mpt_optimizer_ptr_->optimizeTrajectory(planner_data);

  const double elapsed_time = conditional_timer_->getElapsedTime().count();
  const bool elapsed_time_over_three_seconds = (elapsed_time > 3.0);

  // 根据优化结果和耗时决定最终使用的轨迹
  // 优先级：MPT优化结果 > 超时时的输入轨迹 > 历史优化轨迹
  auto optimized_traj_points = [&]() {
    if (mpt_traj) {
      return std::move(*mpt_traj);
    }
    if (elapsed_time_over_three_seconds) {
      return p.traj_points;
    }
    return getPrevOptimizedTrajectory(p.traj_points);
  }();

  const bool optimized_traj_failed = !static_cast<bool>(mpt_traj);
  conditional_timer_->update(optimized_traj_failed);
  is_optimization_failed_ = optimized_traj_failed && elapsed_time_over_three_seconds;

  // 更新轨迹点的速度信息
  // 即使优化失败或被跳过，也需要更新速度，因为输入轨迹的速度可能已发生变化
  applyInputVelocity(optimized_traj_points, p.traj_points, planner_data.ego_pose);

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> PathOptimizer::getPrevOptimizedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto prev_optimized_traj_points = mpt_optimizer_ptr_->getPrevOptimizedTrajectoryPoints();
  if (prev_optimized_traj_points) {
    return *prev_optimized_traj_points;
  }
  return traj_points;
}

void PathOptimizer::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj_points,
  const std::vector<TrajectoryPoint> & input_traj_points,
  const geometry_msgs::msg::Pose & ego_pose) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // trim to ego-pose
  const size_t ego_seg_idx_output_traj =
    trajectory_utils::findEgoSegmentIndex(output_traj_points, ego_pose, ego_nearest_param_);
  output_traj_points = autoware::motion_utils::cropBackwardPoints(
    output_traj_points, ego_pose.position, ego_seg_idx_output_traj,
    traj_param_.output_backward_traj_length);

  // crop forward for faster calculation
  const auto forward_cropped_input_traj_points = [&]() {
    const double optimized_traj_length = mpt_optimizer_ptr_->getTrajectoryLength();
    constexpr double margin_traj_length = 10.0;

    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_traj_points, ego_pose, ego_nearest_param_);
    const auto cropped_points = autoware::motion_utils::cropForwardPoints(
      input_traj_points, ego_pose.position, ego_seg_idx,
      optimized_traj_length + margin_traj_length);

    if (cropped_points.size() < 2) {
      return input_traj_points;
    }
    return cropped_points;
  }();

  // update velocity
  const auto segment_length_vec = calcSegmentLengthVector(forward_cropped_input_traj_points);
  const double mpt_delta_arc_length = mpt_optimizer_ptr_->getDeltaArcLength();
  size_t input_traj_start_idx = trajectory_utils::findEgoSegmentIndex(
    forward_cropped_input_traj_points, output_traj_points.front().pose, ego_nearest_param_);
  for (size_t i = 0; i < output_traj_points.size(); i++) {
    // NOTE: input_traj_start/end_idx is calculated for efficient index calculation
    const size_t input_traj_end_idx = [&]() {
      double sum_segment_length = 0.0;
      for (size_t j = input_traj_start_idx + 1; j < segment_length_vec.size(); ++j) {
        sum_segment_length += segment_length_vec.at(j);
        if (mpt_delta_arc_length < sum_segment_length) {
          return j + 1;
        }
      }
      return forward_cropped_input_traj_points.size() - 1;
    }();

    const auto nearest_traj_point = [&]() {
      if (input_traj_start_idx == input_traj_end_idx) {
        return forward_cropped_input_traj_points.at(input_traj_start_idx);
      }

      // crop forward and backward for efficient calculation
      const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
        forward_cropped_input_traj_points.begin() + input_traj_start_idx,
        forward_cropped_input_traj_points.begin() + input_traj_end_idx + 1};
      assert(2 <= cropped_input_traj_points.size());

      const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
        cropped_input_traj_points, output_traj_points.at(i).pose, ego_nearest_param_);
      input_traj_start_idx += nearest_seg_idx;

      return cropped_input_traj_points.at(nearest_seg_idx);
    }();

    // calculate velocity with zero order hold
    output_traj_points.at(i).longitudinal_velocity_mps =
      nearest_traj_point.longitudinal_velocity_mps;
  }

  // insert stop point explicitly
  const auto stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(forward_cropped_input_traj_points);
  if (stop_idx) {
    const auto & input_stop_pose = forward_cropped_input_traj_points.at(stop_idx.value()).pose;
    // NOTE: autoware::motion_utils::findNearestSegmentIndex is used instead of
    // trajectory_utils::findEgoSegmentIndex
    //       for the case where input_traj_points is much longer than output_traj_points, and the
    //       former has a stop point but the latter will not have.
    const auto stop_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
      output_traj_points, input_stop_pose, ego_nearest_param_.dist_threshold,
      ego_nearest_param_.yaw_threshold);

    // calculate and insert stop pose on output trajectory
    const bool is_stop_point_inside_trajectory = [&]() {
      if (!stop_seg_idx) {
        return false;
      }
      if (*stop_seg_idx == output_traj_points.size() - 2) {
        const double signed_projected_length_to_segment =
          autoware::motion_utils::calcLongitudinalOffsetToSegment(
            output_traj_points, *stop_seg_idx, input_stop_pose.position);
        const double segment_length = autoware::motion_utils::calcSignedArcLength(
          output_traj_points, *stop_seg_idx, *stop_seg_idx + 1);
        if (segment_length < signed_projected_length_to_segment) {
          // NOTE: input_stop_pose is outside output_traj_points.
          return false;
        }
      }
      return true;
    }();
    if (is_stop_point_inside_trajectory) {
      trajectory_utils::insertStopPoint(output_traj_points, input_stop_pose, *stop_seg_idx);
    }
  }
}

void PathOptimizer::insertZeroVelocityOutsideDrivableArea(
  const PlannerData & planner_data, std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (optimized_traj_points.empty()) {
    return;
  }

  // 1. calculate ego_index nearest to optimized_traj_points
  const size_t ego_idx = trajectory_utils::findEgoIndex(
    optimized_traj_points, planner_data.ego_pose, ego_nearest_param_);

  // 2. calculate an end point to check being outside the drivable area
  // NOTE: Some terminal trajectory points tend to be outside the drivable area when
  //       they have high curvature.
  //       Therefore, these points should be ignored to check if being outside the drivable area
  constexpr int num_points_ignore_drivable_area = 5;
  const int end_idx = std::min(
    static_cast<int>(optimized_traj_points.size()) - 1,
    mpt_optimizer_ptr_->getNumberOfPoints() - num_points_ignore_drivable_area);

  // 3. assign zero velocity to the first point being outside the drivable area
  const auto first_outside_idx = [&]() -> std::optional<size_t> {
    for (size_t i = ego_idx; i < static_cast<size_t>(end_idx); ++i) {
      const auto & traj_point = optimized_traj_points.at(i);

      // check if the footprint is outside the drivable area
      const bool is_outside = geometry_utils::isOutsideDrivableAreaFromRectangleFootprint(
        traj_point.pose, planner_data.left_bound, planner_data.right_bound, vehicle_info_,
        use_footprint_polygon_for_outside_drivable_area_check_);

      if (is_outside) {
        return i;
      }
    }
    return std::nullopt;
  }();

  if (first_outside_idx) {
    debug_data_ptr_->stop_pose_by_drivable_area = optimized_traj_points.at(*first_outside_idx).pose;
    const auto stop_idx = [&]() {
      const auto dist =
        autoware::motion_utils::calcSignedArcLength(optimized_traj_points, 0, *first_outside_idx);
      const auto dist_with_margin = dist - vehicle_stop_margin_outside_drivable_area_;
      const auto first_outside_idx_with_margin =
        autoware::motion_utils::insertTargetPoint(0, dist_with_margin, optimized_traj_points);
      if (first_outside_idx_with_margin) {
        return *first_outside_idx_with_margin;
      }
      return *first_outside_idx;
    }();

    publishVirtualWall(optimized_traj_points.at(stop_idx).pose);

    if (enable_outside_drivable_area_stop_) {
      for (size_t i = stop_idx; i < optimized_traj_points.size(); ++i) {
        optimized_traj_points.at(i).longitudinal_velocity_mps = 0.0;
      }
    }
  } else {
    debug_data_ptr_->stop_pose_by_drivable_area = std::nullopt;
  }
}

void PathOptimizer::publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  auto virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    stop_pose, "outside drivable area", now(), 0, vehicle_info_.max_longitudinal_offset_m);
  if (!enable_outside_drivable_area_stop_) {
    virtual_wall_marker.markers.front().color =
      autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.5);
  }

  virtual_wall_pub_->publish(virtual_wall_marker);
}

void PathOptimizer::publishDebugMarkerOfOptimization(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!enable_pub_debug_marker_) {
    return;
  }

  // debug marker
  time_keeper_->start_track("getDebugMarker");
  const auto debug_marker =
    getDebugMarker(*debug_data_ptr_, traj_points, vehicle_info_, enable_pub_extra_debug_marker_);
  time_keeper_->end_track("getDebugMarker");

  time_keeper_->start_track("publishDebugMarker");
  debug_markers_pub_->publish(debug_marker);
  time_keeper_->end_track("publishDebugMarker");
}

std::vector<TrajectoryPoint> PathOptimizer::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & joint_start_pose = optimized_traj_points.back().pose;

  // calculate end idx of optimized points on path points
  const size_t joint_start_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, joint_start_pose, ego_nearest_param_);

  // crop trajectory for extension
  constexpr double joint_traj_max_length_for_smoothing = 15.0;
  constexpr double joint_traj_min_length_for_smoothing = 5.0;
  const auto joint_end_traj_point_idx = trajectory_utils::getPointIndexAfter(
    traj_points, joint_start_pose.position, joint_start_traj_seg_idx,
    joint_traj_max_length_for_smoothing, joint_traj_min_length_for_smoothing);
  if (!joint_end_traj_point_idx) {
    return trajectory_utils::resampleTrajectoryPoints(
      optimized_traj_points, traj_param_.output_delta_arc_length);
  }

  // calculate full trajectory points
  const auto full_traj_points = [&]() {
    auto extended_traj_points = std::vector<TrajectoryPoint>{
      traj_points.begin() + *joint_end_traj_point_idx, traj_points.end()};

    if (!extended_traj_points.empty() && !optimized_traj_points.empty()) {
      // NOTE: Without this code, if optimized_traj_points's back is non zero velocity and
      // extended_traj_points' front
      //       is zero velocity, the zero velocity will be inserted in the whole joint trajectory.
      //       The input stop point will be inserted explicitly in the latter part.
      extended_traj_points.front().longitudinal_velocity_mps =
        optimized_traj_points.back().longitudinal_velocity_mps;
    }
    return concatVectors(optimized_traj_points, extended_traj_points);
  }();

  // resample trajectory points
  auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    full_traj_points, traj_param_.output_delta_arc_length);

  // update stop velocity on joint
  for (size_t i = joint_start_traj_seg_idx + 1; i <= *joint_end_traj_point_idx; ++i) {
    if (hasZeroVelocity(traj_points.at(i))) {
      if (i != 0 && !hasZeroVelocity(traj_points.at(i - 1))) {
        // Here is when current point is 0 velocity, but previous point is not 0 velocity.
        const auto & input_stop_pose = traj_points.at(i).pose;
        const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
          resampled_traj_points, input_stop_pose, ego_nearest_param_);

        // calculate and insert stop pose on output trajectory
        trajectory_utils::insertStopPoint(resampled_traj_points, input_stop_pose, stop_seg_idx);
      }
    }
  }

  // debug_data_ptr_->extended_traj_points =
  //   extended_traj_points ? *extended_traj_points : std::vector<TrajectoryPoint>();
  return resampled_traj_points;
}

void PathOptimizer::publishDebugData(const Header & header) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // publish trajectories
  const auto debug_extended_traj =
    autoware::motion_utils::convertToTrajectory(debug_data_ptr_->extended_traj_points, header);
  debug_extended_traj_pub_->publish(debug_extended_traj);
}
}  // namespace autoware::path_optimizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_optimizer::PathOptimizer)
