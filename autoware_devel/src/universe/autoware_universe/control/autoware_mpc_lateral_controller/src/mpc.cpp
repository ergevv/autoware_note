// Copyright 2018-2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware_utils/math/unit_conversion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
using autoware_utils::calc_distance2d;
using autoware_utils::normalize_radian;
using autoware_utils::rad2deg;

MPC::MPC(rclcpp::Node & node)
{
  m_debug_frenet_predicted_trajectory_pub = node.create_publisher<Trajectory>(
    "~/debug/predicted_trajectory_in_frenet_coordinate", rclcpp::QoS(1));
  m_debug_resampled_reference_trajectory_pub =
    node.create_publisher<Trajectory>("~/debug/resampled_reference_trajectory", rclcpp::QoS(1));
}

ResultWithReason MPC::calculateMPC(
  const SteeringReport & current_steer, const Odometry & current_kinematics, Lateral & ctrl_cmd,
  Trajectory & predicted_trajectory, Float32MultiArrayStamped & diagnostic,
  LateralHorizon & ctrl_cmd_horizon)
{
  // since the reference trajectory does not take into account the current velocity of the ego
  // vehicle, it needs to calculate the trajectory velocity considering the longitudinal dynamics.
  // 背景：上游规划模块给出的轨迹速度 (vx)
  // 通常是理想化的，可能没有充分考虑车辆当前的实际速度和加速度限制（例如，规划说下一秒速度达到
  // 10m/s，但车辆当前只有 2m/s 且加速度有限）。 作用：根据车辆当前的实际速度、最大加速度限制
  // (acceleration_limit) 和时间常数，重新计算参考轨迹上每个点的“可达速度”。这确保了 MPC
  // 基于一个物理上可实现的纵向速度分布来进行横向控制预测。
  const auto reference_trajectory =
    applyVelocityDynamicsFilter(m_reference_trajectory, current_kinematics);

  // get the necessary data
  // 找到自车在参考轨迹上的最近点索引、时间、位姿。
  // 误差计算：计算横向误差 (lateral_err) 和航向角误差 (yaw_err)。
  // 转向预测：通过 SteeringPredictor 估算由于执行器延迟导致的“未来实际转向角” (predicted_steer)。
  // 有效性检查：检查参考轨迹是否足够长以覆盖预测时域。
  const auto [get_data_result, mpc_data] =
    getData(reference_trajectory, current_steer, current_kinematics);
  if (!get_data_result.result) {
    return ResultWithReason{false, fmt::format("getting MPC Data ({}).", get_data_result.reason)};
  }

  // calculate initial state of the error dynamics
  // 设置初始状态
  const auto x0 = getInitialState(mpc_data);

  // apply time delay compensation to the initial state
  // 背景：从计算控制指令到指令真正作用于车辆（转向轮转动）存在硬件延迟（input_delay）。MPC
  // 需要预测的是“延迟结束后”的状态，而不是“当前”状态。 逻辑：利用车辆模型和历史控制指令缓冲区
  // (m_input_buffer)，将初始状态 $x_0$ 向前推演 input_delay 时长，得到 $x_{0_delayed}$。 意义：确保
  // MPC 优化的起点是车辆在执行新指令时的真实预期状态，显著提高跟踪精度。
  const auto [success_delay, x0_delayed] =
    updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, x0);
  if (!success_delay) {
    return ResultWithReason{false, "delay compensation."};
  }

  // resample reference trajectory with mpc sampling time
  // MPC 预测的起始点是 当前最近点时间 + 输入延迟。
  const double mpc_start_time = mpc_data.nearest_time + m_param.input_delay;
  // 确定步长：调用 getPredictionDeltaTime 计算离散时间步长
  // prediction_dt。这个步长是动态调整的，以确保预测时域覆盖足够的纵向距离 (min_prediction_length)。
  // 重采样：MPC 需要在等时间间隔的点上进行离散化建模。此步骤通过线性插值，从参考轨迹中提取出 N
  // 个（prediction_horizon）等时间间隔的点，形成 mpc_resampled_ref_trajectory。
  const double prediction_dt =
    getPredictionDeltaTime(mpc_start_time, reference_trajectory, current_kinematics);

  const auto [resample_result, mpc_resampled_ref_trajectory] =
    resampleMPCTrajectoryByTime(mpc_start_time, prediction_dt, reference_trajectory);
  if (!resample_result.result) {
    return ResultWithReason{
      false, fmt::format("trajectory resampling ({}).", resample_result.reason)};
  }

  // generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex
  // $A_{ex}, B_{ex}, W_{ex}$：描述状态如何随控制和初始状态演化。
  // $Q_{ex}, R_{ex}$：权重矩阵，决定了对误差和控制量的惩罚程度（可能随速度或曲率自适应调整）。
  // $U_{ref_ex}$：前馈控制量参考值（通常由曲率决定 $\delta_{ff} = L \cdot k$）。
  const auto mpc_matrix = generateMPCMatrix(mpc_resampled_ref_trajectory, prediction_dt);

  // solve Optimization problem
  // 输入：预测矩阵、延迟后的初始状态、参考轨迹、当前车速。
  // 过程：
  // 构建二次规划（QP）问题的 H 矩阵和 f 向量。
  // 构建约束矩阵（转向角限制、转向角速度限制）。
  // 调用 QP 求解器（如 OSQP）求解最优控制序列 $U_{ex}$。
  // 输出：$U_{ex}$ 是一个向量，包含了未来 N 个时刻的最优转向角指令。
  const auto [opt_result, Uex] = executeOptimization(
    mpc_matrix, x0_delayed, prediction_dt, mpc_resampled_ref_trajectory,
    current_kinematics.twist.twist.linear.x);
  if (!opt_result.result) {
    return ResultWithReason{false, fmt::format("optimization failure ({}).", opt_result.reason)};
  }

  // apply filters for the input limitation and low pass filter
  const double u_saturated = std::clamp(Uex(0), -m_steer_lim, m_steer_lim);
  const double u_filtered = m_lpf_steering_cmd.filter(u_saturated);

  // set control command
  ctrl_cmd.steering_tire_angle = static_cast<float>(u_filtered);
  // 为了告诉底盘控制器以多快的速度转动方向盘，代码计算了从“当前实际转向角”到“目标转向角”的变化率，或者基于预测状态的转向角变化率。这有助于提高控制的动态响应性能。
  ctrl_cmd.steering_tire_rotation_rate = static_cast<float>(calcDesiredSteeringRate(
    mpc_matrix, x0_delayed, Uex, u_filtered, current_steer.steering_tire_angle, prediction_dt));

  // save the control command for the steering prediction
  m_steering_predictor->storeSteerCmd(u_filtered);

  // save input to buffer for delay compensation
  m_input_buffer.push_back(ctrl_cmd.steering_tire_angle);
  m_input_buffer.pop_front();

  // save previous input for the mpc rate limit
  m_raw_steer_cmd_pprev = m_raw_steer_cmd_prev;
  m_raw_steer_cmd_prev = Uex(0);

  /* calculate predicted trajectory */
  Eigen::VectorXd initial_state = m_use_delayed_initial_state ? x0_delayed : x0;
  predicted_trajectory = calculatePredictedTrajectory(
    mpc_matrix, initial_state, Uex, mpc_resampled_ref_trajectory, prediction_dt, "world");

  // Publish predicted trajectories in different coordinates for debugging purposes
  if (m_publish_debug_trajectories) {
    // Calculate and publish predicted trajectory in Frenet coordinate
    auto predicted_trajectory_frenet = calculatePredictedTrajectory(
      mpc_matrix, initial_state, Uex, mpc_resampled_ref_trajectory, prediction_dt, "frenet");
    predicted_trajectory_frenet.header.stamp = m_clock->now();
    predicted_trajectory_frenet.header.frame_id = "map";
    m_debug_frenet_predicted_trajectory_pub->publish(predicted_trajectory_frenet);
  }

  // prepare diagnostic message
  diagnostic =
    generateDiagData(reference_trajectory, mpc_data, mpc_matrix, ctrl_cmd, Uex, current_kinematics);

  // create LateralHorizon command
  // 计算出了未来一系列的最优转向角 $U_{ex}$。
  ctrl_cmd_horizon.time_step_ms = prediction_dt * 1000.0;
  ctrl_cmd_horizon.controls.clear();
  ctrl_cmd_horizon.controls.push_back(ctrl_cmd);
  for (auto it = std::next(Uex.begin()); it != Uex.end(); ++it) {
    Lateral lateral{};
    lateral.steering_tire_angle = static_cast<float>(std::clamp(*it, -m_steer_lim, m_steer_lim));
    lateral.steering_tire_rotation_rate =
      (lateral.steering_tire_angle - ctrl_cmd_horizon.controls.back().steering_tire_angle) /
      m_ctrl_period;
    ctrl_cmd_horizon.controls.push_back(lateral);
  }

  return ResultWithReason{true};
}

Float32MultiArrayStamped MPC::generateDiagData(
  const MPCTrajectory & reference_trajectory, const MPCData & mpc_data,
  const MPCMatrix & mpc_matrix, const Lateral & ctrl_cmd, const VectorXd & Uex,
  const Odometry & current_kinematics) const
{
  Float32MultiArrayStamped diagnostic;

  // prepare diagnostic message
  const double nearest_k = reference_trajectory.k.at(mpc_data.nearest_idx);
  const double nearest_smooth_k = reference_trajectory.smooth_k.at(mpc_data.nearest_idx);
  const double wb = m_vehicle_model_ptr->getWheelbase();
  const double current_velocity = current_kinematics.twist.twist.linear.x;
  const double wz_predicted = current_velocity * std::tan(mpc_data.predicted_steer) / wb;
  const double wz_measured = current_velocity * std::tan(mpc_data.steer) / wb;
  const double wz_command = current_velocity * std::tan(ctrl_cmd.steering_tire_angle) / wb;
  const int iteration_num = m_qpsolver_ptr->getTakenIter();
  const double runtime = m_qpsolver_ptr->getRunTime();
  const double objective_value = m_qpsolver_ptr->getObjVal();

  typedef decltype(diagnostic.data)::value_type DiagnosticValueType;
  const auto append_diag = [&](const auto & val) -> void {
    diagnostic.data.push_back(static_cast<DiagnosticValueType>(val));
  };
  append_diag(ctrl_cmd.steering_tire_angle);      // [0] final steering command (MPC + LPF)
  append_diag(Uex(0));                            // [1] mpc calculation result
  append_diag(mpc_matrix.Uref_ex(0));             // [2] feed-forward steering value
  append_diag(std::atan(nearest_smooth_k * wb));  // [3] feed-forward steering value raw
  append_diag(mpc_data.steer);                    // [4] current steering angle
  append_diag(mpc_data.lateral_err);              // [5] lateral error
  append_diag(tf2::getYaw(current_kinematics.pose.pose.orientation));  // [6] current_pose yaw
  append_diag(tf2::getYaw(mpc_data.nearest_pose.orientation));         // [7] nearest_pose yaw
  append_diag(mpc_data.yaw_err);                                       // [8] yaw error
  append_diag(reference_trajectory.vx.at(mpc_data.nearest_idx));       // [9] reference velocity
  append_diag(current_velocity);                                       // [10] measured velocity
  append_diag(wz_command);                           // [11] angular velocity from steer command
  append_diag(wz_measured);                          // [12] angular velocity from measured steer
  append_diag(current_velocity * nearest_smooth_k);  // [13] angular velocity from path curvature
  append_diag(nearest_smooth_k);          // [14] nearest path curvature (used for feed-forward)
  append_diag(nearest_k);                 // [15] nearest path curvature (not smoothed)
  append_diag(mpc_data.predicted_steer);  // [16] predicted steer
  append_diag(wz_predicted);              // [17] angular velocity from predicted steer
  append_diag(iteration_num);             // [18] iteration number
  append_diag(runtime);                   // [19] runtime of the latest problem solved
  append_diag(objective_value);           // [20] objective value of the latest problem solved

  return diagnostic;
}

void MPC::setReferenceTrajectory(
  const Trajectory & trajectory_msg, const TrajectoryFilteringParam & param,
  const Odometry & current_kinematics)
{
  // 在接收到的轨迹中找到距离自车当前位置最近的线段索引
  const size_t nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      trajectory_msg.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  // 计算自车位置在该线段上的纵向偏移量
  const double ego_offset_to_segment = autoware::motion_utils::calcLongitudinalOffsetToSegment(
    trajectory_msg.points, nearest_seg_idx, current_kinematics.pose.pose.position);

  // MPCTrajectory 通常包含 x, y, yaw, vx, k (曲率), smooth_k (平滑曲率), relative_time
  // 等数组，便于数学运算。
  const auto mpc_traj_raw = MPCUtils::convertToMPCTrajectory(trajectory_msg);

  // resampling
  // 使用样条插值（Spline Interpolation），按照固定的距离间隔
  // param.traj_resample_dist重新生成轨迹点。
  const auto [success_resample, mpc_traj_resampled] = MPCUtils::resampleMPCTrajectoryByDistance(
    mpc_traj_raw, param.traj_resample_dist, nearest_seg_idx, ego_offset_to_segment);
  if (!success_resample) {
    warn_throttle("[setReferenceTrajectory] spline error when resampling by distance");
    return;
  }

  // 判断车辆是前进还是后退。
  const auto is_forward_shift =
    autoware::motion_utils::isDrivingForward(mpc_traj_resampled.toTrajectoryPoints());

  // if driving direction is unknown, use previous value
  m_is_forward_shift = is_forward_shift ? is_forward_shift.value() : m_is_forward_shift;

  // path smoothing
  // 如果启用了 enable_path_smoothing 且点数足够，对 x, y, yaw, vx 分别进行移动平均滤波 (Moving
  // Average Filter)。
  MPCTrajectory mpc_traj_smoothed = mpc_traj_resampled;  // smooth filtered trajectory
  const int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (
    param.enable_path_smoothing && mpc_traj_resampled_size > 2 * param.path_filter_moving_ave_num) {
    using MoveAverageFilter::filt_vector;
    if (
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.x) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.y) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.yaw) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.vx)) {
      RCLCPP_DEBUG(m_logger, "path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /*
   * Extend terminal points
   * Note: The current MPC does not properly take into account the attitude angle at the end of the
   * path. By extending the end of the path in the attitude direction, the MPC can consider the
   * attitude angle well, resulting in improved control performance. If the trajectory is
   * well-defined considering the end point attitude angle, this feature is not necessary.
   */

  // 背景：MPC 是一个有限 horizon 的控制器。如果预测时域超出了规划轨迹的末端，MPC
  // 就不知道末端之后的路径走向（曲率和航向）。
  // 做法：在轨迹末尾，沿着最后一个点的航向角方向，额外延伸几个点。
  // 效果：让 MPC
  // “看到”终点之后的一段直路（或延续方向），避免车辆在接近轨迹终点时因为缺乏未来信息而提前减速或产生不必要的转向调整。
  if (param.extend_trajectory_for_end_yaw_control) {
    MPCUtils::extendTrajectoryInYawDirection(
      mpc_traj_raw.yaw.back(), param.traj_resample_dist, m_is_forward_shift, mpc_traj_smoothed);
  }

  // calculate yaw angle
  // 重算 Yaw：经过平滑和扩展后，原有的 yaw 值可能不再准确或与 x,y 坐标不匹配。这里根据 x, y
  // 坐标差分重新计算航向角。 单调化：convertEulerAngleToMonotonic 确保 yaw
  // 角是连续变化的（例如从 3.14 变到 -3.14 时，处理后变为 3.14
  // 变到 3.14+epsilon，而不是跳变）。这对于计算航向误差和微分至关重要。
  MPCUtils::calcTrajectoryYawFromXY(mpc_traj_smoothed, m_is_forward_shift);
  MPCUtils::convertEulerAngleToMonotonic(mpc_traj_smoothed.yaw);

  // calculate curvature
  // 计算轨迹上每个点的曲率 k 和平滑曲率 smooth_k
  // k：用于动力学模型预测。
  // smooth_k：主要用于计算前馈控制量（Feed-forward steering angle），即 $\delta_{ff} = L \cdot
  // k$。平滑后的曲率能提供更稳定的前馈指令。
  MPCUtils::calcTrajectoryCurvature(
    param.curvature_smoothing_num_traj, param.curvature_smoothing_num_ref_steer, mpc_traj_smoothed);

  // stop velocity at a terminal point
  mpc_traj_smoothed.vx.back() = 0.0;

  // add a extra point on back with extended time to make the mpc stable.
  // 末端速度：强制将轨迹最后一个有效点的速度设为 0，确保车辆能在终点停下。
  // 时间扩展技巧：
  // 复制最后一个点，将其 relative_time 增加一个很大的值（100秒）。
  // 目的：MPC 在进行时间插值或计算预测时域时，需要保证预测 horizon
  // 内的时间点都在轨迹的时间范围内。如果轨迹太短，MPC
  // 可能会因为查询超出范围的时间点而报错。添加这个“虚拟”的远端点，相当于告诉
  // MPC：“在这个时间点之后，车辆一直保持在终点状态”，从而保证数值计算的稳定性，防止因轨迹过短导致的索引越界或插值错误。
  auto last_point = mpc_traj_smoothed.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  mpc_traj_smoothed.push_back(last_point);

  if (!mpc_traj_smoothed.size()) {
    RCLCPP_DEBUG(m_logger, "path callback: trajectory size is undesired.");
    return;
  }

  m_reference_trajectory = mpc_traj_smoothed;
}

void MPC::resetPrevResult(const SteeringReport & current_steer)
{
  // Consider limit. The prev value larger than limitation breaks the optimization constraint and
  // results in optimization failure.
  const float steer_lim_f = static_cast<float>(m_steer_lim);
  m_raw_steer_cmd_prev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
  m_raw_steer_cmd_pprev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
}

std::pair<ResultWithReason, MPCData> MPC::getData(
  const MPCTrajectory & traj, const SteeringReport & current_steer,
  const Odometry & current_kinematics)
{
  // 从里程计消息 (Odometry) 中提取自车当前的位姿（位置 position 和朝向 orientation）。
  const auto current_pose = current_kinematics.pose.pose;

  MPCData data;
  // 输入：参考轨迹 traj、当前位姿、距离阈值、角度阈值。
  // 输出：
  // data.nearest_pose：轨迹上距离自车最近的点的位姿（经过插值，比离散点更精确）。
  // data.nearest_idx：该最近点在轨迹数组中的索引。
  // data.nearest_time：该最近点对应的时间戳（相对于轨迹起点的时间）。
  // 错误处理：如果找不到合法的最近点（例如车偏离轨迹太远，超过了 ego_nearest_dist_threshold），则返回失败。这是防止在极端偏离情况下 MPC 产生错误指令的安全机制。
  if (!MPCUtils::calcNearestPoseInterp(
        traj, current_pose, &(data.nearest_pose), &(data.nearest_idx), &(data.nearest_time),
        ego_nearest_dist_threshold, ego_nearest_yaw_threshold)) {
    return {ResultWithReason{false, "error in calculating nearest pose"}, MPCData{}};
  }

  // get data
  data.steer = static_cast<double>(current_steer.steering_tire_angle); //当前的实际转向轮角度。
  data.lateral_err = MPCUtils::calcLateralError(current_pose, data.nearest_pose); //计算自车当前位置到最近参考点的垂直距离
  data.yaw_err = normalize_radian(
    tf2::getYaw(current_pose.orientation) - tf2::getYaw(data.nearest_pose.orientation)); //计算自车当前朝向与轨迹最近点朝向之间的差值。

  // get predicted steer
  // 背景：转向执行机构存在物理延迟（例如液压或电动助力转向系统的响应时间）。当你发出指令时，车轮不会瞬间到达目标角度。
  // 功能：调用 SteeringPredictor 估算在控制指令真正生效时（即经过 input_delay 后），车轮实际会达到的角度。
  data.predicted_steer = m_steering_predictor->calcSteerPrediction();

  // check trajectory time length
  // 估算每个步长的平均时间（总长度/步数），用于粗略估计所需时间跨度
  const double max_prediction_time =
    m_param.min_prediction_length / static_cast<double>(m_param.prediction_horizon - 1);
    // MPC 需要查询的未来最远时间点。
  auto end_time = data.nearest_time + m_param.input_delay + m_ctrl_period + max_prediction_time;
  if (end_time > traj.relative_time.back()) {
    // 如果需要的 end_time 超过了轨迹最后一个点的时间 traj.relative_time.back()，说明轨迹太短了
    return {ResultWithReason{false, "path is too short for prediction."}, MPCData{}};
  }
  return {ResultWithReason{true}, data};
}

std::pair<ResultWithReason, MPCTrajectory> MPC::resampleMPCTrajectoryByTime(
  const double ts, const double prediction_dt, const MPCTrajectory & input) const
{
  MPCTrajectory output;
  std::vector<double> mpc_time_v;
  for (double i = 0; i < static_cast<double>(m_param.prediction_horizon); ++i) {
    mpc_time_v.push_back(ts + i * prediction_dt);
  }
  if (!MPCUtils::linearInterpMPCTrajectory(input.relative_time, input, mpc_time_v, output)) {
    return {ResultWithReason{false, "mpc resample error"}, {}};
  }
  // Publish resampled reference trajectory for debug purpose.
  if (m_publish_debug_trajectories) {
    auto converted_output = MPCUtils::convertToAutowareTrajectory(output);
    converted_output.header.stamp = m_clock->now();
    converted_output.header.frame_id = "map";
    m_debug_resampled_reference_trajectory_pub->publish(converted_output);
  }
  return {ResultWithReason{true}, output};
}

VectorXd MPC::getInitialState(const MPCData & data)
{
  // 在模型预测控制（MPC）中，我们需要知道“当前”系统的状态，才能预测未来的行为。由于不同的车辆模型（运动学 vs 动力学）对状态的定义不同，这个函数根据所选的模型类型，从误差数据中提取并组装相应的状态变量。
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  VectorXd x0 = VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err; //横向误差（车辆中心到参考线的垂直距离）。
  const auto & steer = m_use_steer_prediction ? data.predicted_steer : data.steer; //预测转向角或当前的转向角。
  const auto & yaw_err = data.yaw_err; // 航向角误差（车辆朝向与参考线切线方向的夹角）。

  const auto vehicle_model = m_vehicle_model_ptr->modelName();
  if (vehicle_model == "kinematics") {
    // 解释：
    //   $e_y$：横向偏差。
    //   $e_\psi$：航向偏差。
    //   $\delta$：前轮转角。
    //   适用性：这是最常用的模型，它显式地将转向角作为状态变量，从而可以在模型内部处理转向动力学（如一阶延迟）。
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model == "dynamics") {
    // 背景：动力学模型不仅关心位置和角度，还关心它们的变化率（速度），因为惯性力和轮胎侧偏力与速度密切相关。
    // 微分计算：
    // dlat ($\dot{e}_y$)：横向误差的变化率。通过当前误差与上一帧误差 m_lateral_error_prev 的差分除以控制周期 m_ctrl_period 得到。
    // dyaw ($\dot{e}_\psi$)：航向误差的变化率（即横摆角速度误差）。同样通过差分计算。
    // 低通滤波 (LPF)：
    // m_lpf_lateral_error.filter(dlat) 和 m_lpf_yaw_error.filter(dyaw)。
    // 原因：数值微分（差分）会放大传感器噪声。如果直接使用原始差分结果，状态向量中会包含大量高频噪声，导致 MPC 求解器不稳定或产生抖动的控制指令。低通滤波器用于平滑这些导数信号。
    // 更新历史值：保存当前的误差值，供下一帧计算微分使用。
    // 组装：将位置误差、平滑后的速度误差、角度误差、平滑后的角速度误差组装成 4 维向量。
    double dlat = (lat_err - m_lateral_error_prev) / m_ctrl_period;
    double dyaw = (yaw_err - m_yaw_error_prev) / m_ctrl_period;
    m_lateral_error_prev = lat_err;
    m_yaw_error_prev = yaw_err;
    dlat = m_lpf_lateral_error.filter(dlat);
    dyaw = m_lpf_yaw_error.filter(dyaw);
    x0 << lat_err, dlat, yaw_err, dyaw;
    RCLCPP_DEBUG(m_logger, "(before lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
    RCLCPP_DEBUG(m_logger, "(after lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
  } else {
    RCLCPP_ERROR(m_logger, "vehicle_model_type is undefined");
  }
  return x0;
}

std::pair<bool, VectorXd> MPC::updateStateForDelayCompensation(
  const MPCTrajectory & traj, const double & start_time, const VectorXd & x0_orig)
{
  // 补偿控制系统的输入延迟。
  // 在自动驾驶中，从控制器计算出转向指令，到指令发送给底盘，再到底盘执行机构（如电机或液压泵）真正转动方向盘，存在一定的时间滞后（input_delay）。如果 MPC 仅基于“当前时刻”的状态进行优化，那么当指令真正生效时，车辆的状态已经发生了变化，导致控制效果变差甚至不稳定。

  // 状态向量维度 (DIM_X)、控制输入维度 (DIM_U) 和输出观测维度 (DIM_Y)。3 1 2
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  // 功能：声明离散状态空间方程所需的矩阵：
  // $A_d$：状态转移矩阵。
  // $B_d$：控制输入矩阵。
  // $W_d$：常数项/扰动矩阵（通常包含曲率引起的项）。
  // $C_d$：观测矩阵（在此函数中未直接使用，但为了接口统一通常一起计算）。
  // 背景：车辆模型通常是连续时间的微分方程 $\dot{x} = Ax + Bu + W$。为了在计算机中进行离散时间步长的推演，需要将其离散化为 $x_{k+1} = A_d x_k + B_d u_k + W_d$。
  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);

  // 功能：判断车辆是前进还是后退。
  // 原因：在自行车模型中，曲率 $k$ 对状态演化的影响与速度方向有关。如果车辆后退，相同的转向角产生的横摆角速度方向相反。因此，在插值获取曲率后，需要根据行驶方向调整符号。
  const double sign_vx = m_is_forward_shift ? 1 : -1;

  MatrixXd x_curr = x0_orig;  //初始化为传入的原始初始状态 $x_0$（即当前时刻 $t_0$ 的状态）。它将随着循环逐步更新，最终变成 $t_0 + delay$ 时刻的状态。
  double mpc_curr_time = start_time; //nearest_time
  // //m_input_buffer 保存了过去几个控制周期内发出的转向指令。每一个缓冲区元素代表一个时间步长内的控制输入。通过逐个应用这些历史输入，我们可以模拟车辆在过去这段时间内的运动，从而推算出“现在”（即延迟结束后）的状态。
  for (size_t i = 0; i < m_input_buffer.size(); ++i) {  //m_input_buffer永远只有1个元素，即当前时刻的输入
    double k, v = 0.0;
    try {
      // NOTE: When driving backward, the curvature's sign should be reversed.
      // 根据当前推演时间 mpc_curr_time，从参考轨迹 traj.relative_time 中线性插值出对应的曲率 ($k$) 和速度 ($v$)。
      k = autoware::interpolation::lerp(traj.relative_time, traj.k, mpc_curr_time) * sign_vx;
      v = autoware::interpolation::lerp(traj.relative_time, traj.vx, mpc_curr_time);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(m_logger, "mpc resample failed at delay compensation, stop mpc: %s", e.what());
      return {false, {}};
    }

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(v);
    m_vehicle_model_ptr->setCurvature(k);
    // 基于当前的 $v, k$ 和控制周期 m_ctrl_period，计算出该步长下的 $A_d, B_d, W_d$。
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, m_ctrl_period);
    MatrixXd ud = MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = m_input_buffer.at(i);  // for steering input delay
    // 从 m_input_buffer 中取出第 $i$ 个历史转向指令。
    // 意义：这代表了在该时间步长内，实际作用于车辆的转向角（假设执行器完全跟随了指令，或者缓冲区已经包含了执行器模型的预测结果）。
    x_curr = Ad * x_curr + Bd * ud + Wd;
    // 功能：利用离散状态空间方程，将状态从当前时刻推演到下一个控制周期时刻。
    // 时间更新：将 mpc_curr_time 增加一个控制周期，为下一次循环的插值做准备。
    // 循环效应：经过 m_input_buffer.size() 次循环后，x_curr 就从 $t_0$ 时刻的状态变成了 $t_0 + input_delay$ 时刻的状态。
    mpc_curr_time += m_ctrl_period;
  }
  return {true, x_curr};
}

MPCTrajectory MPC::applyVelocityDynamicsFilter(
  const MPCTrajectory & input, const Odometry & current_kinematics) const
{
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);
  if (autoware_traj.points.empty()) {
    return input;
  }

  // 在轨迹中找到距离自车当前位姿最近的线段索引。
  const size_t nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);

  MPCTrajectory output = input;

  // 输入参数：
  // nearest_seg_idx：起始平滑的位置。
  // current_kinematics.twist.twist.linear.x：车辆当前的实际纵向速度。
  // m_param.acceleration_limit：车辆允许的最大加速度/减速度。
  // m_param.velocity_time_constant：一阶滞后系统的时间常数，模拟动力系统（发动机/电机）的响应延迟。
  // output：待修改的轨迹引用。
  // 算法逻辑（通常在 mpc_utils.cpp 中实现）：
  // 从当前实际速度开始。
  // 遍历轨迹后续的点。
  // 对于每个点，计算在最大加速度限制下，从上一帧速度能达到的“最大可达速度”。
  // 同时，应用一阶滞后滤波：$v_{new} = v_{old} + (v_{target} - v_{old}) \cdot (1 - e^{-\Delta t /
  // \tau})$。 取 min(规划速度, 最大可达速度) 作为该点的新速度。
  // 意义：规划模块给出的速度可能是阶跃变化的（例如突然从 0 变到
  // 10m/s），这在物理上是不可能的。此步骤生成的速度曲线是平滑且物理可达的，使得 MPC
  // 基于此速度计算的离心力、前馈转角等更加准确。

  MPCUtils::dynamicSmoothingVelocity(
    nearest_seg_idx, current_kinematics.twist.twist.linear.x, m_param.acceleration_limit,
    m_param.velocity_time_constant, output);

  // “在轨迹正式结束后的很长一段时间内，车辆都保持在终点位置且速度为 0”。这保证了即使 MPC
  // 的预测时域很长，也不会因为查询超出范围的时间点而导致计算失败。
  auto last_point = output.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  output.push_back(last_point);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCMatrix MPC::generateMPCMatrix(
  const MPCTrajectory & reference_trajectory, const double prediction_dt)
{
  const int N = m_param.prediction_horizon; //预测步数
  const double DT = prediction_dt; //时间步长 
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  //把每一次的模型运动矩阵保存在 m 中
  MPCMatrix m; 
  m.Aex = MatrixXd::Zero(DIM_X * N, DIM_X);
  m.Bex = MatrixXd::Zero(DIM_X * N, DIM_U * N);
  m.Wex = MatrixXd::Zero(DIM_X * N, 1);
  m.Cex = MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  m.Qex = MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  m.R1ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.R2ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.Uref_ex = MatrixXd::Zero(DIM_U * N, 1);

  // weight matrix depends on the vehicle model
  MatrixXd Q = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R = MatrixXd::Zero(DIM_U, DIM_U);
  MatrixXd Q_adaptive = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R_adaptive = MatrixXd::Zero(DIM_U, DIM_U);

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);
  MatrixXd Uref(DIM_U, 1);

  const double sign_vx = m_is_forward_shift ? 1 : -1;

  // predict dynamics for N times
  for (int i = 0; i < N; ++i) {
    // 取出提取计算好的第 $i$ 步参考轨迹上的速度 ref_vx 和曲率 ref_k。
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_vx_squared = ref_vx * ref_vx;

    // NOTE: When driving backward, the curvature's sign should be reversed.
    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double ref_smooth_k = reference_trajectory.smooth_k.at(i) * sign_vx;

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(ref_vx);
    m_vehicle_model_ptr->setCurvature(ref_k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = MatrixXd::Zero(DIM_Y, DIM_Y);
    R = MatrixXd::Zero(DIM_U, DIM_U);
    // 基础权重：从配置中获取横向误差、航向误差、转向输入的基准权重。
    const auto mpc_weight = getWeight(ref_k);
    Q(0, 0) = mpc_weight.lat_error;
    Q(1, 1) = mpc_weight.heading_error;
    R(0, 0) = mpc_weight.steering_input;

    Q_adaptive = Q;
    R_adaptive = R;
    // 如果是最后一步 ($i=N-1$)，使用更大的终端权重 (terminal_...)，迫使 MPC 在预测末端尽可能消除误差，保证稳定性。
    if (i == N - 1) {
      Q_adaptive(0, 0) = m_param.nominal_weight.terminal_lat_error;
      Q_adaptive(1, 1) = m_param.nominal_weight.terminal_heading_error;
    }
    // heading_error_squared_vel: 速度越高，航向误差的惩罚越大（高速下航向偏差更危险）。
    // steering_input_squared_vel: 速度越高，转向输入的惩罚越大（高速下剧烈转向不稳定）。
    Q_adaptive(1, 1) += ref_vx_squared * mpc_weight.heading_error_squared_vel;
    R_adaptive(0, 0) += ref_vx_squared * mpc_weight.steering_input_squared_vel;

    // update mpc matrix
    int idx_x_i = i * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      m.Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      m.Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      m.Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      int idx_x_i_prev = (i - 1) * DIM_X;
      m.Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * m.Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        m.Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * m.Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      m.Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * m.Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    m.Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    m.Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    m.Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    m.R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    // get reference input (feed-forward)
    m_vehicle_model_ptr->setCurvature(ref_smooth_k);
    m_vehicle_model_ptr->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < autoware_utils::deg2rad(m_param.zero_ff_steer_deg)) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Uref_ex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  // add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double j = ref_vx * ref_vx * getWeight(ref_k).lat_jerk / (DT * DT);
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(prediction_dt, m.R1ex);

  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
std::pair<ResultWithReason, VectorXd> MPC::executeOptimization(
  const MPCMatrix & m, const VectorXd & x0, const double prediction_dt, const MPCTrajectory & traj,
  const double current_velocity)
{
  VectorXd Uex;

  if (!isValid(m)) {
    return {ResultWithReason{false, "invalid model matrix"}, {}};
  }

  const int DIM_U_N = m_param.prediction_horizon * m_vehicle_model_ptr->getDimU(); //预测步数 × 每个步长的控制输入维度

  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const MatrixXd CB = m.Cex * m.Bex;
  const MatrixXd QCB = m.Qex * CB;
  // MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for
  // a good way.  //NOLINT
  MatrixXd H = MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  MatrixXd f = (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Uref_ex.transpose() * m.R1ex;
  addSteerWeightF(prediction_dt, f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  // steering angle limit
  VectorXd lb = VectorXd::Constant(DIM_U_N, -m_steer_lim);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, m_steer_lim);   // max steering angle

  // steering angle rate limit
  VectorXd steer_rate_limits = calcSteerRateLimitOnTrajectory(traj, current_velocity);
  VectorXd ubA = steer_rate_limits * prediction_dt;
  VectorXd lbA = -steer_rate_limits * prediction_dt;
  ubA(0) = m_raw_steer_cmd_prev + steer_rate_limits(0) * m_ctrl_period;
  lbA(0) = m_raw_steer_cmd_prev - steer_rate_limits(0) * m_ctrl_period;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = m_qpsolver_ptr->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    return {ResultWithReason{false, "qp solver error"}, {}};
  }

  {
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    RCLCPP_DEBUG(m_logger, "qp solver calculation time = %ld [ms]", t);
  }

  if (Uex.array().isNaN().any()) {
    return {ResultWithReason{false, "model Uex including NaN"}, {}};
  }
  return {ResultWithReason{true}, Uex};
}

void MPC::addSteerWeightR(const double prediction_dt, MatrixXd & R) const
{
  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;

  // add steering rate : weight for (u(i) - u(i-1) / dt )^2
  {
    const double steer_rate_r = m_param.nominal_weight.steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += m_param.nominal_weight.steer_rate / (m_ctrl_period * m_ctrl_period);
    }
  }

  // add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2
  {
    const double w = m_param.nominal_weight.steer_acc;
    const double steer_acc_r = w / std::pow(DT, 4);
    const double steer_acc_r_cp1 = w / (std::pow(DT, 3) * m_ctrl_period);
    const double steer_acc_r_cp2 = w / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
    const double steer_acc_r_cp4 = w / std::pow(m_ctrl_period, 4);
    const Eigen::Matrix3d D =
      steer_acc_r *
      (Eigen::Matrix3d() << 1.0, -2.0, 1.0, -2.0, 4.0, -2.0, 1.0, -2.0, 1.0).finished();
    for (int i = 1; i < N - 1; ++i) {
      R.block(i - 1, i - 1, 3, 3) += D;
    }
    if (N > 1) {
      // steer acc i = 1
      R(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
      R(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(1, 1) += steer_acc_r * 1.0;
      // steer acc i = 0
      R(0, 0) += steer_acc_r_cp4 * 1.0;
    }
  }
}

void MPC::addSteerWeightF(const double prediction_dt, MatrixXd & f) const
{
  if (f.rows() < 2) {
    return;
  }

  const double DT = prediction_dt;

  // steer rate for i = 0
  f(0, 0) += -2.0 * m_param.nominal_weight.steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = m_param.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 3) * m_ctrl_period);
  const double steer_acc_r_cp2 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
  const double steer_acc_r_cp4 = m_param.nominal_weight.steer_acc / std::pow(m_ctrl_period, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * m_raw_steer_cmd_prev + m_raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * m_raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * m_raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

double MPC::getPredictionDeltaTime(
  const double start_time, const MPCTrajectory & input, const Odometry & current_kinematics) const
{
  // Calculate the time min_prediction_length ahead from current_pose
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);
  const size_t nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  double sum_dist = 0;
  const double target_time = [&]() {
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    for (size_t i = nearest_idx + 1; i < input.relative_time.size(); i++) {
      const double segment_dist = MPCUtils::calcDistance2d(input, i, i - 1);
      sum_dist += segment_dist;
      if (m_param.min_prediction_length < sum_dist) {
        const double prev_sum_dist = sum_dist - segment_dist;
        const double ratio = (m_param.min_prediction_length - prev_sum_dist) / segment_dist;
        const double relative_time_at_i = i == input.relative_time.size() - 1
                                            ? input.relative_time.at(i) - t_ext
                                            : input.relative_time.at(i);
        return input.relative_time.at(i - 1) +
               (relative_time_at_i - input.relative_time.at(i - 1)) * ratio;
      }
    }
    return input.relative_time.back() - t_ext;
  }();

  // Calculate delta time for min_prediction_length
  const double dt =
    (target_time - start_time) / static_cast<double>(m_param.prediction_horizon - 1);

  return std::max(dt, m_param.prediction_dt);
}

// 计算转向角速度（Steering Rotation Rate），即方向盘转动的快慢。
// 在自动驾驶控制中，底盘控制器通常不仅需要知道“转到多少度”（steering_tire_angle），还需要知道“以多快的速度转过去”（steering_tire_rotation_rate），以便电机能够平滑、快速地执行指令，减少滞后。
double MPC::calcDesiredSteeringRate(
  const MPCMatrix & mpc_matrix, const MatrixXd & x0, const MatrixXd & Uex, const double u_filtered,
  const float current_steer, const double predict_dt) const
{
  if (m_vehicle_model_ptr->modelName() != "kinematics") {
    // not supported yet. Use old implementation.
    return (u_filtered - current_steer) / predict_dt;
  }

  // calculate predicted states to get the steering motion
  const auto & m = mpc_matrix;
  const MatrixXd Xex = m.Aex * x0 + m.Bex * Uex + m.Wex;  //这里是所有的预测状态

  const size_t STEER_IDX = 2;  // for kinematics model

  const auto steer_0 = x0(STEER_IDX, 0); //获取当前状态的转向角度
  const auto steer_1 = Xex(STEER_IDX, 0); //从预测状态序列 $X_{ex}$ 中提取第一个预测步长后的转向角。

  const auto steer_rate = (steer_1 - steer_0) / predict_dt;

  return steer_rate;
}

VectorXd MPC::calcSteerRateLimitOnTrajectory(
  const MPCTrajectory & trajectory, const double current_velocity) const
{
  const auto interp = [&](const auto & steer_rate_limit_map, const auto & current) {
    std::vector<double> reference, limits;
    for (const auto & p : steer_rate_limit_map) {
      reference.push_back(p.first);
      limits.push_back(p.second);
    }

    // If the speed is out of range of the reference, apply zero-order hold.
    if (current <= reference.front()) {
      return limits.front();
    }
    if (current >= reference.back()) {
      return limits.back();
    }

    // Apply linear interpolation
    for (size_t i = 0; i < reference.size() - 1; ++i) {
      if (reference.at(i) <= current && current <= reference.at(i + 1)) {
        auto ratio =
          (current - reference.at(i)) / std::max(reference.at(i + 1) - reference.at(i), 1.0e-5);
        ratio = std::clamp(ratio, 0.0, 1.0);
        const auto interp = limits.at(i) + ratio * (limits.at(i + 1) - limits.at(i));
        return interp;
      }
    }

    std::cerr << "MPC::calcSteerRateLimitOnTrajectory() interpolation logic is broken. Command "
                 "filter is not working. Please check the code."
              << std::endl;
    return reference.back();
  };

  // When the vehicle is stopped, a large steer rate limit is used for the dry steering.
  constexpr double steer_rate_lim = 100.0;
  const bool is_vehicle_stopped = std::fabs(current_velocity) < 0.01;
  if (is_vehicle_stopped) {
    return steer_rate_lim * VectorXd::Ones(m_param.prediction_horizon);
  }

  // calculate steering rate limit
  VectorXd steer_rate_limits = VectorXd::Zero(m_param.prediction_horizon);
  for (int i = 0; i < m_param.prediction_horizon; ++i) {
    const auto limit_by_curvature = interp(m_steer_rate_lim_map_by_curvature, trajectory.k.at(i));
    const auto limit_by_velocity = interp(m_steer_rate_lim_map_by_velocity, trajectory.vx.at(i));
    steer_rate_limits(i) = std::min(limit_by_curvature, limit_by_velocity);
  }

  return steer_rate_limits;
}

Trajectory MPC::calculatePredictedTrajectory(
  const MPCMatrix & mpc_matrix, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
  const MPCTrajectory & reference_trajectory, const double dt, const std::string & coordinate) const
{
  MPCTrajectory predicted_mpc_trajectory;

  if (coordinate == "world") {
    predicted_mpc_trajectory = m_vehicle_model_ptr->calculatePredictedTrajectoryInWorldCoordinate(
      mpc_matrix.Aex, mpc_matrix.Bex, mpc_matrix.Cex, mpc_matrix.Wex, x0, Uex, reference_trajectory,
      dt);
  } else if (coordinate == "frenet") {
    predicted_mpc_trajectory = m_vehicle_model_ptr->calculatePredictedTrajectoryInFrenetCoordinate(
      mpc_matrix.Aex, mpc_matrix.Bex, mpc_matrix.Cex, mpc_matrix.Wex, x0, Uex, reference_trajectory,
      dt);
  } else {
    throw std::invalid_argument("Invalid coordinate system specified. Use 'world' or 'frenet'.");
  }

  // do not over the reference trajectory
  const auto predicted_length = MPCUtils::calcMPCTrajectoryArcLength(reference_trajectory);
  const auto clipped_trajectory =
    MPCUtils::clipTrajectoryByLength(predicted_mpc_trajectory, predicted_length);

  const auto predicted_trajectory = MPCUtils::convertToAutowareTrajectory(clipped_trajectory);

  return predicted_trajectory;
}

bool MPC::isValid(const MPCMatrix & m) const
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Uref_ex.array().isNaN().any()) {
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Uref_ex.array().isInf().any()) {
    return false;
  }

  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
