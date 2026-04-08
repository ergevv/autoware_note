// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
PidLongitudinalController::PidLongitudinalController(
  rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater)
: node_parameters_(node.get_node_parameters_interface()),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("longitudinal_controller"))
{
  using std::placeholders::_1;

  diag_updater_ = diag_updater;

  // parameters timer
  m_longitudinal_ctrl_period = node.get_parameter("ctrl_period").as_double();

  m_wheel_base = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().wheel_base_m;
  m_vehicle_width =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().vehicle_width_m;
  m_front_overhang =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().front_overhang_m;

  // parameters for delay compensation
  m_delay_compensation_time = node.declare_parameter<double>("delay_compensation_time");  // [s]

  // parameters to enable functions
  m_enable_smooth_stop = node.declare_parameter<bool>("enable_smooth_stop");
  m_enable_overshoot_emergency = node.declare_parameter<bool>("enable_overshoot_emergency");
  m_enable_large_tracking_error_emergency =
    node.declare_parameter<bool>("enable_large_tracking_error_emergency");
  m_enable_slope_compensation = node.declare_parameter<bool>("enable_slope_compensation");
  m_enable_keep_stopped_until_steer_convergence =
    node.declare_parameter<bool>("enable_keep_stopped_until_steer_convergence");

  // parameters for state transition
  {
    auto & p = m_state_transition_params;
    // drive
    p.drive_state_stop_dist = node.declare_parameter<double>("drive_state_stop_dist");  // [m]
    p.drive_state_offset_stop_dist =
      node.declare_parameter<double>("drive_state_offset_stop_dist");  // [m]
    // stopping
    p.stopping_state_stop_dist = node.declare_parameter<double>("stopping_state_stop_dist");  // [m]
    p.stopped_state_entry_duration_time =
      node.declare_parameter<double>("stopped_state_entry_duration_time");  // [s]
    // stop
    p.stopped_state_entry_vel = node.declare_parameter<double>("stopped_state_entry_vel");  // [m/s]
    p.stopped_state_entry_acc =
      node.declare_parameter<double>("stopped_state_entry_acc");  // [m/s²]

    // emergency
    p.emergency_state_overshoot_stop_dist =
      node.declare_parameter<double>("emergency_state_overshoot_stop_dist");  // [m]
  }

  // parameters for drive state
  {
    // initialize PID gain
    const double kp{node.declare_parameter<double>("kp")};
    const double ki{node.declare_parameter<double>("ki")};
    const double kd{node.declare_parameter<double>("kd")};
    m_pid_vel.setGains(kp, ki, kd);

    // initialize PID limits
    const double max_pid{node.declare_parameter<double>("max_out")};     // [m/s^2]
    const double min_pid{node.declare_parameter<double>("min_out")};     // [m/s^2]
    const double max_p{node.declare_parameter<double>("max_p_effort")};  // [m/s^2]
    const double min_p{node.declare_parameter<double>("min_p_effort")};  // [m/s^2]
    const double max_i{node.declare_parameter<double>("max_i_effort")};  // [m/s^2]
    const double min_i{node.declare_parameter<double>("min_i_effort")};  // [m/s^2]
    const double max_d{node.declare_parameter<double>("max_d_effort")};  // [m/s^2]
    const double min_d{node.declare_parameter<double>("min_d_effort")};  // [m/s^2]
    m_pid_vel.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

    // set lowpass filter for vel error and pitch
    const double lpf_vel_error_gain{node.declare_parameter<double>("lpf_vel_error_gain")};
    m_lpf_vel_error = std::make_shared<LowpassFilter1d>(0.0, lpf_vel_error_gain);

    m_enable_integration_at_low_speed =
      node.declare_parameter<bool>("enable_integration_at_low_speed");
    m_current_vel_threshold_pid_integrate =
      node.declare_parameter<double>("current_vel_threshold_pid_integration");  // [m/s]

    m_time_threshold_before_pid_integrate =
      node.declare_parameter<double>("time_threshold_before_pid_integration");  // [s]

    m_enable_brake_keeping_before_stop =
      node.declare_parameter<bool>("enable_brake_keeping_before_stop");         // [-]
    m_brake_keeping_acc = node.declare_parameter<double>("brake_keeping_acc");  // [m/s^2]
  }

  // parameters for smooth stop state
  {
    const double max_strong_acc{
      node.declare_parameter<double>("smooth_stop_max_strong_acc")};  // [m/s^2]
    const double min_strong_acc{
      node.declare_parameter<double>("smooth_stop_min_strong_acc")};                // [m/s^2]
    const double weak_acc{node.declare_parameter<double>("smooth_stop_weak_acc")};  // [m/s^2]
    const double weak_stop_acc{
      node.declare_parameter<double>("smooth_stop_weak_stop_acc")};  // [m/s^2]
    const double strong_stop_acc{
      node.declare_parameter<double>("smooth_stop_strong_stop_acc")};  // [m/s^2]

    const double max_fast_vel{node.declare_parameter<double>("smooth_stop_max_fast_vel")};  // [m/s]
    const double min_running_vel{
      node.declare_parameter<double>("smooth_stop_min_running_vel")};  // [m/s]
    const double min_running_acc{
      node.declare_parameter<double>("smooth_stop_min_running_acc")};  // [m/s^2]
    const double weak_stop_time{
      node.declare_parameter<double>("smooth_stop_weak_stop_time")};  // [s]

    const double weak_stop_dist{
      node.declare_parameter<double>("smooth_stop_weak_stop_dist")};  // [m]
    const double strong_stop_dist{
      node.declare_parameter<double>("smooth_stop_strong_stop_dist")};  // [m]

    m_smooth_stop.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }

  // parameters for stop state
  {
    auto & p = m_stopped_state_params;
    p.vel = node.declare_parameter<double>("stopped_vel");  // [m/s]
    p.acc = node.declare_parameter<double>("stopped_acc");  // [m/s^2]
  }

  // parameters for emergency state
  {
    auto & p = m_emergency_state_params;
    p.vel = node.declare_parameter<double>("emergency_vel");    // [m/s]
    p.acc = node.declare_parameter<double>("emergency_acc");    // [m/s^2]
    p.jerk = node.declare_parameter<double>("emergency_jerk");  // [m/s^3]
  }

  // parameters for acc feedback
  {
    const double lpf_acc_error_gain{node.declare_parameter<double>("lpf_acc_error_gain")};
    m_lpf_acc_error = std::make_shared<LowpassFilter1d>(0.0, lpf_acc_error_gain);
    m_acc_feedback_gain = node.declare_parameter<double>("acc_feedback_gain");
  }

  // parameters for acceleration limit
  m_max_acc = node.declare_parameter<double>("max_acc");  // [m/s^2]
  m_min_acc = node.declare_parameter<double>("min_acc");  // [m/s^2]

  // parameters for jerk limit
  m_max_jerk = node.declare_parameter<double>("max_jerk");                  // [m/s^3]
  m_min_jerk = node.declare_parameter<double>("min_jerk");                  // [m/s^3]
  m_max_acc_cmd_diff = node.declare_parameter<double>("max_acc_cmd_diff");  // [m/s^3]

  // parameters for slope compensation
  m_adaptive_trajectory_velocity_th =
    node.declare_parameter<double>("adaptive_trajectory_velocity_th");  // [m/s^2]
  const double lpf_pitch_gain{node.declare_parameter<double>("lpf_pitch_gain")};
  m_lpf_pitch = std::make_shared<LowpassFilter1d>(0.0, lpf_pitch_gain);
  m_max_pitch_rad = node.declare_parameter<double>("max_pitch_rad");  // [rad]
  m_min_pitch_rad = node.declare_parameter<double>("min_pitch_rad");  // [rad]

  // check slope source is proper
  const std::string slope_source = node.declare_parameter<std::string>(
    "slope_source");  // raw_pitch, trajectory_pitch or trajectory_adaptive
  if (slope_source == "raw_pitch") {
    m_slope_source = SlopeSource::RAW_PITCH;
  } else if (slope_source == "trajectory_pitch") {
    m_slope_source = SlopeSource::TRAJECTORY_PITCH;
  } else if (slope_source == "trajectory_adaptive") {
    m_slope_source = SlopeSource::TRAJECTORY_ADAPTIVE;
  } else if (slope_source == "trajectory_goal_adaptive") {
    m_slope_source = SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;
  } else {
    RCLCPP_ERROR(logger_, "Slope source is not valid. Using raw_pitch option as default");
    m_slope_source = SlopeSource::RAW_PITCH;
  }

  // ego nearest index search
  m_ego_nearest_dist_threshold =
    node.has_parameter("ego_nearest_dist_threshold")
      ? node.get_parameter("ego_nearest_dist_threshold").as_double()
      : node.declare_parameter<double>("ego_nearest_dist_threshold");  // [m]
  m_ego_nearest_yaw_threshold =
    node.has_parameter("ego_nearest_yaw_threshold")
      ? node.get_parameter("ego_nearest_yaw_threshold").as_double()
      : node.declare_parameter<double>("ego_nearest_yaw_threshold");  // [rad]

  // subscriber, publisher
  m_pub_slope = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/slope_angle", rclcpp::QoS{1});
  m_pub_debug = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/longitudinal_diagnostic", rclcpp::QoS{1});
  m_pub_virtual_wall_marker = node.create_publisher<MarkerArray>("~/virtual_wall", 1);

  // set parameter callback
  m_set_param_res = node.add_on_set_parameters_callback(
    std::bind(&PidLongitudinalController::paramCallback, this, _1));

  // diagnostic
  setupDiagnosticUpdater();
}

void PidLongitudinalController::setKinematicState(const nav_msgs::msg::Odometry & msg)
{
  m_current_kinematic_state = msg;
}

void PidLongitudinalController::setCurrentAcceleration(
  const geometry_msgs::msg::AccelWithCovarianceStamped & msg)
{
  m_current_accel = msg;
}

void PidLongitudinalController::setCurrentOperationMode(const OperationModeState & msg)
{
  m_current_operation_mode = msg;
}

void PidLongitudinalController::setTrajectory(const autoware_planning_msgs::msg::Trajectory & msg)
{
  if (!longitudinal_utils::isValidTrajectory(msg)) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 3000, "received invalid trajectory. ignore.");
    return;
  }

  if (msg.points.size() < 2) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000, "Unexpected trajectory size < 2. Ignored.");
    return;
  }

  m_trajectory = msg;
}

rcl_interfaces::msg::SetParametersResult PidLongitudinalController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, double & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_double();
      return true;
    }
    return false;
  };

  // delay compensation
  update_param("delay_compensation_time", m_delay_compensation_time);

  // state transition
  {
    auto & p = m_state_transition_params;
    update_param("drive_state_stop_dist", p.drive_state_stop_dist);
    update_param("stopping_state_stop_dist", p.stopping_state_stop_dist);
    update_param("stopped_state_entry_duration_time", p.stopped_state_entry_duration_time);
    update_param("stopped_state_entry_vel", p.stopped_state_entry_vel);
    update_param("stopped_state_entry_acc", p.stopped_state_entry_acc);
    update_param("emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist);
  }

  // drive state
  {
    double kp{node_parameters_->get_parameter("kp").as_double()};
    double ki{node_parameters_->get_parameter("ki").as_double()};
    double kd{node_parameters_->get_parameter("kd").as_double()};
    update_param("kp", kp);
    update_param("ki", ki);
    update_param("kd", kd);
    m_pid_vel.setGains(kp, ki, kd);

    double lpf_vel_error_gain{node_parameters_->get_parameter("lpf_vel_error_gain").as_double()};
    update_param("lpf_vel_error_gain", lpf_vel_error_gain);
    m_lpf_vel_error->setGain(lpf_vel_error_gain);

    double max_pid{node_parameters_->get_parameter("max_out").as_double()};
    double min_pid{node_parameters_->get_parameter("min_out").as_double()};
    double max_p{node_parameters_->get_parameter("max_p_effort").as_double()};
    double min_p{node_parameters_->get_parameter("min_p_effort").as_double()};
    double max_i{node_parameters_->get_parameter("max_i_effort").as_double()};
    double min_i{node_parameters_->get_parameter("min_i_effort").as_double()};
    double max_d{node_parameters_->get_parameter("max_d_effort").as_double()};
    double min_d{node_parameters_->get_parameter("min_d_effort").as_double()};
    update_param("max_out", max_pid);
    update_param("min_out", min_pid);
    update_param("max_p_effort", max_p);
    update_param("min_p_effort", min_p);
    update_param("max_i_effort", max_i);
    update_param("min_i_effort", min_i);
    update_param("max_d_effort", max_d);
    update_param("min_d_effort", min_d);
    m_pid_vel.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

    update_param("current_vel_threshold_pid_integration", m_current_vel_threshold_pid_integrate);
    update_param("time_threshold_before_pid_integration", m_time_threshold_before_pid_integrate);
  }

  // stopping state
  {
    double max_strong_acc{
      node_parameters_->get_parameter("smooth_stop_max_strong_acc").as_double()};
    double min_strong_acc{
      node_parameters_->get_parameter("smooth_stop_min_strong_acc").as_double()};
    double weak_acc{node_parameters_->get_parameter("smooth_stop_weak_acc").as_double()};
    double weak_stop_acc{node_parameters_->get_parameter("smooth_stop_weak_stop_acc").as_double()};
    double strong_stop_acc{
      node_parameters_->get_parameter("smooth_stop_strong_stop_acc").as_double()};
    double max_fast_vel{node_parameters_->get_parameter("smooth_stop_max_fast_vel").as_double()};
    double min_running_vel{
      node_parameters_->get_parameter("smooth_stop_min_running_vel").as_double()};
    double min_running_acc{
      node_parameters_->get_parameter("smooth_stop_min_running_acc").as_double()};
    double weak_stop_time{
      node_parameters_->get_parameter("smooth_stop_weak_stop_time").as_double()};
    double weak_stop_dist{
      node_parameters_->get_parameter("smooth_stop_weak_stop_dist").as_double()};
    double strong_stop_dist{
      node_parameters_->get_parameter("smooth_stop_strong_stop_dist").as_double()};
    update_param("smooth_stop_max_strong_acc", max_strong_acc);
    update_param("smooth_stop_min_strong_acc", min_strong_acc);
    update_param("smooth_stop_weak_acc", weak_acc);
    update_param("smooth_stop_weak_stop_acc", weak_stop_acc);
    update_param("smooth_stop_strong_stop_acc", strong_stop_acc);
    update_param("smooth_stop_max_fast_vel", max_fast_vel);
    update_param("smooth_stop_min_running_vel", min_running_vel);
    update_param("smooth_stop_min_running_acc", min_running_acc);
    update_param("smooth_stop_weak_stop_time", weak_stop_time);
    update_param("smooth_stop_weak_stop_dist", weak_stop_dist);
    update_param("smooth_stop_strong_stop_dist", strong_stop_dist);
    m_smooth_stop.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }

  // stop state
  {
    auto & p = m_stopped_state_params;
    update_param("stopped_vel", p.vel);
    update_param("stopped_acc", p.acc);
  }

  // emergency state
  {
    auto & p = m_emergency_state_params;
    update_param("emergency_vel", p.vel);
    update_param("emergency_acc", p.acc);
    update_param("emergency_jerk", p.jerk);
  }

  // acceleration feedback
  update_param("acc_feedback_gain", m_acc_feedback_gain);
  double lpf_acc_error_gain{node_parameters_->get_parameter("lpf_acc_error_gain").as_double()};
  update_param("lpf_acc_error_gain", lpf_acc_error_gain);
  m_lpf_acc_error->setGain(lpf_acc_error_gain);

  // acceleration limit
  update_param("min_acc", m_min_acc);

  // jerk limit
  update_param("max_jerk", m_max_jerk);
  update_param("min_jerk", m_min_jerk);
  update_param("max_acc_cmd_diff", m_max_acc_cmd_diff);

  // slope compensation
  update_param("max_pitch_rad", m_max_pitch_rad);
  update_param("min_pitch_rad", m_min_pitch_rad);
  update_param("adaptive_trajectory_velocity_th", m_adaptive_trajectory_velocity_th);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

bool PidLongitudinalController::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
}

trajectory_follower::LongitudinalOutput PidLongitudinalController::run(
  trajectory_follower::InputData const & input_data)
{
  // set input data
  setTrajectory(input_data.current_trajectory);
  setKinematicState(input_data.current_odometry);
  setCurrentAcceleration(input_data.current_accel);
  setCurrentOperationMode(input_data.current_operation_mode);

  // calculate current pose and control data
  geometry_msgs::msg::Pose current_pose = m_current_kinematic_state.pose.pose;

  // 定位：确定车在轨迹上的确切位置（插值）。
  // 预测：考虑系统延迟，预测控制执行时的车辆位置和状态。
  // 对齐：在处理轨迹数据（去重）后，重新校准索引以确保准确性。
  // 环境感知：计算道路坡度，并根据配置策略融合 IMU 数据和地图轨迹数据，为坡度补偿提供依据。
  // 状态准备：计算停车距离和档位，为状态机跳转做准备。
  const auto control_data = getControlData(current_pose);

  // update control state
  // 这个函数的主要作用是状态管理。它不直接计算加速度，而是根据：
  //   距离停车点的远近 (stop_dist)
  //   车辆是否真正静止 (is_stopped, stopped_condition)
  //   是否发生危险过冲 (emergency_condition)
  //   转向是否收敛 (is_steer_converged)
  //   是否在自动控制模式下 (is_under_control)
  // 来决定控制器下一步该执行什么策略（是继续 PID 跟踪、平滑减速、保持刹车还是紧急制动）。
  updateControlState(control_data);

  // calculate control command
  // 根据 m_control_state 选择不同的控制策略。
  //   STOPPED: 输出固定静止值。
  //   EMERGENCY: 输出紧急制动值。
  //   STOPPING: 使用平滑停止算法。
  //   DRIVE: 使用 PID 速度反馈 + 前馈。
  const Motion ctrl_cmd = calcCtrlCmd(control_data);

  // create control command
  // 使用速度和加速度做速度控制。
  const auto cmd_msg = createCtrlCmdMsg(ctrl_cmd, control_data.current_motion.vel);
  trajectory_follower::LongitudinalOutput output;
  output.control_cmd = cmd_msg;

  // create control command horizon
  output.control_cmd_horizon.controls.push_back(cmd_msg);
  output.control_cmd_horizon.time_step_ms = 0.0;

  // publish debug data
  publishDebugData(ctrl_cmd, control_data);

  // diagnostic
  diag_updater_->force_update();

  return output;
}

PidLongitudinalController::ControlData PidLongitudinalController::getControlData(
  const geometry_msgs::msg::Pose & current_pose)
{
  ControlData control_data{};

  // dt
  control_data.dt = getDt();  // 计算当前时间与上一次成功控制的时间间隔？

  // current velocity and acceleration
  control_data.current_motion.vel = m_current_kinematic_state.twist.twist.linear.x;
  control_data.current_motion.acc = m_current_accel.accel.accel.linear.x;
  control_data.interpolated_traj = m_trajectory;

  // calculate the interpolated point and segment
  const auto current_interpolated_pose = calcInterpolatedTrajPointAndSegment(
    control_data.interpolated_traj,
    current_pose);  // 根据车辆当前位姿 current_pose
                    // 在轨迹上找到最近的线段，并线性插值出一个精确位于车辆横向投影位置的轨迹点

  // Insert the interpolated point
  control_data.interpolated_traj.points.insert(
    control_data.interpolated_traj.points.begin() + current_interpolated_pose.second + 1,
    current_interpolated_pose
      .first);  // 将车辆当前位姿对应的插值点插入到轨迹中，插入位置为最近线段的后一个位置，即
                // current_interpolated_pose.second + 1
  control_data.nearest_idx = current_interpolated_pose.second + 1;
  control_data.target_idx = control_data.nearest_idx;
  const auto nearest_point = current_interpolated_pose.first;
  auto target_point = current_interpolated_pose.first;

  // Delay compensation - Calculate the distance we got, predicted velocity and predicted
  // acceleration after delay
  control_data.state_after_delay = predictedStateAfterDelay(
    control_data.current_motion,
    m_delay_compensation_time);  // 预测经过m_delay_compensation_time秒后车辆的状态

  // calculate the target motion for delay compensation
  constexpr double min_running_dist = 0.01;
  // 判断是否需要补偿：如果预测的行驶距离大于阈值（0.01米），则进行补偿。
  if (control_data.state_after_delay.running_distance > min_running_dist) {
    control_data.interpolated_traj.points = autoware::motion_utils::removeOverlapPoints(
      control_data.interpolated_traj.points);  // 在查找目标点之前，先移除轨迹中的重叠点
    // 从当前的 nearest_idx 开始，沿着轨迹向前寻找距离为 running_distance 的位置 target_pose。
    const auto target_pose = longitudinal_utils::findTrajectoryPoseAfterDistance(
      control_data.nearest_idx, control_data.state_after_delay.running_distance,
      control_data.interpolated_traj);
    // 在这个预测的目标位姿处，再次进行线性插值
    const auto target_interpolated_point =
      calcInterpolatedTrajPointAndSegment(control_data.interpolated_traj, target_pose);
    control_data.target_idx = target_interpolated_point.second + 1;
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + control_data.target_idx,
      target_interpolated_point.first);
    target_point = target_interpolated_point.first;
  }

  // ==========================================================================================
  // NOTE: due to removeOverlapPoints(), the obtained control_data.target_idx and
  // control_data.nearest_idx may become invalid if the number of points decreased.
  // current API does not provide the way to check duplication beforehand and this function
  // does not tell how many/which index points were removed, so there is no way
  // to tell if our `control_data.target_idx` point still exists or removed.
  // ==========================================================================================
  // Remove overlapped points after inserting the interpolated points
  // 再次去重：由于之前插入了两个新点（当前点和预测点），可能会产生新的重叠点
  control_data.interpolated_traj.points =
    autoware::motion_utils::removeOverlapPoints(control_data.interpolated_traj.points);
  // 重要提示：注释指出，去重操作会导致之前计算的索引 nearest_idx 和 target_idx
  // 失效（因为点数减少了，索引偏移了）。 重新搜索索引： 利用之前保存的 nearest_point.pose 和
  // target_point.pose（位姿信息不受去重影响），在去重后的轨迹中重新搜索最近的点。

  control_data.nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    control_data.interpolated_traj.points, nearest_point.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);
  control_data.target_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    control_data.interpolated_traj.points, target_point.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);

  // send debug values
  m_debug_values.setValues(DebugValues::TYPE::PREDICTED_VEL, control_data.state_after_delay.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);

  // shift
  control_data.shift = getCurrentShift(control_data);  // 根据目标点判断车辆前进还是后退
  if (control_data.shift != m_prev_shift) {
    m_pid_vel.reset();  // 档位发生变化（例如从前进切换到后退），重置 PID 控制器
                        // m_pid_vel，防止积分项累积导致控制突变。
  }
  m_prev_shift = control_data.shift;

  // distance to stopline
  control_data.stop_dist = longitudinal_utils::calcStopDistance(
    current_pose, control_data.interpolated_traj, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);  // 计算车辆当前位置到轨迹中第一个速度为0的点（停车点）的距离。这个距离用于状态机判断是否进入停车或减速状态

  // pitch
  // NOTE: getPitchByTraj() calculates the pitch angle as defined in
  // ../media/slope_definition.drawio.svg while getPitchByPose() is not, so `raw_pitch` is reversed
  const double raw_pitch =
    (-1.0) *
    longitudinal_utils::getPitchByPose(
      current_pose.orientation);   // 获取车辆当前位置的俯仰角，乘以了 -1.0 以统一坐标系定义。
  m_lpf_pitch->filter(raw_pitch);  // 跟上一次俯仰角做一次加权过滤，得到当前俯仰角的加权值
  const double traj_pitch = longitudinal_utils::getPitchByTraj(
    control_data.interpolated_traj, control_data.target_idx,
    m_wheel_base);  // 根据轨迹点在 target_idx(预测位置) 处的几何形状（考虑轴距
                    // m_wheel_base）计算出的坡度。这反映了前方道路的倾斜情况。

  // 选择合适的值作为当前车辆所在位置的坡度。
  if (m_slope_source == SlopeSource::RAW_PITCH) {
    control_data.slope_angle = m_lpf_pitch->getValue();
  } else if (m_slope_source == SlopeSource::TRAJECTORY_PITCH) {
    control_data.slope_angle = traj_pitch;
  } else if (
    m_slope_source == SlopeSource::TRAJECTORY_ADAPTIVE ||
    m_slope_source == SlopeSource::TRAJECTORY_GOAL_ADAPTIVE) {
    // RAJECTORY_ADAPTIVE / TRAJECTORY_GOAL_ADAPTIVE（自适应模式）：
    // 如果车速很低（is_vel_slow）或者接近轨迹终点（is_close_to_trajectory_end），使用 IMU
    // 坡度（因为此时轨迹曲率大或结束，几何坡度可能不准或无意义）。 否则，使用轨迹坡度。
    // 变化率限制：如果使用了自适应模式，会对坡度的变化率进行限制（Clamp），基于最大/最小加加速度（Jerk）和重力常数，防止坡度估计突变导致加速度指令跳变。
    // if velocity is high, use target idx for slope, otherwise, use raw_pitch
    const bool is_vel_slow = control_data.current_motion.vel < m_adaptive_trajectory_velocity_th &&
                             m_slope_source == SlopeSource::TRAJECTORY_ADAPTIVE;

    const double goal_dist = autoware::motion_utils::calcSignedArcLength(
      control_data.interpolated_traj.points, current_pose.position,
      control_data.interpolated_traj.points.size() - 1);
    const bool is_close_to_trajectory_end =
      goal_dist < m_wheel_base && m_slope_source == SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;

    control_data.slope_angle =
      (is_close_to_trajectory_end || is_vel_slow) ? m_lpf_pitch->getValue() : traj_pitch;

    if (m_previous_slope_angle.has_value()) {
      constexpr double gravity_const = 9.8;
      control_data.slope_angle = std::clamp(
        control_data.slope_angle,
        m_previous_slope_angle.value() + m_min_jerk * control_data.dt / gravity_const,
        m_previous_slope_angle.value() + m_max_jerk * control_data.dt / gravity_const);
    }
    m_previous_slope_angle = control_data.slope_angle;
  } else {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 3000, "Slope source is not valid. Using raw_pitch option as default");
    control_data.slope_angle = m_lpf_pitch->getValue();
  }

  updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch, m_lpf_pitch->getValue());

  return control_data;
}

PidLongitudinalController::Motion PidLongitudinalController::calcEmergencyCtrlCmd(const double dt)
{
  // These accelerations are without slope compensation
  const auto & p = m_emergency_state_params;
  Motion raw_ctrl_cmd{p.vel, p.acc};

  raw_ctrl_cmd.vel =
    longitudinal_utils::applyDiffLimitFilter(raw_ctrl_cmd.vel, m_prev_raw_ctrl_cmd.vel, dt, p.acc);
  raw_ctrl_cmd.acc = std::clamp(raw_ctrl_cmd.acc, m_min_acc, m_max_acc);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, raw_ctrl_cmd.acc);
  raw_ctrl_cmd.acc =
    longitudinal_utils::applyDiffLimitFilter(raw_ctrl_cmd.acc, m_prev_raw_ctrl_cmd.acc, dt, p.jerk);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, raw_ctrl_cmd.acc);

  const auto virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    m_current_kinematic_state.pose.pose, "velocity control\n (emergency)", clock_->now(), 0,
    m_wheel_base + m_front_overhang);
  m_pub_virtual_wall_marker->publish(virtual_wall_marker);

  return raw_ctrl_cmd;
}

void PidLongitudinalController::changeControlState(
  const ControlState & control_state, const std::string & reason)
{
  if (control_state != m_control_state) {
    RCLCPP_DEBUG_STREAM(
      logger_,
      "controller state changed: " << toStr(m_control_state) << " -> " << toStr(control_state));
    if (control_state == ControlState::EMERGENCY) {
      RCLCPP_ERROR(logger_, "Emergency Stop since %s", reason.c_str());
    }
  }
  m_control_state = control_state;
}

void PidLongitudinalController::updateControlState(const ControlData & control_data)
{
  const double current_vel = control_data.current_motion.vel;
  const double stop_dist = control_data.stop_dist;  // 获取当前距离停车线（或轨迹终点）的距离

  // flags for state transition
  const auto & p = m_state_transition_params;

  // 从“正在停止(STOPPING)”状态切换到“驱动(DRIVE)”状态的条件：
  // 距离停车线的距离 > (驱动状态下的目标停车距离 + 一个偏移量)
  // 这意味着车离目标还比较远，需要重新加速驱动
  const bool departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  // 从“已停止(STOPPED)”状态切换到“驱动(DRIVE)”状态的条件：
  // 距离停车线的距离 > 驱动状态下的目标停车距离
  const bool departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  // NOTE: the same velocity threshold as autoware::motion_utils::searchZeroVelocity
  static constexpr double vel_epsilon = 1e-3;

  // 进入“正在停止(STOPPING)”状态的条件：
  // 距离停车线的距离 < 停止状态下的目标停车距离
  const bool stopping_condition = stop_dist < p.stopping_state_stop_dist;

  // 判断当前车速是否低于进入“已停止”状态的阈值
  const bool is_stopped = std::abs(current_vel) < p.stopped_state_entry_vel;

  // Case where the ego slips in the opposite direction of the gear due to e.g. a slope is also
  // considered as a stop
  const bool is_not_running = [&]() {
    if (control_data.shift == Shift::Forward) {
      if (is_stopped || current_vel < 0.0) {
        // NOTE: Stopped or moving backward
        return true;
      }
    } else {
      if (is_stopped || 0.0 < current_vel) {
        // NOTE: Stopped or moving forward
        return true;
      }
    }
    return false;
  }();
  // 如果车辆正在运行（!is_not_running），更新最后运行时间戳
  if (!is_not_running) {
    m_last_running_time = std::make_shared<rclcpp::Time>(clock_->now());
  }
  // 判断是否满足“已停止”的时间条件：
  // 如果最后运行时间存在，且当前时间与最后运行时间的差值 > 设定的持续时间阈值
  const bool stopped_condition =
    m_last_running_time
      ? (clock_->now() - *m_last_running_time).seconds() > p.stopped_state_entry_duration_time
      : false;

  // ==========================================================================================
  // NOTE: due to removeOverlapPoints() in getControlData() m_trajectory and
  // control_data.interpolated_traj have different size.
  // ==========================================================================================
  const double current_vel_cmd = std::fabs(
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);

  // 判断是否触发紧急状态 (EMERGENCY)，距离停车线过近（负值表示越过停车线） && 目标速度几乎为0
  const auto emergency_condition = [&]() {
    if (
      m_enable_overshoot_emergency && stop_dist < -p.emergency_state_overshoot_stop_dist &&
      current_vel_cmd < vel_epsilon) {
      return ResultWithReason{
        true, fmt::format("the target velocity {} is less than {}", current_vel_cmd, vel_epsilon)};
    }
    return ResultWithReason{false};
  }();
  const bool has_nonzero_target_vel = std::abs(current_vel_cmd) > 1.0e-5;

  const auto debug_msg_once = [this](const auto & s) { RCLCPP_DEBUG_ONCE(logger_, "%s", s); };

  // 判断车辆是否处于 Autoware 的自动控制模式下
  const bool is_under_control = m_current_operation_mode.is_autoware_control_enabled &&
                                m_current_operation_mode.mode == OperationModeState::AUTONOMOUS;

  // 跟之前不一样则开始记录时间
  if (is_under_control != m_prev_vehicle_is_under_control) {
    m_prev_vehicle_is_under_control = is_under_control;
    m_under_control_starting_time =
      is_under_control ? std::make_shared<rclcpp::Time>(clock_->now()) : nullptr;
  }

  if (m_control_state != ControlState::STOPPED) {
    m_prev_keep_stopped_condition = std::nullopt;
  }

  // transit state
  // in DRIVE state
  if (m_control_state == ControlState::DRIVE) {
    if (emergency_condition.result) {
      return changeControlState(ControlState::EMERGENCY, emergency_condition.reason);
    }
    if (!is_under_control && stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (m_enable_smooth_stop) {
      if (stopping_condition) {
        // predictions after input time delay
        const double pred_vel_in_target = control_data.state_after_delay.vel;
        const double pred_stop_dist =
          control_data.stop_dist -
          0.5 * (pred_vel_in_target + current_vel) * m_delay_compensation_time;
        m_smooth_stop.init(pred_vel_in_target, pred_stop_dist);
        return changeControlState(ControlState::STOPPING);
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return changeControlState(ControlState::STOPPED);
      }
    }
    return;
  }

  // in STOPPING state
  if (m_control_state == ControlState::STOPPING) {
    if (emergency_condition.result) {
      return changeControlState(ControlState::EMERGENCY, emergency_condition.reason);
    }
    if (stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (departure_condition_from_stopping) {
      m_pid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      // prevent the car from taking a long time to start to move
      m_prev_raw_ctrl_cmd.acc = std::max(0.0, m_prev_raw_ctrl_cmd.acc);
      return changeControlState(ControlState::DRIVE);
    }
    return;
  }

  // in STOPPED state
  if (m_control_state == ControlState::STOPPED) {
    // debug print
    if (has_nonzero_target_vel && !departure_condition_from_stopped) {
      debug_msg_once("target speed > 0, but departure condition is not met. Keep STOPPED.");
    }

    if (departure_condition_from_stopped) {
      // Let vehicle start after the steering is converged for dry steering
      const bool current_keep_stopped_condition =
        std::fabs(current_vel) < vel_epsilon && !lateral_sync_data_.is_steer_converged;
      // NOTE: Dry steering is considered unnecessary when the steering is converged twice in a
      //       row. This is because lateral_sync_data_.is_steer_converged is not the current but
      //       the previous value due to the order controllers' run and sync functions.
      const bool keep_stopped_condition =
        !m_prev_keep_stopped_condition ||
        (current_keep_stopped_condition || *m_prev_keep_stopped_condition);
      m_prev_keep_stopped_condition = current_keep_stopped_condition;
      if (m_enable_keep_stopped_until_steer_convergence && keep_stopped_condition) {
        // debug print
        if (has_nonzero_target_vel) {
          debug_msg_once("target speed > 0, but keep stop condition is met. Keep STOPPED.");
        }

        // publish debug marker
        if (is_under_control) {
          const auto virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
            m_current_kinematic_state.pose.pose, "velocity control\n(steering not converged)",
            clock_->now(), 0, m_wheel_base + m_front_overhang);
          m_pub_virtual_wall_marker->publish(virtual_wall_marker);
        }

        // keep STOPPED
        return;
      }

      m_pid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      m_lpf_acc_error->reset(0.0);
      return changeControlState(ControlState::DRIVE);
    }

    return;
  }

  // in EMERGENCY state
  if (m_control_state == ControlState::EMERGENCY) {
    if (stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (!emergency_condition.result) {
      if (!is_under_control) {
        // NOTE: On manual driving, no need stopping to exit the emergency.
        return changeControlState(ControlState::DRIVE);
      }
    }
    return;
  }

  RCLCPP_FATAL(logger_, "invalid state found.");
  return;
}

PidLongitudinalController::Motion PidLongitudinalController::calcCtrlCmd(
  const ControlData & control_data)
{
  const size_t target_idx = control_data.target_idx;

  // velocity and acceleration command
  Motion ctrl_cmd_as_pedal_pos{
    control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(target_idx).acceleration_mps2};

  // 如果车辆处于完全停止状态，直接使用配置好的静止参数，不进行复杂的 PID 计算。
  if (m_control_state == ControlState::STOPPED) {
    const auto & p = m_stopped_state_params;
    // 设置静止时的目标速度（通常为0）和加速度（通常为0或轻微负值以保持刹车）
    ctrl_cmd_as_pedal_pos.vel = p.vel;
    ctrl_cmd_as_pedal_pos.acc = p.acc;

    m_prev_raw_ctrl_cmd.vel = 0.0;
    m_prev_raw_ctrl_cmd.acc = 0.0;

    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, ctrl_cmd_as_pedal_pos.acc);
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, ctrl_cmd_as_pedal_pos.acc);
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, ctrl_cmd_as_pedal_pos.acc);

    RCLCPP_DEBUG(
      logger_, "[Stopped]. vel: %3.3f, acc: %3.3f", ctrl_cmd_as_pedal_pos.vel,
      ctrl_cmd_as_pedal_pos.acc);
  } else {
    // 初始化原始控制命令，同样以轨迹点的前馈值为初值
    Motion raw_ctrl_cmd{
      control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps,
      control_data.interpolated_traj.points.at(target_idx).acceleration_mps2};
    if (m_control_state == ControlState::EMERGENCY) {
      raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);
    } else {
      if (m_control_state == ControlState::DRIVE) {
        raw_ctrl_cmd.vel = control_data.interpolated_traj.points.at(control_data.target_idx)
                             .longitudinal_velocity_mps;
        // 应用速度反馈控制 (PID + Feedforward)，获得加速度
        raw_ctrl_cmd.acc = applyVelocityFeedback(control_data);
        // 在停车前保持一定的制动力，防止溜车或抖动
        raw_ctrl_cmd = keepBrakeBeforeStop(control_data, raw_ctrl_cmd, target_idx);

        RCLCPP_DEBUG(
          logger_,
          "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
          "feedback_ctrl_cmd.ac: %3.3f",
          raw_ctrl_cmd.vel, raw_ctrl_cmd.acc, control_data.dt, control_data.current_motion.vel,
          control_data.interpolated_traj.points.at(control_data.target_idx)
            .longitudinal_velocity_mps,
          raw_ctrl_cmd.acc);
      } else if (m_control_state == ControlState::STOPPING) {
        // 在 STOPPING 状态下，使用平滑停止算法计算加速度
        // 该算法会根据距离停车线的远近动态调整减速度
        raw_ctrl_cmd.acc = m_smooth_stop.calculate(
          control_data.stop_dist, control_data.current_motion.vel, control_data.current_motion.acc,
          m_vel_hist, m_delay_compensation_time, m_debug_values);
        raw_ctrl_cmd.vel = m_stopped_state_params.vel;

        RCLCPP_DEBUG(
          logger_, "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", raw_ctrl_cmd.vel,
          raw_ctrl_cmd.acc);
      }
      // 限制值过大
      raw_ctrl_cmd.acc = std::clamp(raw_ctrl_cmd.acc, m_min_acc, m_max_acc);
      m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, raw_ctrl_cmd.acc);
      raw_ctrl_cmd.acc = longitudinal_utils::applyDiffLimitFilter(
        raw_ctrl_cmd.acc, m_prev_raw_ctrl_cmd.acc, control_data.dt, m_max_jerk, m_min_jerk);
      m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, raw_ctrl_cmd.acc);
    }

    // store acceleration without slope compensation
    m_prev_raw_ctrl_cmd = raw_ctrl_cmd;

    // calc acc feedback
    const double vel_sign = (control_data.shift == Shift::Forward)
                              ? 1.0
                              : (control_data.shift == Shift::Reverse ? -1.0 : 0.0);
    const double acc_err = control_data.current_motion.acc * vel_sign - raw_ctrl_cmd.acc;
    m_debug_values.setValues(DebugValues::TYPE::ERROR_ACC, acc_err);
    m_lpf_acc_error->filter(acc_err);
    m_debug_values.setValues(DebugValues::TYPE::ERROR_ACC_FILTERED, m_lpf_acc_error->getValue());

    const double acc_cmd = raw_ctrl_cmd.acc - m_lpf_acc_error->getValue() * m_acc_feedback_gain;
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_FB_APPLIED, acc_cmd);
    RCLCPP_DEBUG(
      logger_,
      "[acc feedback]: raw_ctrl_cmd.acc: %1.3f, control_data.current_motion.acc: %1.3f, acc_cmd: "
      "%1.3f",
      raw_ctrl_cmd.acc, control_data.current_motion.acc, acc_cmd);

    ctrl_cmd_as_pedal_pos.acc =
      applySlopeCompensation(acc_cmd, control_data.slope_angle, control_data.shift);
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, ctrl_cmd_as_pedal_pos.acc);
    ctrl_cmd_as_pedal_pos.vel = raw_ctrl_cmd.vel;
  }

  storeAccelCmd(m_prev_raw_ctrl_cmd.acc);

  ctrl_cmd_as_pedal_pos.acc = longitudinal_utils::applyDiffLimitFilter(
    ctrl_cmd_as_pedal_pos.acc, m_prev_ctrl_cmd.acc, control_data.dt, m_max_acc_cmd_diff);

  // update debug visualization
  updateDebugVelAcc(control_data);

  RCLCPP_DEBUG(
    logger_, "[final output]: acc: %3.3f, v_curr: %3.3f", ctrl_cmd_as_pedal_pos.acc,
    control_data.current_motion.vel);

  return ctrl_cmd_as_pedal_pos;
}

// Do not use nearest_idx here
autoware_control_msgs::msg::Longitudinal PidLongitudinalController::createCtrlCmdMsg(
  const Motion & ctrl_cmd, const double & current_vel)
{
  // publish control command
  autoware_control_msgs::msg::Longitudinal cmd{};
  cmd.stamp = clock_->now();
  cmd.velocity = static_cast<decltype(cmd.velocity)>(ctrl_cmd.vel);
  cmd.acceleration = static_cast<decltype(cmd.acceleration)>(ctrl_cmd.acc);

  // store current velocity history
  m_vel_hist.push_back({clock_->now(), current_vel});
  while (m_vel_hist.size() >
         static_cast<size_t>(m_delay_compensation_time / m_longitudinal_ctrl_period)) {
    m_vel_hist.erase(m_vel_hist.begin());
  }

  m_prev_ctrl_cmd = ctrl_cmd;

  return cmd;
}

void PidLongitudinalController::publishDebugData(
  const Motion & ctrl_cmd, const ControlData & control_data)
{
  // set debug values
  m_debug_values.setValues(DebugValues::TYPE::DT, control_data.dt);
  m_debug_values.setValues(DebugValues::TYPE::CALCULATED_ACC, control_data.current_motion.acc);
  m_debug_values.setValues(DebugValues::TYPE::SHIFT, static_cast<double>(control_data.shift));
  m_debug_values.setValues(DebugValues::TYPE::STOP_DIST, control_data.stop_dist);
  m_debug_values.setValues(DebugValues::TYPE::CONTROL_STATE, static_cast<double>(m_control_state));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PUBLISHED, ctrl_cmd.acc);

  // publish debug values
  autoware_internal_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = clock_->now();
  for (const auto & v : m_debug_values.getValues()) {
    debug_msg.data.push_back(static_cast<decltype(debug_msg.data)::value_type>(v));
  }
  m_pub_debug->publish(debug_msg);

  // slope angle
  autoware_internal_debug_msgs::msg::Float32MultiArrayStamped slope_msg{};
  slope_msg.stamp = clock_->now();
  slope_msg.data.push_back(
    static_cast<decltype(slope_msg.data)::value_type>(control_data.slope_angle));
  m_pub_slope->publish(slope_msg);
}

double PidLongitudinalController::getDt()
{
  double dt;
  if (!m_prev_control_time) {
    dt = m_longitudinal_ctrl_period;
    m_prev_control_time = std::make_shared<rclcpp::Time>(clock_->now());
  } else {
    dt = (clock_->now() - *m_prev_control_time).seconds();
    *m_prev_control_time = clock_->now();
  }
  const double max_dt = m_longitudinal_ctrl_period * 2.0;
  const double min_dt = m_longitudinal_ctrl_period * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

enum PidLongitudinalController::Shift PidLongitudinalController::getCurrentShift(
  const ControlData & control_data) const
{
  constexpr double epsilon = 1e-5;

  const double target_vel =
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return m_prev_shift;
}

void PidLongitudinalController::storeAccelCmd(const double accel)
{
  if (m_control_state == ControlState::DRIVE) {
    // convert format
    autoware_control_msgs::msg::Longitudinal cmd;
    cmd.stamp = clock_->now();
    cmd.acceleration = static_cast<decltype(cmd.acceleration)>(accel);

    // store published ctrl cmd
    m_ctrl_cmd_vec.emplace_back(cmd);
  } else {
    // reset command
    m_ctrl_cmd_vec.clear();
  }

  // remove unused ctrl cmd
  if (m_ctrl_cmd_vec.size() <= 2) {
    return;
  }
  if ((clock_->now() - m_ctrl_cmd_vec.at(1).stamp).seconds() > m_delay_compensation_time) {
    m_ctrl_cmd_vec.erase(m_ctrl_cmd_vec.begin());
  }
}

double PidLongitudinalController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!m_enable_slope_compensation) {
    return input_acc;
  }
  const double pitch_limited = std::min(std::max(pitch, m_min_pitch_rad), m_max_pitch_rad);

  // Acceleration command is always positive independent of direction (= shift) when car is running
  double sign = (shift == Shift::Forward) ? 1.0 : (shift == Shift::Reverse ? -1.0 : 0);
  double compensated_acc = input_acc + sign * 9.81 * std::sin(pitch_limited);
  return compensated_acc;
}

PidLongitudinalController::Motion PidLongitudinalController::keepBrakeBeforeStop(
  const ControlData & control_data, const Motion & target_motion, const size_t nearest_idx) const
{
  Motion output_motion = target_motion;

  if (m_enable_brake_keeping_before_stop == false) {
    return output_motion;
  }
  const auto traj = control_data.interpolated_traj;

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj.points);
  if (!stop_idx) {
    return output_motion;
  }

  double min_acc_before_stop = std::numeric_limits<double>::max();
  size_t min_acc_idx = std::numeric_limits<size_t>::max();
  for (int i = static_cast<int>(*stop_idx); i >= 0; --i) {
    const auto ui = static_cast<size_t>(i);
    if (traj.points.at(ui).acceleration_mps2 > static_cast<float>(min_acc_before_stop)) {
      break;
    }
    min_acc_before_stop = traj.points.at(ui).acceleration_mps2;
    min_acc_idx = ui;
  }

  const double brake_keeping_acc = std::max(m_brake_keeping_acc, min_acc_before_stop);
  if (nearest_idx >= min_acc_idx && target_motion.acc > brake_keeping_acc) {
    output_motion.acc = brake_keeping_acc;
  }

  return output_motion;
}

std::pair<autoware_planning_msgs::msg::TrajectoryPoint, size_t>
PidLongitudinalController::calcInterpolatedTrajPointAndSegment(
  const autoware_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose) const
{
  if (traj.points.size() == 1) {
    return std::make_pair(traj.points.at(0), 0);
  }

  // apply linear interpolation
  return longitudinal_utils::lerpTrajectoryPoint(
    traj.points, pose, m_ego_nearest_dist_threshold, m_ego_nearest_yaw_threshold);
}

PidLongitudinalController::StateAfterDelay PidLongitudinalController::predictedStateAfterDelay(
  const Motion current_motion, const double delay_compensation_time) const
{
  const double current_vel = current_motion.vel;
  const double current_acc = current_motion.acc;
  double running_distance = 0.0;
  double pred_vel = current_vel;
  double pred_acc = current_acc;

  if (m_ctrl_cmd_vec.empty() || m_current_operation_mode.mode != OperationModeState::AUTONOMOUS) {
    // check time to stop
    const double time_to_stop = -current_vel / current_acc;
    const double delay_time_calculation =
      time_to_stop > 0.0 && time_to_stop < delay_compensation_time ? time_to_stop
                                                                   : delay_compensation_time;
    // simple linear prediction
    pred_vel = current_vel + current_acc * delay_time_calculation;
    running_distance = std::abs(
      delay_time_calculation * current_vel +
      0.5 * current_acc * delay_time_calculation * delay_time_calculation);
    // avoid to change sign of current_vel and pred_vel
    return StateAfterDelay{pred_vel, pred_acc, running_distance};
  }

  for (std::size_t i = 0; i < m_ctrl_cmd_vec.size(); ++i) {
    if ((clock_->now() - m_ctrl_cmd_vec.at(i).stamp).seconds() < delay_compensation_time) {
      // add velocity to accel * dt
      const double time_to_next_acc =
        (i == m_ctrl_cmd_vec.size() - 1)
          ? std::min(
              (clock_->now() - m_ctrl_cmd_vec.back().stamp).seconds(), delay_compensation_time)
          : std::min(
              (rclcpp::Time(m_ctrl_cmd_vec.at(i + 1).stamp) -
               rclcpp::Time(m_ctrl_cmd_vec.at(i).stamp))
                .seconds(),
              delay_compensation_time);
      const double acc = m_ctrl_cmd_vec.at(i).acceleration;
      // because acc_cmd is positive when vehicle is running backward
      pred_acc = std::copysignf(1.0, static_cast<float>(pred_vel)) * acc;
      running_distance += std::abs(
        std::abs(pred_vel) * time_to_next_acc + 0.5 * acc * time_to_next_acc * time_to_next_acc);
      pred_vel += pred_vel < 0.0 ? (-acc * time_to_next_acc) : (acc * time_to_next_acc);
      if (pred_vel / current_vel < 0.0) {
        // sign of velocity is changed
        pred_vel = 0.0;
        break;
      }
    }
  }

  return StateAfterDelay{pred_vel, pred_acc, running_distance};
}

double PidLongitudinalController::applyVelocityFeedback(const ControlData & control_data)
{
  // NOTE: Acceleration command is always positive even if the ego drives backward.
  // 对于汽车而言，运动只有加速或减速，没有后退
  const double vel_sign = (control_data.shift == Shift::Forward)
                            ? 1.0
                            : (control_data.shift == Shift::Reverse ? -1.0 : 0.0);
  const double current_vel = control_data.current_motion.vel;
  const auto target_motion = Motion{
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2};
  // 计算速度误差：(目标速度 - 当前速度) * 符号
  // 乘以 vel_sign 是为了确保误差的方向与车辆行驶方向一致。
  const double diff_vel = (target_motion.vel - current_vel) * vel_sign;
  const bool is_under_control = m_current_operation_mode.is_autoware_control_enabled &&
                                m_current_operation_mode.mode == OperationModeState::AUTONOMOUS;

  // 判断车辆是否在移动：当前速度绝对值 > 阈值
  const bool vehicle_is_moving = std::abs(current_vel) > m_current_vel_threshold_pid_integrate;
  // 获取处于自动控制模式下的持续时间
  const double time_under_control = getTimeUnderControl();
  // 判断车辆是否“卡住”：
  // 车辆未移动 且 处于自动控制模式的时间超过设定阈值。
  // 这通常用于判断车辆是否因为坡度或阻力而无法起步。
  const bool vehicle_is_stuck =
    !vehicle_is_moving && time_under_control > m_time_threshold_before_pid_integrate;

  // 最终决定是否启用积分项：
  // 1. 必须在自动控制模式下。
  // 2. 要么车辆正在移动（正常跟踪）。
  // 3. 要么启用了“低速积分”功能 且 车辆被判定为“卡住”（帮助起步）。
  const bool enable_integration =
    (vehicle_is_moving || (m_enable_integration_at_low_speed && vehicle_is_stuck)) &&
    is_under_control;

  // 跟上一次速度误差做一次加权平均，以平滑速度误差
  const double error_vel_filtered = m_lpf_vel_error->filter(diff_vel);

  std::vector<double> pid_contributions(3);
  // 调用 PID 控制器计算反馈加速度，输入项是速度误差，需要除以时间才是加速度，但是这个时间可以融入到 PID 的参数中
  // 参数：滤波后的误差、时间步长 dt、是否启用积分、输出贡献值的向量
  const double pid_acc =
    m_pid_vel.calculate(error_vel_filtered, control_data.dt, enable_integration, pid_contributions);

  // Feedforward scaling:
  // This is for the coordinate conversion where feedforward is applied, from Time to Arclength.
  // Details: For accurate control, the feedforward should be calculated in the arclength coordinate
  // system, not in the time coordinate system. Otherwise, even if FF is applied, the vehicle speed
  // deviation will be bigger.
  // Feedforward scaling:
  // 这是一个从时间坐标到弧长坐标的转换补偿。
  // 原理：如果当前车速远低于目标车速，说明车辆滞后于规划轨迹。
  // 此时如果直接应用规划的前馈加速度，可能不足以弥补速度差距。
  // 通过缩放，可以在速度偏差大时增强或减弱前馈的作用（具体效果取决于 ff_scale 的计算方式）。
  // 注意：这里的逻辑是 ff_scale = |v_curr| / |v_target|。
  // 如果 v_curr < v_target，ff_scale < 1，前馈减小？
  // 实际上，这段注释提到是为了 "coordinate conversion"，通常在某些实现中，
  // 当速度不匹配时，时间域的前馈加速度映射到空间域需要修正。
  constexpr double ff_scale_max = 2.0;  // for safety
  constexpr double ff_scale_min = 0.5;  // for safety
  const double ff_scale = std::clamp(
    std::abs(current_vel) / std::max(std::abs(target_motion.vel), 0.1), ff_scale_min, ff_scale_max);
  // 计算最终的前馈加速度：轨迹规划加速度 * 缩放因子
  const double ff_acc =
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2 * ff_scale;
  // 总加速度 = 前馈加速度 + PID 反馈加速度
  const double feedback_acc = ff_acc + pid_acc;

  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PID_APPLIED, feedback_acc);
  m_debug_values.setValues(DebugValues::TYPE::ERROR_VEL_FILTERED, error_vel_filtered);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_P_CONTRIBUTION, pid_contributions.at(0));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_I_CONTRIBUTION, pid_contributions.at(1));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_D_CONTRIBUTION, pid_contributions.at(2));
  m_debug_values.setValues(DebugValues::TYPE::FF_SCALE, ff_scale);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FF, ff_acc);

  return feedback_acc;
}

void PidLongitudinalController::updatePitchDebugValues(
  const double pitch_using, const double traj_pitch, const double localization_pitch,
  const double localization_pitch_lpf)
{
  const double to_degrees = (180.0 / static_cast<double>(M_PI));
  m_debug_values.setValues(DebugValues::TYPE::PITCH_USING_RAD, pitch_using);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_USING_DEG, pitch_using * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_RAD, localization_pitch_lpf);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_DEG, localization_pitch_lpf * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_RAD, localization_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_DEG, localization_pitch * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_RAD, traj_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_DEG, traj_pitch * to_degrees);
}

void PidLongitudinalController::updateDebugVelAcc(const ControlData & control_data)
{
  m_debug_values.setValues(DebugValues::TYPE::CURRENT_VEL, control_data.current_motion.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_ACC,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_ACC,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::ERROR_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps -
      control_data.current_motion.vel);
}

void PidLongitudinalController::setupDiagnosticUpdater()
{
  diag_updater_->add("control_state", this, &PidLongitudinalController::checkControlState);
}

void PidLongitudinalController::checkControlState(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (m_control_state == ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred due to ";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(m_control_state));
  stat.summary(level, msg);
}

double PidLongitudinalController::getTimeUnderControl()
{
  if (!m_under_control_starting_time) return 0.0;
  return (clock_->now() - *m_under_control_starting_time).seconds();
}

}  // namespace autoware::motion::control::pid_longitudinal_controller
