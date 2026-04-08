// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

MpcLateralController::MpcLateralController(
  rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater)
: clock_(node.get_clock()), logger_(node.get_logger().get_child("lateral_controller"))
{
  const auto dp_int = [&](const std::string & s) { return node.declare_parameter<int>(s); };
  const auto dp_bool = [&](const std::string & s) { return node.declare_parameter<bool>(s); };
  const auto dp_double = [&](const std::string & s) { return node.declare_parameter<double>(s); };

  diag_updater_ = diag_updater;

  m_mpc = std::make_unique<MPC>(node);

  m_mpc->m_ctrl_period = node.get_parameter("ctrl_period").as_double();

  auto & p_filt = m_trajectory_filtering_param;
  p_filt.enable_path_smoothing = dp_bool("enable_path_smoothing");
  p_filt.path_filter_moving_ave_num = dp_int("path_filter_moving_ave_num");
  p_filt.curvature_smoothing_num_traj = dp_int("curvature_smoothing_num_traj");
  p_filt.curvature_smoothing_num_ref_steer = dp_int("curvature_smoothing_num_ref_steer");
  p_filt.traj_resample_dist = dp_double("traj_resample_dist");
  p_filt.extend_trajectory_for_end_yaw_control = dp_bool("extend_trajectory_for_end_yaw_control");

  m_mpc->m_use_steer_prediction = dp_bool("use_steer_prediction");
  m_mpc->m_param.steer_tau = dp_double("vehicle_model_steer_tau");

  /* stop state parameters */
  m_stop_state_entry_ego_speed = dp_double("stop_state_entry_ego_speed");
  m_stop_state_entry_target_speed = dp_double("stop_state_entry_target_speed");
  m_converged_steer_rad = dp_double("converged_steer_rad");
  m_keep_steer_control_until_converged = dp_bool("keep_steer_control_until_converged");
  m_new_traj_duration_time = dp_double("new_traj_duration_time");            // [s]
  m_new_traj_end_dist = dp_double("new_traj_end_dist");                      // [m]
  m_mpc_converged_threshold_rps = dp_double("mpc_converged_threshold_rps");  // [rad/s]

  /* mpc parameters */
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  const double wheelbase = vehicle_info.wheel_base_m;
  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;
  m_mpc->m_steer_lim = vehicle_info.max_steer_angle_rad;

  // steer rate limit depending on curvature
  const auto steer_rate_lim_dps_list_by_curvature =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_curvature");
  const auto curvature_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("curvature_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_curvature.size(); ++i) {
    m_mpc->m_steer_rate_lim_map_by_curvature.emplace_back(
      curvature_list_for_steer_rate_lim.at(i),
      steer_rate_lim_dps_list_by_curvature.at(i) * deg2rad);
  }

  // steer rate limit depending on velocity
  const auto steer_rate_lim_dps_list_by_velocity =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_velocity");
  const auto velocity_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("velocity_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_velocity.size(); ++i) {
    m_mpc->m_steer_rate_lim_map_by_velocity.emplace_back(
      velocity_list_for_steer_rate_lim.at(i), steer_rate_lim_dps_list_by_velocity.at(i) * deg2rad);
  }

  /* vehicle model setup */
  auto vehicle_model_ptr =
    createVehicleModel(wheelbase, m_mpc->m_steer_lim, m_mpc->m_param.steer_tau, node);
  m_mpc->setVehicleModel(vehicle_model_ptr);

  /* QP solver setup */
  auto qpsolver_ptr = createQPSolverInterface(node);
  m_mpc->setQPSolver(qpsolver_ptr);

  /* delay compensation */
  {
    const double delay_tmp = dp_double("input_delay");
    const double delay_step = std::round(delay_tmp / m_mpc->m_ctrl_period);
    m_mpc->m_param.input_delay = delay_step * m_mpc->m_ctrl_period;
    m_mpc->m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* steering offset compensation */
  enable_auto_steering_offset_removal_ =
    dp_bool("steering_offset.enable_auto_steering_offset_removal");
  steering_offset_ = createSteerOffsetEstimator(wheelbase, node);

  /* initialize low-pass filter */
  {
    const double steering_lpf_cutoff_hz = dp_double("steering_lpf_cutoff_hz");
    const double error_deriv_lpf_cutoff_hz = dp_double("error_deriv_lpf_cutoff_hz");
    m_mpc->initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  }

  // ego nearest index search
  const auto check_and_get_param = [&](const auto & param) {
    return node.has_parameter(param) ? node.get_parameter(param).as_double() : dp_double(param);
  };
  m_ego_nearest_dist_threshold = check_and_get_param("ego_nearest_dist_threshold");
  m_ego_nearest_yaw_threshold = check_and_get_param("ego_nearest_yaw_threshold");
  m_mpc->ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_mpc->ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  m_mpc->m_use_delayed_initial_state = dp_bool("use_delayed_initial_state");

  m_mpc->m_publish_debug_trajectories = dp_bool("publish_debug_trajectories");

  m_pub_predicted_traj = node.create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  m_pub_debug_values =
    node.create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
  m_pub_steer_offset = node.create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

  declareMPCparameters(node);

  /* get parameter updates */
  using std::placeholders::_1;
  m_set_param_res =
    node.add_on_set_parameters_callback(std::bind(&MpcLateralController::paramCallback, this, _1));

  m_mpc->initializeSteeringPredictor();

  m_mpc->setLogger(logger_);
  m_mpc->setClock(clock_);

  setupDiag();
}

MpcLateralController::~MpcLateralController()
{
}

/*
1. 运动学自行车模型 (kinematics)
特点:
基于运动学 (Kinematics) 原理，忽略轮胎侧偏、悬挂动力学等力学因素。
假设车轮纯滚动，无侧滑。
包含转向延迟: 构造函数中传入了
steer_tau，表示该模型考虑了转向执行机构的一阶延迟特性（即指令转角与实际转角之间存在动态响应过程）。
适用场景:
低速或中速行驶，路面附着力良好，对计算效率要求较高，且需要一定精度的场景。这是自动驾驶中最常用的简化模型。
2. 无延迟运动学自行车模型 (kinematics_no_delay)
特点:
同样基于运动学原理。
忽略转向延迟: 假设转向指令瞬间生效，实际转角等于指令转角。
模型复杂度最低，计算量最小。
适用场景:
极低速场景，或者转向系统响应极快、延迟可忽略不计的情况。也常用于快速原型验证或对实时性要求极高的嵌入式平台。
3. 动力学自行车模型 (dynamics)
特点:
基于动力学 (Dynamics) 原理，考虑了力的作用。
考虑轮胎侧偏: 引入了轮胎 cornering stiffness (cf, cr) 和各个车轮的质量分布 (mass_fl, mass_fr,
mass_rl, mass_rr)。 能够模拟高速转弯时的车身侧倾、轮胎侧滑等现象。
模型最复杂，计算量最大，需要更多的车辆物理参数。
适用场景:
高速行驶、极限工况（如紧急避障）、低附着力路面（如雨雪天），此时车辆的力学行为对轨迹跟踪精度影响显著，运动学模型误差过大。
*/
std::shared_ptr<VehicleModelInterface> MpcLateralController::createVehicleModel(
  const double wheelbase, const double steer_lim, const double steer_tau, rclcpp::Node & node)
{
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr;

  const std::string vehicle_model_type = node.declare_parameter<std::string>("vehicle_model_type");

  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModel>(wheelbase, steer_lim, steer_tau);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_lim);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "dynamics") {
    const double mass_fl = node.declare_parameter<double>("vehicle.mass_fl");
    const double mass_fr = node.declare_parameter<double>("vehicle.mass_fr");
    const double mass_rl = node.declare_parameter<double>("vehicle.mass_rl");
    const double mass_rr = node.declare_parameter<double>("vehicle.mass_rr");
    const double cf = node.declare_parameter<double>("vehicle.cf");
    const double cr = node.declare_parameter<double>("vehicle.cr");

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time
    vehicle_model_ptr =
      std::make_shared<DynamicsBicycleModel>(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    return vehicle_model_ptr;
  }

  RCLCPP_ERROR(logger_, "vehicle_model_type is undefined");
  return vehicle_model_ptr;
}

std::shared_ptr<QPSolverInterface> MpcLateralController::createQPSolverInterface(
  rclcpp::Node & node)
{
  std::shared_ptr<QPSolverInterface> qpsolver_ptr;

  const std::string qp_solver_type = node.declare_parameter<std::string>("qp_solver_type");

  if (qp_solver_type == "unconstraint_fast") {
    // 无约束：严格来说，它求解的是最小二乘问题（Least
    // Squares），通常用于处理没有硬约束（或约束已被简化/忽略）的情况，或者作为有约束求解器的初始猜测。
    // 快速：基于 Eigen 库的 LLT 分解（Cholesky
    // 分解），计算速度极快，适合对实时性要求极高且工况简单的场景。
    // 适用性：通常在路径非常平滑、车辆远离边界、不需要严格满足转向角速度限制等复杂约束时使用。
    qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
    return qpsolver_ptr;
  }

  if (qp_solver_type == "osqp") {
    // OSQP (Operator Splitting Quadratic Program)：一个开源的、基于算子分裂法的凸二次规划求解器。
    // 支持约束：能够处理不等式约束（如最大转向角、最大转向角速度、最大横向误差等）。
    // 鲁棒性：在自动驾驶中更常用，因为它能保证控制指令在物理限制范围内，提高安全性和舒适性。
    // 计算量：相比无约束求解器，计算开销稍大，但仍在实时可控范围内。
    qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger_, clock_);
    return qpsolver_ptr;
  }

  RCLCPP_ERROR(logger_, "qp_solver_type is undefined");
  return qpsolver_ptr;
}

std::shared_ptr<SteeringOffsetEstimator> MpcLateralController::createSteerOffsetEstimator(
  const double wheelbase, rclcpp::Node & node)
{
  const std::string ns = "steering_offset.";
  const auto vel_thres = node.declare_parameter<double>(ns + "update_vel_threshold");
  const auto steer_thres = node.declare_parameter<double>(ns + "update_steer_threshold");
  const auto limit = node.declare_parameter<double>(ns + "steering_offset_limit");
  const auto num = node.declare_parameter<int>(ns + "average_num");
  // 在实车控制中由于机械装配误差、轮胎磨损或传感器校准问题，当车辆直线行驶时，转向盘或转向轮的读数可能不为零（例如显示为
  // 0.5度）。如果直接使用这个带有偏差的值进行 MPC
  // 控制，会导致车辆无法走直线，产生持续的横向误差震荡。
  // 这个非常有必要了解技术细节，估计这个偏差也可以用来标定传感器跟前进方向的yaw角
  steering_offset_ =
    std::make_shared<SteeringOffsetEstimator>(wheelbase, num, vel_thres, steer_thres, limit);
  return steering_offset_;
}

void MpcLateralController::setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (m_mpc_solved_status.result) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "MPC succeeded.");
  } else {
    const std::string error_msg = "MPC failed due to " + m_mpc_solved_status.reason;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
  }
}

void MpcLateralController::setupDiag()
{
  diag_updater_->add("MPC_solve_checker", [&](auto & stat) { setStatus(stat); });
}

trajectory_follower::LateralOutput MpcLateralController::run(
  trajectory_follower::InputData const & input_data)
{
  // set input data
  setTrajectory(input_data.current_trajectory, input_data.current_odometry);

  // 保存最新的车辆里程计（位置、速度、朝向）和转向角报告。如果启用了自动转向偏移移除，从传感器读取的原始转向角中减去估算的机械零点偏差 
  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;
  if (enable_auto_steering_offset_removal_) {
    m_current_steering.steering_tire_angle -= steering_offset_->getOffset();
  }

  Lateral ctrl_cmd;
  Trajectory predicted_traj;
  Float32MultiArrayStamped debug_values;

  const bool is_under_control = input_data.current_operation_mode.is_autoware_control_enabled &&
                                input_data.current_operation_mode.mode ==
                                  autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;

  // 如果这是第一次运行，或者刚刚退出自动控制状态，重置 m_ctrl_cmd_prev。
  // getInitialControlCommand：通常将初始控制命令设为当前的实际转向角，避免上电瞬间方向盘突变。
  if (!m_is_ctrl_cmd_prev_initialized || !is_under_control) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  trajectory_follower::LateralHorizon ctrl_cmd_horizon{};
  // 输入：去偏后的转向角、当前车辆状态。
  // 输出：
  // ctrl_cmd：最优控制量。
  // mpc_solved_status：包含求解是否成功 (result) 和失败原因 (reason)。
  // ctrl_cmd_horizon：未来一段时间内的控制序列（用于预览或级联控制）。
  const auto mpc_solved_status = m_mpc->calculateMPC(
    m_current_steering, m_current_kinematic_state, ctrl_cmd, predicted_traj, debug_values,
    ctrl_cmd_horizon);

  if (
    (m_mpc_solved_status.result == true && mpc_solved_status.result == false) ||
    (!mpc_solved_status.result && mpc_solved_status.reason != m_mpc_solved_status.reason)) {
    RCLCPP_ERROR(logger_, "MPC failed due to %s", mpc_solved_status.reason.c_str());
  }
  m_mpc_solved_status = mpc_solved_status;  // for diagnostic updater

  diag_updater_->force_update();

  // reset previous MPC result
  // Note: When a large deviation from the trajectory occurs, the optimization stops and
  // the vehicle will return to the path by re-planning the trajectory or external operation.
  // After the recovery, the previous value of the optimization may deviate greatly from
  // the actual steer angle, and it may make the optimization result unstable.
  if (!mpc_solved_status.result || !is_under_control) {
    // 原因：MPC 是一种基于历史的优化算法（特别是涉及转向速率限制时）。如果求解失败，之前的优化状态（如上一帧的转向角）可能已经不可信或导致下一次求解约束冲突。重置为当前实际转向角可以“重启”优化器，防止误差累积。
    m_mpc->resetPrevResult(m_current_steering);
  } else {
    setSteeringToHistory(ctrl_cmd);
  }

  if (enable_auto_steering_offset_removal_) {
    // 在线学习：利用当前的车速和原始转向角（注意这里用的是 input_data 中的原始值，因为估计器需要对比“指令”与“实际”的偏差，或者基于运动学反推）来更新偏移量估计值。
    // 补偿输出：将估算出的偏移量加回到控制命令 ctrl_cmd 中。
    steering_offset_->updateOffset(
      m_current_kinematic_state.twist.twist,
      input_data.current_steering.steering_tire_angle);  // use unbiased steering
    ctrl_cmd.steering_tire_angle += steering_offset_->getOffset();
  }

  publishPredictedTraj(predicted_traj);
  publishDebugValues(debug_values);

  const auto createLateralOutput =
    [this](
      const auto & cmd, const bool is_mpc_solved,
      const auto & cmd_horizon) -> trajectory_follower::LateralOutput {
    trajectory_follower::LateralOutput output;
    output.control_cmd = createCtrlCmdMsg(cmd);
    output.control_cmd_horizon = createCtrlCmdHorizonMsg(cmd_horizon);
    // To be sure current steering of the vehicle is desired steering angle, we need to check
    // following conditions.
    // 1. At the last loop, mpc should be solved because command should be optimized output.
    // 2. The mpc should be converged.
    // 3. The steer angle should be converged.
    output.sync_data.is_steer_converged =
      is_mpc_solved && isMpcConverged() && isSteerConverged(cmd);

    return output;
  };

  if (isStoppedState()) {
    // Reset input buffer
    debug_throttle("Stopped state detected, use previous control command");
    for (auto & value : m_mpc->m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc->m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;
    return createLateralOutput(m_ctrl_cmd_prev, false, ctrl_cmd_horizon);
  }

  if (!mpc_solved_status.result) {
    debug_throttle("MPC is not solved, use stop control command");
    //求解失败，使用上一次控制指令
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  return createLateralOutput(ctrl_cmd, mpc_solved_status.result, ctrl_cmd_horizon);
}

bool MpcLateralController::isSteerConverged(const Lateral & cmd) const
{
  // wait for a while to propagate the trajectory shape to the output command when the trajectory
  // shape is changed.
  if (!m_has_received_first_trajectory || isTrajectoryShapeChanged()) {
    RCLCPP_DEBUG(logger_, "trajectory shaped is changed");
    return false;
  }

  const bool is_converged =
    std::abs(cmd.steering_tire_angle - m_current_steering.steering_tire_angle) <
    static_cast<float>(m_converged_steer_rad);

  return is_converged;
}

bool MpcLateralController::isReady(const trajectory_follower::InputData & input_data)
{
  setTrajectory(input_data.current_trajectory, input_data.current_odometry);
  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

  if (!m_mpc->hasVehicleModel()) {
    info_throttle("MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc->hasQPSolver()) {
    info_throttle("MPC does not have a QP solver");
    return false;
  }
  if (m_mpc->m_reference_trajectory.empty()) {
    info_throttle("trajectory size is zero.");
    return false;
  }

  return true;
}

void MpcLateralController::setTrajectory(
  const Trajectory & msg, const Odometry & current_kinematics)
{
  m_current_trajectory = msg;

  if (msg.points.size() < 3) {
    RCLCPP_DEBUG(logger_, "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(msg)) {
    RCLCPP_ERROR(logger_, "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc->setReferenceTrajectory(msg, m_trajectory_filtering_param, current_kinematics);

  // update trajectory buffer to check the trajectory shape change.
  // 保存了过去 duration_time
  // 秒内的所有轨迹，如果检测到轨迹形状突变（例如规划模块重新规划了一条完全不同的路径），MPC
  // 控制器会认为之前的优化状态（如前馈量、历史转向记录）不再可靠，从而触发重置或等待收敛逻辑，防止车辆因跟踪突变的轨迹而产生剧烈抖动。是一个滑动时间窗口管理器，确保只保留最近几秒的轨迹历史，用于后续的轨迹连续性检查。
  m_trajectory_buffer.push_back(m_current_trajectory);
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(m_trajectory_buffer.back().header.stamp) -
                           rclcpp::Time(m_trajectory_buffer.front().header.stamp);

    const double first_trajectory_duration_time = 5.0;
    const double duration_time =
      m_has_received_first_trajectory ? m_new_traj_duration_time : first_trajectory_duration_time;
    if (time_diff.seconds() < duration_time) {
      m_has_received_first_trajectory = true;
      break;
    }
    m_trajectory_buffer.pop_front();
  }
}

Lateral MpcLateralController::getStopControlCommand() const
{
  Lateral cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

Lateral MpcLateralController::getInitialControlCommand() const
{
  Lateral cmd;
  cmd.steering_tire_angle = m_current_steering.steering_tire_angle;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool MpcLateralController::isStoppedState() const
{
  const double current_vel = m_current_kinematic_state.twist.twist.linear.x;
  // If the nearest index is not found, return false
  if (
    m_current_trajectory.points.empty() || std::fabs(current_vel) > m_stop_state_entry_ego_speed) {
    return false;
  }

  const auto latest_published_cmd = m_ctrl_cmd_prev;  // use prev_cmd as a latest published command
  if (m_keep_steer_control_until_converged && !isSteerConverged(latest_published_cmd)) {
    debug_throttle("steering is not converged.");
    return false;  // not stopState: keep control
  }

  // Note: This function used to take into account the distance to the stop line
  // for the stop state judgement. However, it has been removed since the steering
  // control was turned off when approaching/exceeding the stop line on a curve or
  // emergency stop situation and it caused large tracking error.
  const size_t nearest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    m_current_trajectory.points, m_current_kinematic_state.pose.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);

  // It is possible that stop is executed earlier than stop point, and velocity controller
  // will not start when the distance from ego to stop point is less than 0.5 meter.
  // So we use a distance margin to ensure we can detect stopped state.
  static constexpr double distance_margin = 1.0;
  const double target_vel = std::invoke([&]() -> double {
    auto min_vel = m_current_trajectory.points.at(nearest).longitudinal_velocity_mps;
    auto covered_distance = 0.0;
    for (auto i = nearest + 1; i < m_current_trajectory.points.size(); ++i) {
      min_vel = std::min(min_vel, m_current_trajectory.points.at(i).longitudinal_velocity_mps);
      covered_distance += autoware_utils::calc_distance2d(
        m_current_trajectory.points.at(i - 1).pose, m_current_trajectory.points.at(i).pose);
      if (covered_distance > distance_margin) break;
    }
    return min_vel;
  });

  return std::fabs(target_vel) < m_stop_state_entry_target_speed;
}

Lateral MpcLateralController::createCtrlCmdMsg(const Lateral & ctrl_cmd)
{
  auto out = ctrl_cmd;
  out.stamp = clock_->now();
  m_steer_cmd_prev = out.steering_tire_angle;
  return out;
}

LateralHorizon MpcLateralController::createCtrlCmdHorizonMsg(
  const LateralHorizon & ctrl_cmd_horizon) const
{
  auto out = ctrl_cmd_horizon;
  const auto now = clock_->now();
  for (auto & cmd : out.controls) {
    cmd.stamp = now;
  }
  return out;
}

void MpcLateralController::publishPredictedTraj(Trajectory & predicted_traj) const
{
  predicted_traj.header.stamp = clock_->now();
  predicted_traj.header.frame_id = m_current_trajectory.header.frame_id;
  m_pub_predicted_traj->publish(predicted_traj);
}

void MpcLateralController::publishDebugValues(Float32MultiArrayStamped & debug_values) const
{
  debug_values.stamp = clock_->now();
  m_pub_debug_values->publish(debug_values);

  Float32Stamped offset;
  offset.stamp = clock_->now();
  offset.data = steering_offset_->getOffset();
  m_pub_steer_offset->publish(offset);
}

void MpcLateralController::setSteeringToHistory(const Lateral & steering)
{
  const auto time = clock_->now();
  if (m_mpc_steering_history.empty()) {
    m_mpc_steering_history.emplace_back(steering, time);
    m_is_mpc_history_filled = false;
    return;
  }

  m_mpc_steering_history.emplace_back(steering, time);

  // Check the history is filled or not.
  if (rclcpp::Duration(time - m_mpc_steering_history.begin()->second).seconds() >= 1.0) {
    m_is_mpc_history_filled = true;
    // remove old data that is older than 1 sec
    for (auto itr = m_mpc_steering_history.begin(); itr != m_mpc_steering_history.end(); ++itr) {
      if (rclcpp::Duration(time - itr->second).seconds() > 1.0) {
        m_mpc_steering_history.erase(m_mpc_steering_history.begin());
      } else {
        break;
      }
    }
  } else {
    m_is_mpc_history_filled = false;
  }
}

bool MpcLateralController::isMpcConverged()
{
  // If the number of variable below the 2, there is no enough data so MPC is not converged.
  if (m_mpc_steering_history.size() < 2) {
    return false;
  }

  // If the history is not filled, return false.

  if (!m_is_mpc_history_filled) {
    return false;
  }

  // Find the maximum and minimum values of the steering angle in the past 1 second.
  double min_steering_value = m_mpc_steering_history[0].first.steering_tire_angle;
  double max_steering_value = min_steering_value;
  for (size_t i = 1; i < m_mpc_steering_history.size(); i++) {
    if (m_mpc_steering_history.at(i).first.steering_tire_angle < min_steering_value) {
      min_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
    if (m_mpc_steering_history.at(i).first.steering_tire_angle > max_steering_value) {
      max_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
  }
  return (max_steering_value - min_steering_value) < m_mpc_converged_threshold_rps;
}

void MpcLateralController::declareMPCparameters(rclcpp::Node & node)
{
  m_mpc->m_param.prediction_horizon = node.declare_parameter<int>("mpc_prediction_horizon");
  m_mpc->m_param.prediction_dt = node.declare_parameter<double>("mpc_prediction_dt");

  const auto dp = [&](const auto & param) { return node.declare_parameter<double>(param); };

  auto & nw = m_mpc->m_param.nominal_weight;
  nw.lat_error = dp("mpc_weight_lat_error");
  nw.heading_error = dp("mpc_weight_heading_error");
  nw.heading_error_squared_vel = dp("mpc_weight_heading_error_squared_vel");
  nw.steering_input = dp("mpc_weight_steering_input");
  nw.steering_input_squared_vel = dp("mpc_weight_steering_input_squared_vel");
  nw.lat_jerk = dp("mpc_weight_lat_jerk");
  nw.steer_rate = dp("mpc_weight_steer_rate");
  nw.steer_acc = dp("mpc_weight_steer_acc");
  nw.terminal_lat_error = dp("mpc_weight_terminal_lat_error");
  nw.terminal_heading_error = dp("mpc_weight_terminal_heading_error");

  auto & lcw = m_mpc->m_param.low_curvature_weight;
  lcw.lat_error = dp("mpc_low_curvature_weight_lat_error");
  lcw.heading_error = dp("mpc_low_curvature_weight_heading_error");
  lcw.heading_error_squared_vel = dp("mpc_low_curvature_weight_heading_error_squared_vel");
  lcw.steering_input = dp("mpc_low_curvature_weight_steering_input");
  lcw.steering_input_squared_vel = dp("mpc_low_curvature_weight_steering_input_squared_vel");
  lcw.lat_jerk = dp("mpc_low_curvature_weight_lat_jerk");
  lcw.steer_rate = dp("mpc_low_curvature_weight_steer_rate");
  lcw.steer_acc = dp("mpc_low_curvature_weight_steer_acc");
  m_mpc->m_param.low_curvature_thresh_curvature = dp("mpc_low_curvature_thresh_curvature");

  m_mpc->m_param.zero_ff_steer_deg = dp("mpc_zero_ff_steer_deg");
  m_mpc->m_param.acceleration_limit = dp("mpc_acceleration_limit");
  m_mpc->m_param.velocity_time_constant = dp("mpc_velocity_time_constant");
  m_mpc->m_param.min_prediction_length = dp("mpc_min_prediction_length");
}

rcl_interfaces::msg::SetParametersResult MpcLateralController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = m_mpc->m_param;

  using MPCUtils::update_param;
  try {
    auto & nw = param.nominal_weight;
    auto & lcw = param.low_curvature_weight;

    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

    const std::string ns_nw = "mpc_weight_";
    update_param(parameters, ns_nw + "lat_error", nw.lat_error);
    update_param(parameters, ns_nw + "heading_error", nw.heading_error);
    update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
    update_param(parameters, ns_nw + "steering_input", nw.steering_input);
    update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
    update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
    update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
    update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
    update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
    update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

    const std::string ns_lcw = "mpc_low_curvature_weight_";
    update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
    update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
    update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
    update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
    update_param(parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
    update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
    update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
    update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

    update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
    update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
    update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
    update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

    // initialize input buffer
    update_param(parameters, "input_delay", param.input_delay);
    const double delay_step = std::round(param.input_delay / m_mpc->m_ctrl_period);
    const double delay = delay_step * m_mpc->m_ctrl_period;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      m_mpc->m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    m_mpc->m_param = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

bool MpcLateralController::isTrajectoryShapeChanged() const
{
  // TODO(Horibe): update implementation to check trajectory shape around ego vehicle.
  // Now temporally check the goal position.
  for (const auto & trajectory : m_trajectory_buffer) {
    const auto change_distance = autoware_utils::calc_distance2d(
      trajectory.points.back().pose, m_current_trajectory.points.back().pose);
    if (change_distance > m_new_traj_end_dist) {
      return true;
    }
  }
  return false;
}

bool MpcLateralController::isValidTrajectory(const Trajectory & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.orientation.w) || !isfinite(p.pose.orientation.x) ||
      !isfinite(p.pose.orientation.y) || !isfinite(p.pose.orientation.z) ||
      !isfinite(p.longitudinal_velocity_mps) || !isfinite(p.lateral_velocity_mps) ||
      !isfinite(p.heading_rate_rps) || !isfinite(p.front_wheel_angle_rad) ||
      !isfinite(p.rear_wheel_angle_rad)) {
      return false;
    }
  }
  return true;
}

}  // namespace autoware::motion::control::mpc_lateral_controller
