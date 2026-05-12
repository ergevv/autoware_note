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

#include "autoware/velocity_smoother/node.hpp"

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "autoware/velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "autoware/velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// clang-format on
namespace autoware::velocity_smoother
{
VelocitySmootherNode::VelocitySmootherNode(const rclcpp::NodeOptions & node_options)
: Node("velocity_smoother", node_options),
  diagnostics_interface_(std::make_unique<DiagnosticsInterface>(this, "velocity_smoother"))
{
  using std::placeholders::_1;

  // set common params
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  wheelbase_ = vehicle_info.wheel_base_m;
  base_link2front_ = vehicle_info.max_longitudinal_offset_m;
  initCommonParam();
  over_stop_velocity_warn_thr_ = declare_parameter<double>("over_stop_velocity_warn_thr");

  // create time_keeper and its publisher
  // NOTE: This has to be called before setupSmoother to pass the time_keeper to the smoother.
  debug_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_);

  // create smoother
  setupSmoother(wheelbase_);

  // publishers, subscribers
  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_virtual_wall_ = create_publisher<MarkerArray>("~/virtual_wall", 1);
  pub_velocity_limit_ = create_publisher<VelocityLimit>(
    "~/output/current_velocity_limit_mps", rclcpp::QoS{1}.transient_local());
  pub_dist_to_stopline_ = create_publisher<Float32Stamped>("~/distance_to_stopline", 1);
  sub_current_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&VelocitySmootherNode::onCurrentTrajectory, this, _1));

  // parameter update
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&VelocitySmootherNode::onParameter, this, _1));

  // debug
  publish_debug_trajs_ = declare_parameter<bool>("publish_debug_trajs");
  debug_closest_velocity_ = create_publisher<Float32Stamped>("~/closest_velocity", 1);
  debug_closest_acc_ = create_publisher<Float32Stamped>("~/closest_acceleration", 1);
  debug_closest_jerk_ = create_publisher<Float32Stamped>("~/closest_jerk", 1);
  debug_closest_max_velocity_ = create_publisher<Float32Stamped>("~/closest_max_velocity", 1);
  debug_calculation_time_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  pub_trajectory_raw_ = create_publisher<Trajectory>("~/debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ =
    create_publisher<Trajectory>("~/debug/trajectory_external_velocity_limited", 1);
  pub_trajectory_latacc_filtered_ =
    create_publisher<Trajectory>("~/debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_steering_rate_limited_ =
    create_publisher<Trajectory>("~/debug/trajectory_steering_rate_limited", 1);
  pub_trajectory_resampled_ = create_publisher<Trajectory>("~/debug/trajectory_time_resampled", 1);

  external_velocity_limit_.velocity = node_param_.max_velocity;
  max_velocity_with_deceleration_ = node_param_.max_velocity;

  // publish default max velocity
  VelocityLimit max_vel_msg{};
  max_vel_msg.stamp = this->now();
  max_vel_msg.max_velocity = node_param_.max_velocity;
  pub_velocity_limit_->publish(max_vel_msg);

  clock_ = get_clock();

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
}

void VelocitySmootherNode::setupSmoother(const double wheelbase)
{
  // 追求最高舒适度：选择Jerk Filtered
  // 平衡性能和效果：选择L2或L∞ Pseudo Jerk
  // 实时性要求高：选择Analytical
  // 参数调节灵活性：Jerk Filtered和Analytical提供更多可调参数
  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      smoother_ = std::make_shared<JerkFilteredSmoother>(*this, time_keeper_);

      // Set Publisher for jerk filtered algorithm
      pub_forward_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/forward_filtered_trajectory", 1);
      pub_backward_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/backward_filtered_trajectory", 1);
      pub_merged_filtered_trajectory_ =
        create_publisher<Trajectory>("~/debug/merged_filtered_trajectory", 1);
      pub_closest_merged_velocity_ =
        create_publisher<Float32Stamped>("~/closest_merged_velocity", 1);
      break;
    }
    case AlgorithmType::L2: {
      smoother_ = std::make_shared<L2PseudoJerkSmoother>(*this, time_keeper_);
      break;
    }
    case AlgorithmType::LINF: {
      smoother_ = std::make_shared<LinfPseudoJerkSmoother>(*this, time_keeper_);
      break;
    }
    case AlgorithmType::ANALYTICAL: {
      smoother_ = std::make_shared<AnalyticalJerkConstrainedSmoother>(*this, time_keeper_);
      break;
    }
    default:
      throw std::domain_error("[VelocitySmootherNode] invalid algorithm");
  }

  smoother_->setWheelBase(wheelbase);
}

template <class T>
bool get_param_general(
  const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

rcl_interfaces::msg::SetParametersResult VelocitySmootherNode::onParameter(
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

  // TODO(Horibe): temporally. replace with template.
  auto update_param_bool = [&](const std::string & name, bool & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_bool();
      return true;
    }
    return false;
  };

  {
    auto & p = node_param_;
    update_param_bool("enable_lateral_acc_limit", p.enable_lateral_acc_limit);
    update_param_bool("enable_steering_rate_limit", p.enable_steering_rate_limit);

    update_param("max_vel", p.max_velocity);
    update_param(
      "margin_to_insert_external_velocity_limit", p.margin_to_insert_external_velocity_limit);
    update_param("replan_vel_deviation", p.replan_vel_deviation);
    update_param("engage_velocity", p.engage_velocity);
    update_param("engage_acceleration", p.engage_acceleration);
    update_param("engage_exit_ratio", p.engage_exit_ratio);
    update_param("stopping_velocity", p.stopping_velocity);
    update_param("stopping_distance", p.stopping_distance);
    update_param("extract_ahead_dist", p.extract_ahead_dist);
    update_param("extract_behind_dist", p.extract_behind_dist);
    update_param("stop_dist_to_prohibit_engage", p.stop_dist_to_prohibit_engage);
    update_param("ego_nearest_dist_threshold", p.ego_nearest_dist_threshold);
    update_param("ego_nearest_yaw_threshold", p.ego_nearest_yaw_threshold);
    update_param_bool("plan_from_ego_speed_on_manual_mode", p.plan_from_ego_speed_on_manual_mode);
  }

  {
    auto p = smoother_->getBaseParam();
    update_param("normal.max_acc", p.max_accel);
    update_param("normal.min_acc", p.min_decel);
    update_param("stop_decel", p.stop_decel);
    update_param("normal.max_jerk", p.max_jerk);
    update_param("normal.min_jerk", p.min_jerk);
    update_param("min_curve_velocity", p.min_curve_velocity);
    update_param("decel_distance_before_curve", p.decel_distance_before_curve);
    update_param("decel_distance_after_curve", p.decel_distance_after_curve);
    update_param("max_trajectory_length", p.resample_param.max_trajectory_length);
    update_param("min_trajectory_length", p.resample_param.min_trajectory_length);
    update_param("resample_time", p.resample_param.resample_time);
    update_param("dense_resample_dt", p.resample_param.dense_resample_dt);
    update_param("min_interval_distance", p.resample_param.dense_min_interval_distance);
    update_param("sparse_resample_dt", p.resample_param.sparse_resample_dt);
    update_param("sparse_min_interval_distance", p.resample_param.sparse_min_interval_distance);
    update_param("resample_ds", p.sample_ds);
    update_param("curvature_threshold", p.curvature_threshold);
    get_param_general(parameters, "velocity_thresholds", p.velocity_thresholds);
    get_param_general(parameters, "lateral_acceleration_limits", p.lateral_acceleration_limits);
    get_param_general(parameters, "steering_angle_rate_limits", p.steering_angle_rate_limits);
    update_param("curvature_calculation_distance", p.curvature_calculation_distance);
    smoother_->setParam(p);
  }

  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      auto p = std::dynamic_pointer_cast<JerkFilteredSmoother>(smoother_)->getParam();
      update_param("jerk_weight", p.jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      update_param("over_j_weight", p.over_j_weight);
      update_param("jerk_filter_ds", p.jerk_filter_ds);
      std::dynamic_pointer_cast<JerkFilteredSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::L2: {
      auto p = std::dynamic_pointer_cast<L2PseudoJerkSmoother>(smoother_)->getParam();
      update_param("pseudo_jerk_weight", p.pseudo_jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      std::dynamic_pointer_cast<L2PseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::LINF: {
      auto p = std::dynamic_pointer_cast<LinfPseudoJerkSmoother>(smoother_)->getParam();
      update_param("pseudo_jerk_weight", p.pseudo_jerk_weight);
      update_param("over_v_weight", p.over_v_weight);
      update_param("over_a_weight", p.over_a_weight);
      std::dynamic_pointer_cast<LinfPseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::ANALYTICAL: {
      auto p = std::dynamic_pointer_cast<AnalyticalJerkConstrainedSmoother>(smoother_)->getParam();
      update_param("resample.delta_yaw_threshold", p.resample.delta_yaw_threshold);
      update_param(
        "latacc.constant_velocity_dist_threshold", p.latacc.constant_velocity_dist_threshold);
      update_param("forward.max_acc", p.forward.max_acc);
      update_param("forward.min_acc", p.forward.min_acc);
      update_param("forward.max_jerk", p.forward.max_jerk);
      update_param("forward.min_jerk", p.forward.min_jerk);
      update_param("forward.kp", p.forward.kp);
      update_param("backward.start_jerk", p.backward.start_jerk);
      update_param("backward.min_jerk_mild_stop", p.backward.min_jerk_mild_stop);
      update_param("backward.min_jerk", p.backward.min_jerk);
      update_param("backward.min_acc_mild_stop", p.backward.min_acc_mild_stop);
      update_param("backward.min_acc", p.backward.min_acc);
      update_param("backward.span_jerk", p.backward.span_jerk);
      std::dynamic_pointer_cast<AnalyticalJerkConstrainedSmoother>(smoother_)->setParam(p);
      break;
    }
    default:
      throw std::domain_error("[VelocitySmootherNode] invalid algorithm");
  }

  rcl_interfaces::msg::SetParametersResult result{};
  result.successful = true;
  result.reason = "success";
  return result;
}

void VelocitySmootherNode::initCommonParam()
{
  auto & p = node_param_;
  p.enable_lateral_acc_limit = declare_parameter<bool>("enable_lateral_acc_limit");
  p.enable_steering_rate_limit = declare_parameter<bool>("enable_steering_rate_limit");

  p.max_velocity = declare_parameter<double>("max_vel");
  p.margin_to_insert_external_velocity_limit =
    declare_parameter<double>("margin_to_insert_external_velocity_limit");
  p.replan_vel_deviation = declare_parameter<double>("replan_vel_deviation");
  p.engage_velocity = declare_parameter<double>("engage_velocity");
  p.engage_acceleration = declare_parameter<double>("engage_acceleration");
  p.engage_exit_ratio = declare_parameter<double>("engage_exit_ratio");
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);
  p.stopping_velocity = declare_parameter<double>("stopping_velocity");
  p.stopping_distance = declare_parameter<double>("stopping_distance");
  p.extract_ahead_dist = declare_parameter<double>("extract_ahead_dist");
  p.extract_behind_dist = declare_parameter<double>("extract_behind_dist");
  p.stop_dist_to_prohibit_engage = declare_parameter<double>("stop_dist_to_prohibit_engage");
  p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
  p.post_resample_param.max_trajectory_length =
    declare_parameter<double>("post_max_trajectory_length");
  p.post_resample_param.min_trajectory_length =
    declare_parameter<double>("post_min_trajectory_length");
  p.post_resample_param.resample_time = declare_parameter<double>("post_resample_time");
  p.post_resample_param.dense_resample_dt = declare_parameter<double>("post_dense_resample_dt");
  p.post_resample_param.dense_min_interval_distance =
    declare_parameter<double>("post_dense_min_interval_distance");
  p.post_resample_param.sparse_resample_dt = declare_parameter<double>("post_sparse_resample_dt");
  p.post_resample_param.sparse_min_interval_distance =
    declare_parameter<double>("post_sparse_min_interval_distance");
  p.algorithm_type = getAlgorithmType(declare_parameter<std::string>("algorithm_type"));

  p.plan_from_ego_speed_on_manual_mode =
    declare_parameter<bool>("plan_from_ego_speed_on_manual_mode");
}

void VelocitySmootherNode::publishTrajectory(const TrajectoryPoints & trajectory) const
{
  Trajectory publishing_trajectory = autoware::motion_utils::convertToTrajectory(trajectory);
  publishing_trajectory.header = base_traj_raw_ptr_->header;
  pub_trajectory_->publish(publishing_trajectory);
  published_time_publisher_->publish_if_subscribed(
    pub_trajectory_, publishing_trajectory.header.stamp);
}


void VelocitySmootherNode::calcExternalVelocityLimit()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!external_velocity_limit_ptr_) {
    external_velocity_limit_.acceleration_request.request = false;
    return;
  }

  // sender
  external_velocity_limit_.sender = external_velocity_limit_ptr_->sender;  // 记录外部限速的发送模块

  // on the first time, apply directly
  // 系统启动初期无历史数据时，直接应用限速，不计算减速距离。
  if (prev_output_.empty() || !current_closest_point_from_prev_output_) {
    external_velocity_limit_.velocity = external_velocity_limit_ptr_->max_velocity;
    pub_velocity_limit_->publish(*external_velocity_limit_ptr_);
    return;
  }

  constexpr double eps = 1.0E-04;
  const double margin = node_param_.margin_to_insert_external_velocity_limit;

  // Set distance as zero if ego vehicle is stopped and external velocity limit is zero
  // 场景: 车辆已经停止（速度接近 0）且限速也是 0。
  // 处理: 将减速距离设为 0。
  // 原因: 既然车已经停了，就不需要再预留减速距离，避免不必要的计算开销。
  if (
    std::fabs(current_odometry_ptr_->twist.twist.linear.x) < eps &&
    external_velocity_limit_.velocity < eps) {
    external_velocity_limit_.dist = 0.0;
  }

  //  平滑器正常情况下的最大加速度限制（从 ROS 参数服务器获取）。
  const auto base_max_acceleration = get_parameter("normal.max_acc").as_double();
  // acceleration_request: 判断是否需要临时提升加速度上限。条件是：
  // 外部消息启用了约束 (use_constraints)。
  // 外部要求的最大加速度大于平滑器的默认值。
  const auto acceleration_request =
    external_velocity_limit_ptr_->use_constraints &&
    base_max_acceleration < external_velocity_limit_ptr_->constraints.max_acceleration;
  // 当外部模块（如紧急制动）需要更大加速度时，临时提升平滑器的加速度限制。在紧急情况下（如前方突然出现障碍物），允许平滑器突破常规的舒适性限制，以更激进的方式减速。
  if (
    acceleration_request &&
    current_odometry_ptr_->twist.twist.linear.x < external_velocity_limit_ptr_->max_velocity) {
    external_velocity_limit_.acceleration_request.request = true;
    external_velocity_limit_.acceleration_request.max_acceleration =
      external_velocity_limit_ptr_->constraints.max_acceleration;
    external_velocity_limit_.acceleration_request.max_jerk =
      external_velocity_limit_ptr_->constraints.max_jerk;
  }

  // calculate distance and maximum velocity
  // to decelerate to external velocity limit with jerk and acceleration
  // constraints.
  // if external velocity limit decreases
  // 仅当限速变化时才重新计算减速距离，避免重复计算。
  // external_velocity_limit_.velocity 是模块内部保存的上一轮外部限速；external_velocity_limit_ptr_->max_velocity 是这次新收到的限速。
  // 如果两者几乎相等，就不重复计算减速距离。因为外部限速点距离会在后面的 updateDataForExternalVelocityLimit() 中根据车辆已行驶距离逐帧递减。
  if (
    std::fabs((external_velocity_limit_.velocity - external_velocity_limit_ptr_->max_velocity)) >
    eps) {
//       上一帧平滑输出轨迹在当前 ego 位置的投影速度和加速度。
// 原因是速度平滑模块希望跨帧连续。如果直接用传感器速度/加速度，噪声会让规划结果抖动；用上一帧轨迹状态可以让本帧规划从“上一次计划到这里时的状态”接着走。
    const double v0 = current_closest_point_from_prev_output_->longitudinal_velocity_mps;
    const double a0 = current_closest_point_from_prev_output_->acceleration_mps2;

    if (isEngageStatus(v0)) {
      // 车辆当前很慢，目标速度已经超过 engage velocity，正在起步阶段。这种情况下不做复杂的减速距离计算，直接把限速插在当前位置，因为起步时速度很低，本来就没有“需要提前很远减速”的问题。
      max_velocity_with_deceleration_ = external_velocity_limit_ptr_->max_velocity;
      external_velocity_limit_.dist = 0.0;
    } else {
      // 如果外部限速消息带了自己的 constraints，就用外部约束；否则用 smoother 默认参数。
      const auto & cstr = external_velocity_limit_ptr_->constraints;
      const auto a_min = external_velocity_limit_ptr_->use_constraints ? cstr.min_acceleration
                                                                       : smoother_->getMinDecel();
      const auto j_max =
        external_velocity_limit_ptr_->use_constraints ? cstr.max_jerk : smoother_->getMaxJerk();
      const auto j_min =
        external_velocity_limit_ptr_->use_constraints ? cstr.min_jerk : smoother_->getMinJerk();

      // If the closest acceleration is positive, velocity will increase
      // until the acceleration becomes zero
      // So we set the maximum increased velocity as the velocity limit
      if (a0 > 0) {
        max_velocity_with_deceleration_ = v0 - 0.5 * a0 * a0 / j_min; //当前加速度 a0 > 0，车辆正在加速。即使现在开始施加负 jerk j_min，加速度也不会瞬间变成 0，而是逐渐下降，这段时间内车辆速度仍然增加
      } else {
        max_velocity_with_deceleration_ = v0;
      }

      // 如果新限速低于未来可能达到的峰值速度，就需要规划一个减速距离
      if (external_velocity_limit_ptr_->max_velocity < max_velocity_with_deceleration_) {
        // TODO(mkuri) If v0 < external_velocity_limit_ptr_->max_velocity <
        // max_velocity_with_deceleration_ meets, stronger jerk than expected may be applied to
        // external velocity limit.
        // 当前速度还低于新限速，但因为当前加速度为正，车辆未来可能冲过新限速。这种情况下，如果要严格不超过新限速，需要很快压低加速度，可能导致实际需要的 jerk 比预期更强，所以代码给 warning。
        if (v0 < external_velocity_limit_ptr_->max_velocity) {
          RCLCPP_WARN(
            get_logger(),
            "Stronger jerk than expected may be applied to external velocity limit in this "
            "condition.");
        }

        double stop_dist = 0.0; //从 v0, a0 平滑降到目标速度 target_vel 所需距离
        std::map<double, double> jerk_profile;
        // 阶段 1：使用负 jerk，让加速度下降到 a_min
        // 阶段 2：保持 a_min 匀减速
        // 阶段 3：使用正 jerk，让加速度回到 0

        if (!trajectory_utils::calcStopDistWithJerkConstraints(
              v0, a0, j_max, j_min, a_min, external_velocity_limit_ptr_->max_velocity, jerk_profile,
              stop_dist)) {  // 基于当前速度/加速度，计算满足加加速度约束的减速距离
          RCLCPP_WARN(get_logger(), "Stop distance calculation failed!");
        }
        external_velocity_limit_.dist = stop_dist + margin;
      } else {
        max_velocity_with_deceleration_ = external_velocity_limit_ptr_->max_velocity;
        external_velocity_limit_.dist = 0.0;
      }
    }
  }

  external_velocity_limit_.velocity = external_velocity_limit_ptr_->max_velocity;
  pub_velocity_limit_->publish(*external_velocity_limit_ptr_);

  return;
}

bool VelocitySmootherNode::checkData() const
{
  if (!current_odometry_ptr_ || !base_traj_raw_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "wait topics : current_vel = %d, base_traj = %d, acceleration = %d",
      (bool)current_odometry_ptr_, (bool)base_traj_raw_ptr_, (bool)current_acceleration_ptr_);
    return false;
  }
  if (base_traj_raw_ptr_->points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "input trajectory size must > 1. Skip computation.");
    return false;
  }

  return true;
}

void VelocitySmootherNode::onCurrentTrajectory(const Trajectory::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  RCLCPP_DEBUG(get_logger(), "========================= run start =========================");
  stop_watch_.tic();

  diagnostics_interface_->clear();
  // 轨迹：~/input/trajectory
  // 里程计：/localization/kinematic_state
  // 当前加速度：~/input/acceleration
  // 外部限速：~/input/external_velocity_limit_mps
  // 模式状态：~/input/operation_mode_state  
  base_traj_raw_ptr_ = msg;

  // receive data
  current_odometry_ptr_ =
    sub_current_odometry_.take_data();  // 主动轮询的数据获取机制，它不依赖 ROS2
                                        // 的回调系统，而是直接从订阅队列中提取数据，
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();
  external_velocity_limit_ptr_ = sub_external_velocity_limit_.take_data();
  const auto operation_mode_ptr = sub_operation_mode_.take_data();
  if (operation_mode_ptr) {
    operation_mode_ = *operation_mode_ptr;
  }

  // guard
  if (!checkData()) {
    return;
  }

  // calculate trajectory velocity
  auto input_points =
    autoware::motion_utils::convertToTrajectoryPointArray(*base_traj_raw_ptr_);  // 提取点出来

  // guard for invalid trajectory
  input_points = autoware::motion_utils::removeOverlapPoints(input_points);
  if (input_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }

  // Set 0 at the end of the trajectory
  input_points.back().longitudinal_velocity_mps = 0.0;

  // calculate prev closest point
  if (!prev_output_.empty()) {
    // 在上一帧输出的平滑轨迹上，找到距离当前自车位置最近的轨迹点，并计算该点的完整运动状态（位置、速度、加速度）。
    //  使用上一帧轨迹的最近点状态作为初始条件
    //  确保相邻两帧轨迹在速度、加速度上平滑过渡
    //  避免轨迹跳变导致车辆顿挫
    current_closest_point_from_prev_output_ = calcProjectedTrajectoryPointFromEgo(prev_output_);
  }

  // calculate distance to insert external velocity limit
  calcExternalVelocityLimit();  //新的外部限速应该从轨迹前方多远开始生效，才能让车辆在 jerk 和加速度限制下平滑地降到这个速度。
  updateDataForExternalVelocityLimit();  // 基于当前速度/加速度，计算满足加加速度约束的减速距离

  // For negative velocity handling, multiple -1 to velocity if it is for reverse.
  // NOTE: this process must be in the beginning of the process
  is_reverse_ = isReverse(input_points);  // 检查是否存在任意一个点速度负向
  if (is_reverse_) {
    flipVelocity(input_points);
  }

  const auto output = calcTrajectoryVelocity(input_points);
  if (output.empty()) {
    RCLCPP_WARN(get_logger(), "Output Point is empty");
    return;
  }

  // Note that output velocity is resampled by linear interpolation
  // 使用线性插值使速度曲线更连续，优化器输出的点间距可能不均匀
  auto output_resampled = resampling::resampleTrajectory(
    output, current_odometry_ptr_->twist.twist.linear.x, current_odometry_ptr_->pose.pose,
    node_param_.ego_nearest_dist_threshold, node_param_.ego_nearest_yaw_threshold,
    node_param_.post_resample_param, false);

  // Set 0 at the end of the trajectory
  if (!output_resampled.empty()) {
    output_resampled.back().longitudinal_velocity_mps = 0.0;
  }

  // update previous step infomation
  updatePrevValues(output);

  // for reverse velocity
  // NOTE: this process must be in the end of the process
  if (is_reverse_) {
    flipVelocity(output_resampled);
  }

  // publish message
  publishTrajectory(output_resampled);

  // publish debug message
  publishStopDistance(output);
  publishClosestState(output);

  // Publish Calculation Time
  publishStopWatchTime();

  // Publish diagnostics
  diagnostics_interface_->publish(now());

  RCLCPP_DEBUG(get_logger(), "========================== run() end ==========================\n\n");
}

// 基于当前速度/加速度，计算满足加加速度约束的减速距离
void VelocitySmootherNode::updateDataForExternalVelocityLimit()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (prev_output_.empty()) {
    return;
  }

  // calculate distance to insert external velocity limit
  // external_velocity_limit_.dist 表示：从当前 ego 位置往前多少米插入外部限速点。但车辆每一帧都在往前走。
  const double travel_dist = calcTravelDistance();  //用上一帧输出轨迹 prev_output_，计算上一帧 ego 投影点和当前 ego 投影点之间的二维距离。也就是说，它估计“车辆沿上一帧规划轨迹前进了多少”。
  external_velocity_limit_.dist = std::max(external_velocity_limit_.dist - travel_dist, 0.0);
  RCLCPP_DEBUG(
    get_logger(), "run: travel_dist = %f, external_velocity_limit_dist_ = %f", travel_dist,
    external_velocity_limit_.dist);

  return;
}

// 从完整输入轨迹中截取 ego 附近的一段轨迹，叠加外部限速/停车限速，然后调用 smoothVelocity() 生成平滑后的速度轨迹
TrajectoryPoints VelocitySmootherNode::calcTrajectoryVelocity(
  const TrajectoryPoints & traj_input) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPoints output{};  // velocity is optimized by qp solver

  // Extract trajectory around self-position with desired forward-backward length
  const size_t input_closest =
    findNearestIndexFromEgo(traj_input);  // 查找轨迹上距离自车最近的点索引，不是简单只看欧氏距离，还会用yaw，距离近但朝向差太多的点可能不会被优先选中。这对交叉、掉头、重叠 lane 很重要。

  auto traj_extracted = trajectory_utils::extractPathAroundIndex(
    traj_input, input_closest, node_param_.extract_ahead_dist,
    node_param_
      .extract_behind_dist);  // 以最近点为中心，前后提取指定距离的轨迹段，远处轨迹对当前速度规划影响较小，减少计算量
  if (traj_extracted.empty()) {
    RCLCPP_WARN(get_logger(), "Fail to extract the path from the input trajectory");
    return prev_output_;
  }

  // Debug
  if (publish_debug_trajs_) {
    auto tmp = traj_extracted;
    if (is_reverse_) flipVelocity(tmp);  //如果是倒车，前面主流程里为了复用正向速度规划，会把负速度翻成正速度。debug 发布时再翻回去
    pub_trajectory_raw_->publish(toTrajectoryMsg(tmp));
  }

  // Apply external velocity limit
  applyExternalVelocityLimit(traj_extracted);  // 在轨迹上插入限速点（如之前提取的最大速度、红绿灯、临时限速区）

  // Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close
  const size_t traj_extracted_closest = findNearestIndexFromEgo(
    traj_extracted);  // 在提取后的局部轨迹上重新查找最近点索引,traj_extracted
                      // 是新的轨迹数组，索引已变化,后续 smoothVelocity 需要正确的最近点索引

  // Apply velocity to approach stop point
  applyStopApproachingVelocity(traj_extracted);  // 在接近停车点时限制速度

  // Debug
  if (publish_debug_trajs_) {
    auto tmp = traj_extracted;
    if (is_reverse_) flipVelocity(tmp);
    pub_trajectory_vel_lim_->publish(toTrajectoryMsg(traj_extracted));
  }

  // Smoothing velocity
  if (!smoothVelocity(traj_extracted, traj_extracted_closest, output)) {
    return prev_output_;
  }

  return output;
}

// 输入已经是局部轨迹，并且已经叠加了外部限速、停车点限速；它要做的是：进一步叠加弯道/转向约束，然后从 ego 最近点开始求一条满足纵向加速度与 jerk 约束的速度曲线
bool VelocitySmootherNode::smoothVelocity(
  const TrajectoryPoints & input, const size_t input_closest,
  TrajectoryPoints & traj_smoothed) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input.empty()) {
    return false;  // cannot apply smoothing
  }

  // Calculate initial motion for smoothing
  // 正常情况下，它优先用上一帧平滑轨迹在当前 ego 位置的投影速度/加速度，这样跨帧连续。只有首次运行、速度偏差过大、engage 起步、手动模式切自动等情况，才切换到 ego 当前速度或 engage 参数。
  const auto [initial_motion, type] =
    calcInitialMotion(input, input_closest);  

  // 根据速度来限制轨迹
  // Lateral acceleration limit
  // 
  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;
  const auto traj_lateral_acc_filtered =
    node_param_.enable_lateral_acc_limit
      ? smoother_->applyLateralAccelerationFilter(
          input, initial_motion.vel, initial_motion.acc, enable_smooth_limit, use_resampling)
      : input;  // 输入轨迹 → 计算各点曲率 → 根据曲率限制速度 → 输出滤波后轨迹

  // Steering angle rate limit (Note: set use_resample = false since it is resampled above)
  const auto traj_steering_rate_limited =
    node_param_.enable_steering_rate_limit
      ? smoother_->applySteeringRateLimit(traj_lateral_acc_filtered, false)
      : traj_lateral_acc_filtered;  // 避免方向盘转动过快，提升舒适性，横向加速度滤波后轨迹 →
                                    // 计算转向角变化率 → 限制超速变化 → 输出转向限制后轨迹

  // Resample trajectory with ego-velocity based interval distance
//   1. 让轨迹点间距更适合速度优化
//   2. 根据当前车速决定近处密、远处疏
// 低速时点距小，便于停车和低速控制；高速时点距适当变大，避免优化规模过大。
  auto traj_resampled = smoother_->resampleTrajectory(
    traj_steering_rate_limited, current_odometry_ptr_->twist.twist.linear.x,
    current_odometry_ptr_->pose.pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);

  const size_t traj_resampled_closest =
    findNearestIndexFromEgo(traj_resampled);  // 轨迹改变了，查找重采样轨迹上距离自车最近的点索引

  // Set 0[m/s] in the terminal point
  if (!traj_resampled.empty()) {
    traj_resampled.back().longitudinal_velocity_mps = 0.0;
  }

  // Publish Closest Resample Trajectory Velocity
  publishClosestVelocity(
    traj_resampled, current_odometry_ptr_->pose.pose, debug_closest_max_velocity_);

  // 完整轨迹：[后方点]─[后方点]─●(最近点)─[前方点]─[前方点]
  //                           ↓
  //                       自车位置

  // 裁剪后：            ●(最近点)─[前方点]─[前方点]
  // Clip trajectory from closest point
  // 优化器只处理从 ego 最近点往前的轨迹,后方点只是为了输出轨迹结构完整，优化完再补回。
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());

  // Set maximum acceleration before applying smoother. Depends on acceleration request from
  // external velocity limit
  // 外部有加速度请求？→ 是 → 使用外部约束值（紧急场景，允许更大加速度）
  //                ↓
  //                否 → 使用默认参数值（正常场景）
  const double smoother_max_acceleration =
    external_velocity_limit_.acceleration_request.request
      ? external_velocity_limit_.acceleration_request.max_acceleration
      : get_parameter("normal.max_acc").as_double();
  const double smoother_max_jerk = external_velocity_limit_.acceleration_request.request
                                     ? external_velocity_limit_.acceleration_request.max_jerk
                                     : get_parameter("normal.max_jerk").as_double();
  smoother_->setMaxAccel(smoother_max_acceleration);
  smoother_->setMaxJerk(smoother_max_jerk);

  // 根据初始速度和加速度约束，优化速度

  // 目标函数：min J = w_j·∫jerk² dt + w_v·∫(v-v_ref)² dt + w_a·∫(a-a_ref)² dt

  // 约束条件：
  //   v_min ≤ v ≤ v_max          (速度限制)
  //   a_min ≤ a ≤ a_max          (加速度限制)
  //   j_min ≤ j ≤ j_max          (加加速度限制)
  //   v(0) = initial_motion.vel  (初始速度)
  //   a(0) = initial_motion.acc  (初始加速度)
  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother_->apply(
        initial_motion.vel, initial_motion.acc, clipped, traj_smoothed, debug_trajectories,
        publish_debug_trajs_)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }

  // Set 0 velocity after input-stop-point
  // 问题：优化器可能因约束冲突无法精确达到 0 速度
  // 解决：强制修正停车点速度，保证安全性
  overwriteStopPoint(clipped, traj_smoothed);

  // 优化前裁剪：
  // 后方：[●]─[●]─[●]─┐
  // 前方：            ●─[●]─[●] → 优化器处理
  //                   ↓
  // 优化后合并：
  //   完整：[●]─[●]─[●]─●─[●]─[●]
  //优化只处理了 ego 前方，所以这里把后方点补回来，恢复完整局部轨迹。
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(), traj_resampled.begin() + traj_resampled_closest);

  // For the endpoint of the trajectory
  // 再次确保终点速度为 0，限制轨迹速度不超过最大速度，防止超速，即使前面某个滤波/优化步骤出现数值问题，也不能发布超过全局最大速度的轨迹。
  if (!traj_smoothed.empty()) {
    traj_smoothed.back().longitudinal_velocity_mps = 0.0;
  }

  // Max velocity filter for safety，
  trajectory_utils::applyMaximumVelocityLimit(
    traj_resampled_closest, traj_smoothed.size(), node_param_.max_velocity, traj_smoothed);

  // Insert behind velocity for output's consistency，会给后方点填速度。正常情况下，它会参考上一帧输出轨迹的速度；如果是首次、engage、偏差重规划等情况，则让后方点速度跟最近点一致，避免后方轨迹速度杂乱。
  insertBehindVelocity(traj_resampled_closest, type, traj_smoothed);

  RCLCPP_DEBUG(get_logger(), "smoothVelocity : traj_smoothed.size() = %lu", traj_smoothed.size());
  if (publish_debug_trajs_) {
    {
      auto tmp = traj_lateral_acc_filtered;
      if (is_reverse_) flipVelocity(tmp);
      pub_trajectory_latacc_filtered_->publish(toTrajectoryMsg(tmp));
    }
    {
      auto tmp = traj_resampled;
      if (is_reverse_) flipVelocity(tmp);
      pub_trajectory_resampled_->publish(toTrajectoryMsg(tmp));
    }
    {
      auto tmp = traj_steering_rate_limited;
      if (is_reverse_) flipVelocity(tmp);
      pub_trajectory_steering_rate_limited_->publish(toTrajectoryMsg(tmp));
    }

    for (auto & debug_trajectory : debug_trajectories) {
      debug_trajectory.insert(
        debug_trajectory.begin(), traj_resampled.begin(),
        traj_resampled.begin() + traj_resampled_closest);
      for (size_t i = 0; i < traj_resampled_closest; ++i) {
        debug_trajectory.at(i).longitudinal_velocity_mps =
          debug_trajectory.at(traj_resampled_closest).longitudinal_velocity_mps;
      }
    }
    publishDebugTrajectories(debug_trajectories);
  }

  return true;
}

void VelocitySmootherNode::insertBehindVelocity(
  const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const bool keep_closest_vel_for_behind =
    (type == InitializeType::EGO_VELOCITY || type == InitializeType::LARGE_DEVIATION_REPLAN ||
     type == InitializeType::ENGAGING);

  for (size_t i = output_closest - 1; i < output.size(); --i) {
    if (keep_closest_vel_for_behind) {
      output.at(i).longitudinal_velocity_mps = output.at(output_closest).longitudinal_velocity_mps;
      output.at(i).acceleration_mps2 = output.at(output_closest).acceleration_mps2;
    } else {
      // TODO(planning/control team) deal with overlapped lanes with the same direction
      const size_t seg_idx = [&]() {
        // with distance and yaw thresholds
        const auto opt_nearest_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose, node_param_.ego_nearest_dist_threshold,
          node_param_.ego_nearest_yaw_threshold);
        if (opt_nearest_seg_idx) {
          return opt_nearest_seg_idx.value();
        }

        // with distance threshold
        const auto opt_second_nearest_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose, node_param_.ego_nearest_dist_threshold);
        if (opt_second_nearest_seg_idx) {
          return opt_second_nearest_seg_idx.value();
        }

        return autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose.position);
      }();
      const auto prev_output_point =
        trajectory_utils::calcInterpolatedTrajectoryPoint(prev_output_, output.at(i).pose, seg_idx);

      // output should be always positive: TODO(Horibe) think better way
      output.at(i).longitudinal_velocity_mps =
        std::abs(prev_output_point.longitudinal_velocity_mps);
      output.at(i).acceleration_mps2 = prev_output_point.acceleration_mps2;
    }
  }
}

void VelocitySmootherNode::publishStopDistance(const TrajectoryPoints & trajectory) const
{
  const size_t closest = findNearestIndexFromEgo(trajectory);

  // stop distance calculation
  const double stop_dist_lim{50.0};
  double stop_dist{stop_dist_lim};
  const auto stop_idx{autoware::motion_utils::searchZeroVelocityIndex(trajectory)};
  if (stop_idx) {
    stop_dist = autoware::motion_utils::calcSignedArcLength(trajectory, closest, *stop_idx);
  } else {
    stop_dist = closest > 0 ? stop_dist : -stop_dist;
  }
  Float32Stamped dist_to_stopline{};
  dist_to_stopline.stamp = this->now();
  dist_to_stopline.data = std::clamp(stop_dist, -stop_dist_lim, stop_dist_lim);
  pub_dist_to_stopline_->publish(dist_to_stopline);
}

// calcInitialMotion()
//     │
//     ├─→ 有 prev_output 吗？
//     │   │
//     │   ├─ 否 → EGO_VELOCITY (首次运行)
//     │   │
//     │   └─ 是 → 继续判断
//     │           │
//     │           ├─→ 速度偏差 > 阈值？
//     │           │   │
//     │           │   ├─ 是 → LARGE_DEVIATION_REPLAN
//     │           │   │
//     │           │   └─ 否 → 继续判断
//     │           │           │
//     │           │           ├─→ 车速 < engage 阈值？
//     │           │           │   │
//     │           │           │   ├─ 是 → 目标速度 ≥ engage_velocity？
//     │           │           │   │   │
//     │           │           │   │   ├─ 是 → 停车距离 > 阈值？
//     │           │           │   │   │   │
//     │           │           │   │   │   ├─ 是 → ENGAGING
//     │           │           │   │   │   │
//     │           │           │   │   │   └─ 否 → 继续判断
//     │           │           │   │   │
//     │           │           │   │   └─ 否 → 继续判断
//     │           │           │   │
//     │           │           │   └─ 否 → 继续判断
//     │           │           │
//     │           │           ├─→ 手动模式且 plan_from_ego_speed？
//     │           │           │   │
//     │           │           │   ├─ 是 → EGO_VELOCITY
//     │           │           │   │
//     │           │           │   └─ 否 → NORMAL
//     │           │           │
//     │           │           └─→ 默认 → NORMAL

// input_traj	const TrajectoryPoints &	输入的轨迹点数组
// input_closest	const size_t	自车在轨迹上的最近点索引
// 返回值	std::pair<Motion, InitializeType>	初始运动状态 + 初始化类型
std::pair<Motion, VelocitySmootherNode::InitializeType> VelocitySmootherNode::calcInitialMotion(
  const TrajectoryPoints & input_traj, const size_t input_closest) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // vehicle_speed	自车当前纵向速度的绝对值
  // vehicle_acceleration	自车当前纵向加速度（可正可负）
  // target_vel	轨迹最近点的目标速度绝对值
  const double vehicle_speed = std::fabs(current_odometry_ptr_->twist.twist.linear.x);
  const double vehicle_acceleration = current_acceleration_ptr_->accel.accel.linear.x;
  const double target_vel = std::fabs(input_traj.at(input_closest).longitudinal_velocity_mps);

  // first time
  if (!current_closest_point_from_prev_output_) {  // 检查是否有上一帧的最近点信息
    Motion initial_motion = {vehicle_speed, 0.0};
    return {initial_motion, InitializeType::EGO_VELOCITY};
  }

  // desired_vel	上一帧轨迹最近点的速度（期望速度）
  // desired_acc	上一帧轨迹最近点的加速度（期望加速度）
  // vel_error	自车实际速度与期望速度的偏差
  // 为什么要计算速度偏差？
  // 检测车辆是否跟上了规划轨迹
  // 偏差过大时需要重新规划，避免轨迹跳变
  // when velocity tracking deviation is large
  const double desired_vel = current_closest_point_from_prev_output_->longitudinal_velocity_mps;
  const double desired_acc = current_closest_point_from_prev_output_->acceleration_mps2;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);

  if (std::fabs(vel_error) > node_param_.replan_vel_deviation) {
    RCLCPP_DEBUG(
      get_logger(),
      "calcInitialMotion : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, node_param_.replan_vel_deviation);
    // 场景：车辆紧急制动，但轨迹速度还很高
    //   轨迹速度：10 m/s
    //   自车速度：2 m/s
    //   偏差：8 m/s > 阈值

    // 如果继续用轨迹速度 → 优化器会命令车辆突然加速 → 危险！
    // 用自车速度 → 从当前速度平滑过渡 → 安全
    Motion initial_motion = {vehicle_speed, desired_acc};  // TODO(Horibe): use current acc
    return {initial_motion, InitializeType::LARGE_DEVIATION_REPLAN};
  }

  // if current vehicle velocity is low && base_desired speed is high,
  // use engage_velocity for engage vehicle
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= node_param_.engage_velocity) {
      //       场景 1: 前方 50m 才停车 → 可以起步 ✓
      // 场景 2: 前方 2m 就停车 → 不应起步，避免反复启停 ✗
      const double stop_dist = trajectory_utils::calcStopDistance(input_traj, input_closest);
      if (stop_dist > node_param_.stop_dist_to_prohibit_engage) {
        RCLCPP_DEBUG(
          get_logger(),
          "calcInitialMotion : vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, node_param_.engage_velocity, engage_vel_thr, stop_dist);
        const double engage_acceleration =
          external_velocity_limit_.acceleration_request.request
            ? external_velocity_limit_.acceleration_request.max_acceleration
            : node_param_.engage_acceleration;
        Motion initial_motion = {node_param_.engage_velocity, engage_acceleration};
        return {initial_motion, InitializeType::ENGAGING};
      }
      RCLCPP_DEBUG(
        get_logger(), "calcInitialMotion : stop point is close (%.3f[m]). no engage.", stop_dist);
    } else if (target_vel > 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clock_, 3000,
        "calcInitialMotion : target velocity(%.3f[m/s]) is lower than engage velocity(%.3f[m/s]). ",
        target_vel, node_param_.engage_velocity);
    }
  }

  // If the control mode is not AUTONOMOUS (vehicle is not under control of the planning module),
  // use ego velocity/acceleration in the planning for smooth transition from MANUAL to AUTONOMOUS.
  if (node_param_.plan_from_ego_speed_on_manual_mode) {  // could be false for debug purpose
    const bool is_in_autonomous_control = operation_mode_.is_autoware_control_enabled &&
                                          (operation_mode_.mode == OperationModeState::AUTONOMOUS ||
                                           operation_mode_.mode == OperationModeState::STOP);
    if (!is_in_autonomous_control) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *clock_, 10000, "Not in autonomous control. Plan from ego velocity.");
      // We should plan from the current vehicle speed, but if the initial value is greater than the
      // velocity limit, the current planning algorithm decelerates with a very high deceleration.
      // To avoid this, we set the initial value of the vehicle speed to be below the speed limit.
      const auto p = smoother_->getBaseParam();
      const auto v0 = std::min(target_vel, vehicle_speed);
      const auto a0 = std::clamp(vehicle_acceleration, p.min_decel, p.max_accel);
      const Motion initial_motion = {v0, a0};
      return {initial_motion, InitializeType::EGO_VELOCITY};
    }
  }
  // 这是最常见的情况：

  // 车辆正常跟随轨迹行驶
  // 速度偏差在允许范围内
  // 使用上一帧状态保证连续性
  // normal update: use closest in current_closest_point_from_prev_output
  Motion initial_motion = {desired_vel, desired_acc};
  RCLCPP_DEBUG(
    get_logger(),
    "calcInitialMotion : normal update. v0 = %f, a0 = %f, vehicle_speed = %f, target_vel = %f",
    initial_motion.vel, initial_motion.acc, vehicle_speed, target_vel);
  return {initial_motion, InitializeType::NORMAL};
}

void VelocitySmootherNode::overwriteStopPoint(
  const TrajectoryPoints & input, TrajectoryPoints & output) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(input);
  if (!stop_idx) {
    return;
  }

  // Get Closest Point from Output
  // TODO(planning/control team) deal with overlapped lanes with the same directions
  const auto nearest_output_point_idx = autoware::motion_utils::findNearestIndex(
    output, input.at(*stop_idx).pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);

  // check over velocity
  bool is_stop_velocity_exceeded{false};
  double input_stop_vel{};
  double output_stop_vel{};
  if (nearest_output_point_idx) {
    double optimized_stop_point_vel =
      output.at(*nearest_output_point_idx).longitudinal_velocity_mps;
    is_stop_velocity_exceeded = (optimized_stop_point_vel > over_stop_velocity_warn_thr_);
    input_stop_vel = input.at(*stop_idx).longitudinal_velocity_mps;
    output_stop_vel = output.at(*nearest_output_point_idx).longitudinal_velocity_mps;
    trajectory_utils::applyMaximumVelocityLimit(
      *nearest_output_point_idx, output.size(), 0.0, output);
    RCLCPP_DEBUG(
      get_logger(),
      "replan : input_stop_idx = %lu, stop velocity : input = %f, output = %f, thr = %f",
      *nearest_output_point_idx, input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  } else {
    input_stop_vel = -1.0;
    output_stop_vel = -1.0;
    RCLCPP_DEBUG(
      get_logger(),
      "replan : input_stop_idx = -1, stop velocity : input = %f, output = %f, thr = %f",
      input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  }

  diagnostics_interface_->add_key_value(
    "The velocity on the stop point is larger than 0.", is_stop_velocity_exceeded);
}

void VelocitySmootherNode::applyExternalVelocityLimit(TrajectoryPoints & traj) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (traj.size() < 1) {
    return;
  }

  trajectory_utils::applyMaximumVelocityLimit(
    0, traj.size(), max_velocity_with_deceleration_, traj);  //所有都设置为max_velocity_with_deceleration_，因为这个已经是最高的，这是在 calcExternalVelocityLimit() 中计算出的"考虑减速能力后的最高速度"。只是设置了大于max_velocity_with_deceleration_的设回max_velocity_with_deceleration_，小于的保持不变

  // insert the point at the distance of external velocity limit
  const auto & current_pose = current_odometry_ptr_->pose.pose;
  const size_t closest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj, current_pose, node_param_.ego_nearest_dist_threshold,
      node_param_.ego_nearest_yaw_threshold); //找到轨迹上距离自车最近的线段索引。不仅考虑欧氏距离，还考虑航向角差异.避免在 U 型转弯等场景下，错误地匹配到轨迹的另一端（虽然距离近但方向相反）。
  const auto inserted_index =
    autoware::motion_utils::insertTargetPoint(closest_seg_idx, external_velocity_limit_.dist, traj); //从自车当前位置向前推算的减速距离（在 calcExternalVelocityLimit() 中计算得出）对应的索引
  if (!inserted_index) {
    // 如果减速距离超过了轨迹长度，无法在轨迹内插入点。将轨迹最后一个点的速度限制为外部限速值
    traj.back().longitudinal_velocity_mps = std::min(
      traj.back().longitudinal_velocity_mps, static_cast<float>(external_velocity_limit_.velocity));
    return;
  }

  // apply external velocity limit from the inserted point
  trajectory_utils::applyMaximumVelocityLimit(
    *inserted_index, traj.size(), external_velocity_limit_.velocity, traj); // 从插入点开始，将后续所有轨迹点的速度上限设为 external_velocity_limit_.velocity

  // create virtual wall，发布数据，速度为0时，发布虚拟墙
  if (std::abs(external_velocity_limit_.velocity) < 1e-3) {
    const auto virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
      traj.at(*inserted_index).pose, external_velocity_limit_.sender, this->now(), 0,
      base_link2front_);
    pub_virtual_wall_->publish(virtual_wall_marker);
  }

  RCLCPP_DEBUG(
    get_logger(), "externalVelocityLimit : limit_vel = %.3f", external_velocity_limit_.velocity);
}

// 这个函数会寻找轨迹中的第一个 0 速度点，也就是停车点。如果存在停车点，则在停车点前 stopping_distance 范围内，把速度限制到 stopping_velocity，让车辆接近停止线或障碍物停车点时更稳，不要高速冲到最后再急刹。
void VelocitySmootherNode::applyStopApproachingVelocity(TrajectoryPoints & traj) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj);
  if (!stop_idx) {
    return;  // no stop point.
  }
  double distance_sum = 0.0;
  for (size_t i = *stop_idx - 1; i < traj.size(); --i) {  // search backward
    distance_sum += autoware_utils_geometry::calc_distance2d(traj.at(i), traj.at(i + 1));
    if (distance_sum > node_param_.stopping_distance) {
      break;
    }
    if (traj.at(i).longitudinal_velocity_mps > node_param_.stopping_velocity) {
      traj.at(i).longitudinal_velocity_mps = node_param_.stopping_velocity;
    }
  }
}

void VelocitySmootherNode::publishDebugTrajectories(
  const std::vector<TrajectoryPoints> & debug_trajectories) const
{
  auto debug_trajectories_tmp = debug_trajectories;
  if (node_param_.algorithm_type == AlgorithmType::JERK_FILTERED) {
    if (debug_trajectories_tmp.size() != 3) {
      RCLCPP_DEBUG(get_logger(), "Size of the debug trajectories is incorrect");
      return;
    }
    if (is_reverse_) {
      flipVelocity(debug_trajectories_tmp.at(0));
      flipVelocity(debug_trajectories_tmp.at(1));
      flipVelocity(debug_trajectories_tmp.at(2));
    }
    pub_forward_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories_tmp.at(0)));
    pub_backward_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories_tmp.at(1)));
    pub_merged_filtered_trajectory_->publish(toTrajectoryMsg(debug_trajectories_tmp.at(2)));
    publishClosestVelocity(
      debug_trajectories_tmp.at(2), current_odometry_ptr_->pose.pose, pub_closest_merged_velocity_);
  }
}

void VelocitySmootherNode::publishClosestVelocity(
  const TrajectoryPoints & trajectory, const Pose & current_pose,
  const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) const
{
  const auto closest_point = calcProjectedTrajectoryPoint(trajectory, current_pose);

  Float32Stamped vel_data{};
  vel_data.stamp = this->now();
  vel_data.data = std::max(closest_point.longitudinal_velocity_mps, static_cast<float>(0.0));
  pub->publish(vel_data);
}

void VelocitySmootherNode::publishClosestState(const TrajectoryPoints & trajectory)
{
  const auto closest_point = calcProjectedTrajectoryPointFromEgo(trajectory);

  auto publishFloat = [=](const double data, const auto pub) {
    Float32Stamped msg{};
    msg.stamp = this->now();
    msg.data = data;
    pub->publish(msg);
    return;
  };

  const double curr_vel{closest_point.longitudinal_velocity_mps};
  const double curr_acc{closest_point.acceleration_mps2};
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    prev_acc_ = curr_acc;
    return;
  }

  // Calculate jerk
  rclcpp::Time curr_time{this->now()};
  double dt = (curr_time - *prev_time_).seconds();
  double curr_jerk = (curr_acc - prev_acc_) / dt;

  // Publish data
  publishFloat(curr_vel, debug_closest_velocity_);
  publishFloat(curr_acc, debug_closest_acc_);
  publishFloat(curr_jerk, debug_closest_jerk_);

  // Update
  prev_acc_ = curr_acc;
  *prev_time_ = curr_time;
}

void VelocitySmootherNode::updatePrevValues(const TrajectoryPoints & final_result)
{
  prev_output_ = final_result;
  prev_closest_point_ = calcProjectedTrajectoryPointFromEgo(final_result);
}

VelocitySmootherNode::AlgorithmType VelocitySmootherNode::getAlgorithmType(
  const std::string & algorithm_name) const
{
  if (algorithm_name == "JerkFiltered") {
    return AlgorithmType::JERK_FILTERED;
  }
  if (algorithm_name == "L2") {
    return AlgorithmType::L2;
  }
  if (algorithm_name == "Linf") {
    return AlgorithmType::LINF;
  }
  if (algorithm_name == "Analytical") {
    return AlgorithmType::ANALYTICAL;
  }

  throw std::domain_error("[VelocitySmootherNode] undesired algorithm is selected.");
}

double VelocitySmootherNode::calcTravelDistance() const
{
  const auto closest_point = calcProjectedTrajectoryPointFromEgo(prev_output_);

  if (prev_closest_point_) {
    const double travel_dist =
      autoware_utils_geometry::calc_distance2d(*prev_closest_point_, closest_point);
    return travel_dist;
  }

  return 0.0;
}

bool VelocitySmootherNode::isEngageStatus(const double target_vel) const
{
  const double vehicle_speed = std::fabs(current_odometry_ptr_->twist.twist.linear.x);
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  return vehicle_speed < engage_vel_thr && target_vel >= node_param_.engage_velocity;
}

Trajectory VelocitySmootherNode::toTrajectoryMsg(
  const TrajectoryPoints & points, const std_msgs::msg::Header * header) const
{
  auto trajectory = autoware::motion_utils::convertToTrajectory(points);
  trajectory.header = header ? *header : base_traj_raw_ptr_->header;
  return trajectory;
}

size_t VelocitySmootherNode::findNearestIndexFromEgo(const TrajectoryPoints & points) const
{
  return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, current_odometry_ptr_->pose.pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);
}

bool VelocitySmootherNode::isReverse(const TrajectoryPoints & points) const
{
  if (points.empty()) return true;

  return std::any_of(
    points.begin(), points.end(), [](const auto & pt) { return pt.longitudinal_velocity_mps < 0; });
}
void VelocitySmootherNode::flipVelocity(TrajectoryPoints & points) const
{
  for (auto & pt : points) {
    pt.longitudinal_velocity_mps *= -1.0;
  }
}

void VelocitySmootherNode::publishStopWatchTime()
{
  Float64Stamped calculation_time_data{};
  calculation_time_data.stamp = this->now();
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
}

TrajectoryPoint VelocitySmootherNode::calcProjectedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & pose) const
{
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      trajectory, pose, node_param_.ego_nearest_dist_threshold,
      node_param_.ego_nearest_yaw_threshold);
  return trajectory_utils::calcInterpolatedTrajectoryPoint(trajectory, pose, current_seg_idx);
}

TrajectoryPoint VelocitySmootherNode::calcProjectedTrajectoryPointFromEgo(
  const TrajectoryPoints & trajectory) const
{
  return calcProjectedTrajectoryPoint(trajectory, current_odometry_ptr_->pose.pose);
}

}  // namespace autoware::velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::velocity_smoother::VelocitySmootherNode)
