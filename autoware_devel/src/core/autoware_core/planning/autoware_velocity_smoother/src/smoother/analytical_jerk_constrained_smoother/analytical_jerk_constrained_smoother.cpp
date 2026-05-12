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

#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::msg::Pose pose;
  pose.position.x = tf_point.getX();
  pose.position.y = tf_point.getY();
  pose.position.z = tf_point.getZ();
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool applyMaxVelocity(
  const double max_velocity, const size_t start_index, const size_t end_index,
  TrajectoryPoints & output_trajectory)
{
  if (end_index < start_index || output_trajectory.size() < end_index) {
    return false;
  }

  for (size_t i = start_index; i <= end_index; ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps =
      std::min(output_trajectory.at(i).longitudinal_velocity_mps, static_cast<float>(max_velocity));
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }
  return true;
}

}  // namespace

namespace autoware::velocity_smoother
{
AnalyticalJerkConstrainedSmoother::AnalyticalJerkConstrainedSmoother(
  rclcpp::Node & node, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: SmootherBase(node, time_keeper)
{
  auto & p = smoother_param_;
  p.resample.ds_resample = node.declare_parameter<double>("resample.ds_resample");
  p.resample.num_resample = node.declare_parameter<int>("resample.num_resample");
  p.resample.delta_yaw_threshold = node.declare_parameter<double>("resample.delta_yaw_threshold");
  p.latacc.enable_constant_velocity_while_turning =
    node.declare_parameter<bool>("latacc.enable_constant_velocity_while_turning");
  p.latacc.constant_velocity_dist_threshold =
    node.declare_parameter<double>("latacc.constant_velocity_dist_threshold");
  p.forward.max_acc = node.declare_parameter<double>("forward.max_acc");
  p.forward.min_acc = node.declare_parameter<double>("forward.min_acc");
  p.forward.max_jerk = node.declare_parameter<double>("forward.max_jerk");
  p.forward.min_jerk = node.declare_parameter<double>("forward.min_jerk");
  p.forward.kp = node.declare_parameter<double>("forward.kp");
  p.backward.start_jerk = node.declare_parameter<double>("backward.start_jerk");
  p.backward.min_jerk_mild_stop = node.declare_parameter<double>("backward.min_jerk_mild_stop");
  p.backward.min_jerk = node.declare_parameter<double>("backward.min_jerk");
  p.backward.min_acc_mild_stop = node.declare_parameter<double>("backward.min_acc_mild_stop");
  p.backward.min_acc = node.declare_parameter<double>("backward.min_acc");
  p.backward.span_jerk = node.declare_parameter<double>("backward.span_jerk");
}

void AnalyticalJerkConstrainedSmoother::setParam(const Param & smoother_param)
{
  smoother_param_ = smoother_param;
}

AnalyticalJerkConstrainedSmoother::Param AnalyticalJerkConstrainedSmoother::getParam() const
{
  return smoother_param_;
}

// v0                -> 优先用上一帧平滑轨迹在当前 ego 位置的投影速度/加速度，当前自车速度
// a0                -> 当前自车加速度
// clipped           -> 从自车最近点开始裁剪后的前方轨迹
// traj_smoothed     -> 输出的平滑后轨迹
// debug_trajectories -> 这里基本没用
// false             -> 不发布 debug trajectory

bool AnalyticalJerkConstrainedSmoother::apply(
  const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
  TrajectoryPoints & output, [[maybe_unused]] std::vector<TrajectoryPoints> & debug_trajectories,
  [[maybe_unused]] const bool publish_debug_trajs)
{
  RCLCPP_DEBUG(logger_, "-------------------- Start --------------------");

  // guard
  if (input.empty()) {
    RCLCPP_DEBUG(logger_, "Fail. input trajectory is empty");
    return false;
  }

  // intput trajectory is cropped, so closest_index = 0
  const size_t closest_index = 0;

  // Find deceleration targets
  if (input.size() == 1) {
    RCLCPP_DEBUG(
      logger_,
      "Input trajectory size is too short. Cannot find decel targets and "
      "return v0, a0");
    output = input;
    output.front().longitudinal_velocity_mps = initial_vel;
    output.front().acceleration_mps2 = initial_acc;
    return true;
  }

  // 寻找低速点，前面速度在下降，后面速度又开始上升,如果 v[i] 比前一个点低，而且后一个点又开始变高,说明 i 是一个局部低速点
  std::vector<std::pair<size_t, double>> decel_target_indices;
  searchDecelTargetIndices(input, closest_index, decel_target_indices);
  RCLCPP_DEBUG(logger_, "Num deceleration targets: %zd", decel_target_indices.size());
  for (auto & index : decel_target_indices) {
    RCLCPP_DEBUG(
      logger_, "Target deceleration index: %ld, target velocity: %f", index.first, index.second);
  }

  // Apply filters according to deceleration targets
  TrajectoryPoints reference_trajectory = input;
  TrajectoryPoints filtered_trajectory = input;
  // 获取低点的速度和加速度
  for (size_t i = 0; i < decel_target_indices.size(); ++i) {
    size_t fwd_start_index;
    double fwd_start_vel;
    double fwd_start_acc;
    if (i == 0) {
      fwd_start_index = closest_index;
      fwd_start_vel = initial_vel;
      fwd_start_acc = initial_acc;
    } else {
      fwd_start_index = decel_target_indices.at(i - 1).first;
      fwd_start_vel = filtered_trajectory.at(fwd_start_index).longitudinal_velocity_mps;
      fwd_start_acc = filtered_trajectory.at(fwd_start_index).acceleration_mps2;
    }

    RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", fwd_start_index);
    // 从低速点开始，向前逐点积分,从某个起点状态 (v, a) 出发，沿轨迹向前积分，把整个轨迹都处理完，使速度尽量跟上参考速度，但加速度和 jerk 受限
    applyForwardJerkFilter(
      reference_trajectory, fwd_start_index, fwd_start_vel, fwd_start_acc, smoother_param_,
      filtered_trajectory); // 

    // 为什么需要后向 decel filter,对每个低速目标，需要从目标点往前反推：从哪里开始减速，才能刚好以目标速度到达目标点。
    // 当前 10 m/s
    // 前方 20 m 停车
    // 前向滤波还在尽量跟踪较高速度
    // 到停车点前才发现来不及刹
    size_t bwd_start_index = closest_index;
    double bwd_start_vel = initial_vel;
    double bwd_start_acc = initial_acc;
    for (int j = i; j >= 0; --j) { //根据前一个减速目标，向后走
      if (j == 0) {
        bwd_start_index = closest_index;
        bwd_start_vel = initial_vel;
        bwd_start_acc = initial_acc;
        break;
      }
      
      if (decel_target_indices.at(j - 1).second < decel_target_indices.at(j).second) {  // 这里可能存在一个加速过程，因此需要判断是否需要减速
        bwd_start_index = decel_target_indices.at(j - 1).first;
        bwd_start_vel = filtered_trajectory.at(bwd_start_index).longitudinal_velocity_mps; // 获取的是修改后的速度
        bwd_start_acc = filtered_trajectory.at(bwd_start_index).acceleration_mps2;
        break;
      }
    }
    std::vector<size_t> start_indices;
    if (bwd_start_index != fwd_start_index) {
      start_indices.push_back(bwd_start_index);
      start_indices.push_back(fwd_start_index);  //bwd_start_index <= fwd_start_index
    } else {
      start_indices.push_back(bwd_start_index);
    }

    const size_t decel_target_index = decel_target_indices.at(i).first;  // 这个点是前向积分处理的起点
    const double decel_target_vel = decel_target_indices.at(i).second;
    RCLCPP_DEBUG(
      logger_, "Apply backward decel filter from: %s, to: %ld (%f)",
      strStartIndices(start_indices).c_str(), decel_target_index, decel_target_vel);
    if (!applyBackwardDecelFilter(
          start_indices, decel_target_index, decel_target_vel, smoother_param_,
          filtered_trajectory)) {
      RCLCPP_DEBUG(
        logger_,
        "Failed to apply backward decel filter, so apply max velocity filter. max velocity = %f, "
        "start_index = %s, end_index = %zd",
        decel_target_vel, strStartIndices(start_indices).c_str(), filtered_trajectory.size() - 1);

      const double ep = 0.001;
      if (std::abs(decel_target_vel) < ep) {
        applyMaxVelocity(0.0, bwd_start_index, filtered_trajectory.size() - 1, filtered_trajectory);
        output = filtered_trajectory;
        RCLCPP_DEBUG(logger_, "-------------------- Finish --------------------");
        return true;
      }
      applyMaxVelocity(decel_target_vel, bwd_start_index, decel_target_index, reference_trajectory);
      RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", bwd_start_index);
      applyForwardJerkFilter(
        reference_trajectory, bwd_start_index, bwd_start_vel, bwd_start_acc, smoother_param_,
        filtered_trajectory);
    }
  }

  size_t start_index;
  double start_vel;
  double start_acc;
  if (decel_target_indices.empty() == true) {
    start_index = closest_index;
    start_vel = initial_vel;
    start_acc = initial_acc;
  } else {
    start_index = decel_target_indices.back().first;
    start_vel = filtered_trajectory.at(start_index).longitudinal_velocity_mps;
    start_acc = filtered_trajectory.at(start_index).acceleration_mps2;
  }
  RCLCPP_DEBUG(logger_, "Apply forward jerk filter from: %ld", start_index);
  // 理最后一个减速目标之后的剩余轨迹；如果没有减速目标，它则负责处理整条轨迹。
  applyForwardJerkFilter(
    reference_trajectory, start_index, start_vel, start_acc, smoother_param_, filtered_trajectory);

  output = filtered_trajectory;

  RCLCPP_DEBUG(logger_, "-------------------- Finish --------------------");
  return true;
}

TrajectoryPoints AnalyticalJerkConstrainedSmoother::resampleTrajectory(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const geometry_msgs::msg::Pose & current_pose,
  [[maybe_unused]] const double nearest_dist_threshold,
  [[maybe_unused]] const double nearest_yaw_threshold) const
{
  TrajectoryPoints output;
  if (input.empty()) {
    RCLCPP_WARN(logger_, "Input trajectory is empty");
    return input;
  }

  const double ds = 1.0 / static_cast<double>(smoother_param_.resample.num_resample);

  for (size_t i = 0; i < input.size() - 1; ++i) {
    double s = 0.0;
    const auto tp0 = input.at(i);
    const auto tp1 = input.at(i + 1);

    const double dist_thr = 0.001;  // 1mm
    const double dist_tp0_tp1 = autoware_utils_geometry::calc_distance2d(tp0, tp1);
    if (std::fabs(dist_tp0_tp1) < dist_thr) {
      output.push_back(input.at(i));
      continue;
    }

    for (size_t j = 0; j < static_cast<size_t>(smoother_param_.resample.num_resample); ++j) {
      auto tp = input.at(i);

      tp.pose = lerpByPose(tp0.pose, tp1.pose, s);
      tp.longitudinal_velocity_mps = tp0.longitudinal_velocity_mps;
      tp.heading_rate_rps = (1.0 - s) * tp0.heading_rate_rps + s * tp1.heading_rate_rps;
      tp.acceleration_mps2 = tp0.acceleration_mps2;
      // tp.accel.angular.z = (1.0 - s) * tp0.accel.angular.z + s * tp1.accel.angular.z;

      output.push_back(tp);

      s += ds;
    }
  }

  output.push_back(input.back());

  return output;
}

TrajectoryPoints AnalyticalJerkConstrainedSmoother::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit,
  const bool use_resampling, const double input_points_interval) const
{
  if (input.size() < 3) {
    return input;  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.

  // 重采样：后面曲率计算默认轨迹点间距是近似均匀的。若原始轨迹点间距不均匀，比如有的点隔 0.2 m，有的点隔 3 m，曲率计算会非常不稳定。
  const double points_interval = use_resampling ? 0.1 : input_points_interval;  // [m]

  TrajectoryPoints output;
  // since the resampling takes a long time, omit the resampling when it is not requested
  if (use_resampling) {
    std::vector<double> out_arclength;
    const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
    for (double s = 0; s < in_arclength.back(); s += points_interval) {
      out_arclength.push_back(s);
    }
    const auto output_traj = autoware::motion_utils::resampleTrajectory(
      autoware::motion_utils::convertToTrajectory(input), out_arclength);
    output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);
    output.back() = input.back();  // keep the final speed.
  } else {
    output = input;
  }

  // 计算曲率：计算某个点曲率时，不是用相邻的前后 0.1 m 点，而是用相距约 5 m 的点。
  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  // Decrease speed according to lateral G
  // 计算曲率限速影响范围，获取这个范围内最大的曲率，而不是只在曲率最大的地方限速
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));

  // 参数里面设置了不同速度使用不同的横向加速度限制
  // 函数会计算每个速度区间两端的：
  // a_lat_limit / v²
  // 比如第一个区间 [0, 5]：

  // v = 0:  1.0 / 0²   很大，代码里用 epsilon 防止除 0
  // v = 5:  1.0 / 25 = 0.04
  // 第二个区间 [5, 10]：

  // v = 5:   0.8 / 25  = 0.032
  // v = 10:  0.8 / 100 = 0.008
  // 第三个区间 [10, 20]：

  // v = 10:  0.6 / 100 = 0.006
  // v = 20:  0.6 / 400 = 0.0015
  // 这些值都是“允许曲率边界”。

  const auto lateral_acceleration_velocity_square_ratio_limits =
    computeLateralAccelerationVelocitySquareRatioLimits(); // 参数是工程测试出来的？速度越快，横向加速度越小，避免车子失控

  std::vector<int> filtered_points;
  for (size_t i = 0; i < output.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > before_decel_index ? i - before_decel_index : 0;
    const size_t end = std::min(output.size(), i + after_decel_index);
    for (size_t j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = computeVelocityLimitFromLateralAcc(
      curvature, lateral_acceleration_velocity_square_ratio_limits);
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);  // v_curvature_max可能为0的，保证最小是min_curve_velocity
    if (output.at(i).longitudinal_velocity_mps > v_curvature_max) {
      output.at(i).longitudinal_velocity_mps = v_curvature_max;
      filtered_points.push_back(i); //如果这个点被限速了，就把索引记到 filtered_points，后面用于“转弯时保持恒定速度”。
    }
  }

  // Keep constant velocity while turning
  // 把相邻限速点合并成转弯区间,如果当前限速点和上一个限速点距离小于阈值，就认为它们属于同一个弯道区域。如果距离超过阈值，就认为进入了另一个弯道，关闭上一段区间，开启新区间。因为仅按曲率逐点限速，速度可能会这样变化: 3.8 -> 3.4 -> 3.1 -> 3.3 -> 3.7,车辆过弯过程中会轻微加减速，影响舒适性，也可能引入纵向控制抖动。恒速过弯后变成：3.1 -> 3.1 -> 3.1 -> 3.1 -> 3.1,更符合实际驾驶：进弯前减速，弯中保持稳定速度，出弯后再加速。

  const double dist_threshold = smoother_param_.latacc.constant_velocity_dist_threshold;
  std::vector<std::tuple<size_t, size_t, double>> latacc_filtered_ranges;
  size_t start_index = 0;
  size_t end_index = 0;
  bool is_updated = false;
  double min_latacc_velocity;
  for (size_t i = 0; i < filtered_points.size(); ++i) {
    const size_t index = filtered_points.at(i);

    if (is_updated == false) {
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).longitudinal_velocity_mps;
      is_updated = true;
      continue;
    }

    if (
      autoware_utils_geometry::calc_distance2d(output.at(end_index), output.at(index)) <
      dist_threshold) {
      end_index = index;
      min_latacc_velocity = std::min(
        static_cast<double>(output.at(index).longitudinal_velocity_mps), min_latacc_velocity);
    } else {
      latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
      start_index = index;
      end_index = index;
      min_latacc_velocity = output.at(index).longitudinal_velocity_mps;
    }
  }
  if (is_updated) {
    latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
  }

  for (size_t i = 0; i < output.size(); ++i) {
    for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
      const size_t filtered_start_index = std::get<0>(lat_acc_filtered_range);
      const size_t filtered_end_index = std::get<1>(lat_acc_filtered_range);
      const double filtered_min_latacc_velocity = std::get<2>(lat_acc_filtered_range);

      if (
        filtered_start_index <= i && i <= filtered_end_index &&
        smoother_param_.latacc.enable_constant_velocity_while_turning) {
        output.at(i).longitudinal_velocity_mps = filtered_min_latacc_velocity;
        break;
      }
    }
  }

  return output;
}

bool AnalyticalJerkConstrainedSmoother::searchDecelTargetIndices(
  const TrajectoryPoints & trajectory, const size_t closest_index,
  std::vector<std::pair<size_t, double>> & decel_target_indices) const
{
  const double ep = -0.00001;
  const size_t start_index = std::max<size_t>(1, closest_index);
  std::vector<std::pair<size_t, double>> tmp_indices;
  for (size_t i = start_index; i < trajectory.size() - 1; ++i) {
    const double dv_before =
      trajectory.at(i).longitudinal_velocity_mps - trajectory.at(i - 1).longitudinal_velocity_mps;
    const double dv_after =
      trajectory.at(i + 1).longitudinal_velocity_mps - trajectory.at(i).longitudinal_velocity_mps;
    if (dv_before < ep && dv_after > ep) {
      tmp_indices.emplace_back(i, trajectory.at(i).longitudinal_velocity_mps);
    }
  }

  const unsigned int i = trajectory.size() - 1;
  const double dv_before =
    trajectory.at(i).longitudinal_velocity_mps - trajectory.at(i - 1).longitudinal_velocity_mps;
  if (dv_before < ep) {
    tmp_indices.emplace_back(i, trajectory.at(i).longitudinal_velocity_mps);
  }

  if (!tmp_indices.empty()) {
    for (unsigned int j = 0; j < tmp_indices.size() - 1; ++j) {
      const size_t index_err = 10;
      if (
        (tmp_indices.at(j + 1).first - tmp_indices.at(j).first < index_err) &&
        (tmp_indices.at(j + 1).second < tmp_indices.at(j).second)) {
        continue;
      }

      decel_target_indices.emplace_back(tmp_indices.at(j).first, tmp_indices.at(j).second);
    }
  }
  if (!tmp_indices.empty()) {
    decel_target_indices.emplace_back(tmp_indices.back().first, tmp_indices.back().second);
  }
  return true;
}


// 在 jerk/acc 限制下，尽量向参考速度上限靠近,适合处理加速、正常跟踪，但不能保证前方停车点一定能刹住，所以还需要后向减速滤波。
bool AnalyticalJerkConstrainedSmoother::applyForwardJerkFilter(
  const TrajectoryPoints & base_trajectory, const size_t start_index, const double initial_vel,
  const double initial_acc, const Param & params, TrajectoryPoints & output_trajectory) const
{
  output_trajectory.at(start_index).longitudinal_velocity_mps = initial_vel;
  output_trajectory.at(start_index).acceleration_mps2 = initial_acc;

  for (size_t i = start_index + 1; i < base_trajectory.size(); ++i) {
    const double prev_vel = output_trajectory.at(i - 1).longitudinal_velocity_mps;
    const double ds =
      autoware_utils_geometry::calc_distance2d(base_trajectory.at(i - 1), base_trajectory.at(i));
    const double dt = ds / std::max(prev_vel, 1.0);

    const double prev_acc = output_trajectory.at(i - 1).acceleration_mps2;
    const double curr_vel = std::max(prev_vel + prev_acc * dt, 0.0);

    const double error_vel = base_trajectory.at(i).longitudinal_velocity_mps - curr_vel;
    const double fb_acc = params.forward.kp * error_vel;
    const double limited_acc =
      std::max(params.forward.min_acc, std::min(params.forward.max_acc, fb_acc));
    const double fb_jerk = (limited_acc - prev_acc) / dt;
    const double limited_jerk =
      std::max(params.forward.min_jerk, std::min(params.forward.max_jerk, fb_jerk));

    const double curr_acc = prev_acc + limited_jerk * dt;

    output_trajectory.at(i).longitudinal_velocity_mps = curr_vel;
    output_trajectory.at(i).acceleration_mps2 = curr_acc;
  }

  return true;
}
// 已知前方某点 decel_target_index 的目标速度 decel_target_vel，
// 从哪些候选 start_index 开始减速？
// 用多大的负 jerk？
// 需要多少距离？
// 距离够不够？
// 如果够，把这段速度曲线写回 output_trajectory。


bool AnalyticalJerkConstrainedSmoother::applyBackwardDecelFilter(
  const std::vector<size_t> & start_indices, const size_t decel_target_index,
  const double decel_target_vel, const Param & params, TrajectoryPoints & output_trajectory) const
{
  const double ep = 0.001;

  double output_planning_jerk = -100.0;
  size_t output_start_index = 0;
  std::vector<double> output_dist_to_target;
  int output_type;
  std::vector<double> output_times;

  // start_indices:
  //   closest_index
  //   上一个减速目标点
  //   本轮前向滤波起点
// 为什么可能有多个？因为多个低速目标连续出现时，不一定应该从当前最近点开始减速，也可能从上一个低速区间之后开始处理。
  for (size_t start_index : start_indices) {
    double dist = 0.0;
    std::vector<double> dist_to_target(output_trajectory.size(), 0);
    dist_to_target.at(decel_target_index) = dist;
    // 找到速度超出目标速度的索引
    for (size_t i = start_index; i < decel_target_index; ++i) {
      if (output_trajectory.at(i).longitudinal_velocity_mps >= decel_target_vel) {
        start_index = i;
        break;
      }
    }
    // 计算每个点到目标点的距离
    for (size_t i = decel_target_index; i > start_index; --i) {
      dist += autoware_utils_geometry::calc_distance2d(
        output_trajectory.at(i - 1), output_trajectory.at(i));
      dist_to_target.at(i - 1) = dist;
    }

    RCLCPP_DEBUG(logger_, "Check enough dist to decel. start_index: %ld", start_index);
    double planning_jerk;
    int type;
    std::vector<double> times;
    double stop_dist;
    bool is_enough_dist = false;
    for (planning_jerk = params.backward.start_jerk; planning_jerk > params.backward.min_jerk - ep;
         planning_jerk += params.backward.span_jerk) { //尝试不同 jerk，优先使用温和的减速
      if (calcEnoughDistForDecel(
            output_trajectory, start_index, decel_target_vel, planning_jerk, params, dist_to_target,
            is_enough_dist, type, times, stop_dist)) {
        break;
      }
    }

    //如果所有 jerk 都不够,说明从这个候选起点出发，哪怕用最强允许 jerk 也刹不住，于是换下一个候选起点。
    if (!is_enough_dist) {
      RCLCPP_DEBUG(logger_, "Distance is not enough for decel with all jerk condition");
      continue;
    }

    // 选择最温和的 Jerk
    if (planning_jerk >= output_planning_jerk) {
      output_planning_jerk = planning_jerk;
      output_start_index = start_index;
      output_dist_to_target = dist_to_target;
      output_type = type;
      output_times = times;
      RCLCPP_DEBUG(
        logger_, "Update planning jerk: %f, start_index: %ld", planning_jerk, start_index);
    }
  }

  if (output_planning_jerk == -100.0) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is not enough for decel with all jerk and start index "
      "condition");
    return false;
  }

  RCLCPP_DEBUG(logger_, "Search decel start index");
  size_t decel_start_index = output_start_index;
  // 如果计算的 jerk 是最温和的 start_jerk，说明减速能力比较充足。此时它会尝试把减速起点往后推，找一个最晚还能刹住的位置。所以这里从目标点前一个点往回搜索，找到第一个距离足够的位置就停下。这一步只在 output_planning_jerk == start_jerk 时做，是因为如果已经需要更强 jerk，说明减速余量不大，再往后推可能不稳。
  if (output_planning_jerk == params.backward.start_jerk) {
    for (size_t i = decel_target_index - 1; i >= output_start_index; --i) {
      bool is_enough_dist = false;
      double stop_dist;
      if (calcEnoughDistForDecel(
            output_trajectory, i, decel_target_vel, output_planning_jerk, params,
            output_dist_to_target, is_enough_dist, output_type, output_times, stop_dist)) {
        decel_start_index = i;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    logger_,
    "Apply filter. decel_start_index: %ld, target_vel: %f, "
    "planning_jerk: %f, type: %d, times: %s",
    decel_start_index, decel_target_vel, output_planning_jerk, output_type,
    strTimes(output_times).c_str());
    // 真正把速度/加速度写入 output_trajectory。
  if (!applyDecelVelocityFilter(
        decel_start_index, decel_target_vel, output_planning_jerk, params, output_type,
        output_times, output_trajectory)) {
    RCLCPP_DEBUG(
      logger_,
      "[applyDecelVelocityFilter] dist is enough, but fail to plan backward decel velocity");
    return false;
  }

  return true;
}
// 判断从某个轨迹点 start_index 开始，用给定的负 jerk planning_jerk 减速，能不能在到达目标点之前降到 decel_target_vel。
// 当前点状态 v0, a0
// 目标速度 decel_target_vel
// 给定 jerk / 最小加速度约束
//       ↓
// 计算所需减速距离 stop_dist
//       ↓
// 和可用距离 allowed_dist 比较
//       ↓
// 距离够：返回 true，并输出 type/times/stop_dist
// 距离不够：返回 false

bool AnalyticalJerkConstrainedSmoother::calcEnoughDistForDecel(
  const TrajectoryPoints & trajectory, const size_t start_index, const double decel_target_vel,
  const double planning_jerk, const Param & params, const std::vector<double> & dist_to_target,
  bool & is_enough_dist, int & type, std::vector<double> & times, double & stop_dist) const
{
  const double v0 = trajectory.at(start_index).longitudinal_velocity_mps;
  const double a0 = trajectory.at(start_index).acceleration_mps2;
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    // 温和 jerk 能解决 -> 使用温和减速度
    // 需要强 jerk       -> 允许更大减速度

    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);
  type = 0;
  times.clear();
  stop_dist = 0.0;

  if (!analytical_velocity_planning_utils::calcStopDistWithJerkAndAccConstraints(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist)) {
    return false;
  }
// 按照当前 v0/a0、给定 jerk、给定 min_acc，
// 减速到目标速度需要 stop_dist 米。

// 如果目标点距离当前点有 allowed_dist 米，
// 并且 stop_dist 不超过 allowed_dist，
// 那么来得及减速。

  const double allowed_dist = dist_to_target.at(start_index);
  if (0.0 <= stop_dist && stop_dist <= allowed_dist) {
    RCLCPP_DEBUG(
      logger_,
      "Distance is enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
      "allowed_dist: %f",
      v0, a0, planning_jerk, stop_dist, allowed_dist);
    is_enough_dist = true;
    return true;
  }
  RCLCPP_DEBUG(
    logger_,
    "Distance is not enough. v0: %f, a0: %f, jerk: %f, stop_dist: %f, "
    "allowed_dist: %f",
    v0, a0, planning_jerk, stop_dist, allowed_dist);
  return false;
}

bool AnalyticalJerkConstrainedSmoother::applyDecelVelocityFilter(
  const size_t decel_start_index, const double decel_target_vel, const double planning_jerk,
  const Param & params, const int type, const std::vector<double> & times,
  TrajectoryPoints & output_trajectory) const
{
  const double v0 = output_trajectory.at(decel_start_index).longitudinal_velocity_mps;
  const double a0 = output_trajectory.at(decel_start_index).acceleration_mps2;
  const double jerk_acc = std::abs(planning_jerk);
  const double jerk_dec = planning_jerk;
  auto calcMinAcc = [&params](const double planning_jerk) {
    if (planning_jerk < params.backward.min_jerk_mild_stop) {
      return params.backward.min_acc;
    }
    return params.backward.min_acc_mild_stop;
  };
  const double min_acc = calcMinAcc(planning_jerk);

  if (!analytical_velocity_planning_utils::calcStopVelocityWithConstantJerkAccLimit(
        v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, decel_start_index,
        output_trajectory)) {
    return false;
  }

  return true;
}

std::string AnalyticalJerkConstrainedSmoother::strTimes(const std::vector<double> & times) const
{
  std::stringstream ss;
  unsigned int i = 0;
  for (double time : times) {
    ss << "time[" << i << "] = " << time << ", ";
    i++;
  }
  return ss.str();
}

std::string AnalyticalJerkConstrainedSmoother::strStartIndices(
  const std::vector<size_t> & start_indices) const
{
  std::stringstream ss;
  for (size_t i = 0; i < start_indices.size(); ++i) {
    if (i != (start_indices.size() - 1)) {
      ss << start_indices.at(i) << ", ";
    } else {
      ss << start_indices.at(i);
    }
  }
  return ss.str();
}

}  // namespace autoware::velocity_smoother
