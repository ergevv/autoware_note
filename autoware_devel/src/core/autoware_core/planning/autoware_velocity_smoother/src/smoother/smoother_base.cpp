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

#include "autoware/velocity_smoother/smoother/smoother_base.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/velocity_smoother/resample.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::velocity_smoother
{

namespace
{
TrajectoryPoints applyPreProcess(
  const TrajectoryPoints & input, const double interval, const bool use_resampling)
{
  using autoware::motion_utils::calcArcLength;
  using autoware::motion_utils::convertToTrajectory;
  using autoware::motion_utils::convertToTrajectoryPointArray;
  using autoware::motion_utils::resampleTrajectory;

  if (!use_resampling) {
    return input;
  }

  TrajectoryPoints output;
  std::vector<double> arc_length;

  // since the resampling takes a long time, omit the resampling when it is not requested
  const auto traj_length = calcArcLength(input);
  for (double s = 0; s < traj_length; s += interval) {
    arc_length.push_back(s);
  }

  const auto points = resampleTrajectory(convertToTrajectory(input), arc_length);
  output = convertToTrajectoryPointArray(points);
  output.back() = input.back();  // keep the final speed.

  return output;
}
}  // namespace

SmootherBase::SmootherBase(
  rclcpp::Node & node, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: time_keeper_(time_keeper)
{
  auto & p = base_param_;
  p.max_accel = node.declare_parameter<double>("normal.max_acc");
  p.min_decel = node.declare_parameter<double>("normal.min_acc");
  p.stop_decel = node.declare_parameter<double>("stop_decel");
  p.max_jerk = node.declare_parameter<double>("normal.max_jerk");
  p.min_jerk = node.declare_parameter<double>("normal.min_jerk");
  p.min_decel_for_lateral_acc_lim_filter =
    node.declare_parameter<double>("min_decel_for_lateral_acc_lim_filter");
  p.sample_ds = node.declare_parameter<double>("resample_ds");
  p.curvature_threshold = node.declare_parameter<double>("curvature_threshold");
  p.lateral_acceleration_limits =
    node.declare_parameter<std::vector<double>>("lateral_acceleration_limits");
  p.velocity_thresholds = node.declare_parameter<std::vector<double>>("velocity_thresholds");
  p.steering_angle_rate_limits =
    node.declare_parameter<std::vector<double>>("steering_angle_rate_limits");
  p.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  p.decel_distance_before_curve = node.declare_parameter<double>("decel_distance_before_curve");
  p.decel_distance_after_curve = node.declare_parameter<double>("decel_distance_after_curve");
  p.min_curve_velocity = node.declare_parameter<double>("min_curve_velocity");
  p.resample_param.max_trajectory_length = node.declare_parameter<double>("max_trajectory_length");
  p.resample_param.min_trajectory_length = node.declare_parameter<double>("min_trajectory_length");
  p.resample_param.resample_time = node.declare_parameter<double>("resample_time");
  p.resample_param.dense_resample_dt = node.declare_parameter<double>("dense_resample_dt");
  p.resample_param.dense_min_interval_distance =
    node.declare_parameter<double>("dense_min_interval_distance");
  p.resample_param.sparse_resample_dt = node.declare_parameter<double>("sparse_resample_dt");
  p.resample_param.sparse_min_interval_distance =
    node.declare_parameter<double>("sparse_min_interval_distance");
}

void SmootherBase::setWheelBase(const double wheel_base)
{
  base_param_.wheel_base = wheel_base;
}

void SmootherBase::setMaxAccel(const double max_acceleration)
{
  base_param_.max_accel = max_acceleration;
}

void SmootherBase::setMaxJerk(const double max_jerk)
{
  base_param_.max_jerk = max_jerk;
}

void SmootherBase::setParam(const BaseParam & param)
{
  base_param_ = param;
}

SmootherBase::BaseParam SmootherBase::getBaseParam() const
{
  return base_param_;
}

double SmootherBase::getMaxAccel() const
{
  return base_param_.max_accel;
}

double SmootherBase::getMinDecel() const
{
  return base_param_.min_decel;
}

double SmootherBase::getMaxJerk() const
{
  return base_param_.max_jerk;
}

double SmootherBase::getMinJerk() const
{
  return base_param_.min_jerk;
}

// Template function for computing ratio limits
template <typename ThresholdType, typename ComputeRatioFunc>
std::vector<std::pair<double, double>> SmootherBase::computeRatioLimits(
  const std::vector<double> & velocity_thresholds,
  const std::vector<ThresholdType> & threshold_values, ComputeRatioFunc compute_ratio_func) const
{
  std::vector<std::pair<double, double>> output;
  constexpr double epsilon = 1e-5;

  // Process each velocity threshold
  for (size_t i = 0; i < velocity_thresholds.size(); ++i) {
    ThresholdType threshold = threshold_values[i];
    double vi = velocity_thresholds[i];
    double vi_1 = (i == 0) ? 0.0 : velocity_thresholds[i - 1];

    // Add the ratio pair for this threshold using the provided function
    output.push_back(std::make_pair(
      compute_ratio_func(threshold, vi_1, epsilon), compute_ratio_func(threshold, vi, epsilon)));
  }

  // Add the last part
  auto v_max = velocity_thresholds.back();
  output.push_back(
    std::make_pair(compute_ratio_func(threshold_values.back(), v_max, epsilon), 0.0));

  return output;
}

// Template function for computing velocity limits
template <typename RatioType, typename ThresholdType, typename ComputeVelocityFunc>
double SmootherBase::computeVelocityLimit(
  const RatioType local_ratio, const std::vector<std::pair<double, double>> & ratio_limits,
  const std::vector<double> & velocity_thresholds,
  const std::vector<ThresholdType> & threshold_values,
  ComputeVelocityFunc compute_velocity_func) const
{
  // Iterate through the limits in reverse order
  for (int i = static_cast<int>(ratio_limits.size()) - 1; i >= 0; --i) {
    const double lower = ratio_limits[i].second;
    const double higher = ratio_limits[i].first;

    // If ratio is higher than the upper bound, continue to the next range
    if (local_ratio > higher) { //说明这个弯太急了，即使在当前速度区间的低速端也无法满足横向加速度限制。所以当前速度区间不可用，继续往更低速度区间查。
      continue;
    }

    // If ratio is lower than or equal to the lower bound
    if (local_ratio <= lower) {
      if (i == static_cast<int>(ratio_limits.size()) - 1) {
        return std::numeric_limits<double>::max();  // max_double equivalent,这个约束本身不需要限速。
      } else {
        return velocity_thresholds[i];  //如果不是最后一个区间，就返回当前区间的上边界速度
      }
    }

    // Between lower and higher, calculate the velocity threshold
    ThresholdType current_threshold;
    // 在满足约束的情况下，允许的最高速度。以横向加速度为例，如果曲率很小，应该尽量返回高速度甚至不限制；如果曲率很大，才逐步落到低速区间。
    if (i == static_cast<int>(ratio_limits.size()) - 1) {
      current_threshold = threshold_values.back();
    } else {
      current_threshold = threshold_values[i];
    }

    return compute_velocity_func(current_threshold, local_ratio); //使用当前车速区间允许的加速度跟曲率推测速度
  }

  // If no appropriate range is found
  return 0.0;
}

std::vector<std::pair<double, double>>
SmootherBase::computeLateralAccelerationVelocitySquareRatioLimits() const
{
  auto compute_lateral_acc_ratio = [](double acc, double v, double epsilon) {
    return acc / (v * v + epsilon);
  };

  return computeRatioLimits<double>(
    base_param_.velocity_thresholds, base_param_.lateral_acceleration_limits,
    compute_lateral_acc_ratio);
}

std::vector<std::pair<double, double>> SmootherBase::computeSteerRateVelocityRatioLimits() const
{
  auto compute_steer_rate_ratio = [](double rate_deg, double v, double epsilon) {
    return autoware_utils_math::deg2rad(rate_deg) / (v + epsilon);
  };

  return computeRatioLimits<double>(
    base_param_.velocity_thresholds, base_param_.steering_angle_rate_limits,
    compute_steer_rate_ratio);
}

double SmootherBase::computeVelocityLimitFromLateralAcc(
  const double local_curvature,
  const std::vector<std::pair<double, double>> lateral_acceleration_velocity_square_ratio_limits)
  const
{
  auto compute_velocity = [](double acc, double curvature) {
    return std::sqrt(acc / std::max(curvature, 1.0E-5));
  };

  return computeVelocityLimit<double, double>(
    local_curvature, lateral_acceleration_velocity_square_ratio_limits,
    base_param_.velocity_thresholds, base_param_.lateral_acceleration_limits, compute_velocity);
}

double SmootherBase::computeVelocityLimitFromSteerRate(
  const double local_steer_rate_velocity_ratio,
  const std::vector<std::pair<double, double>> steer_rate_velocity_ratio_limits) const
{
  auto compute_velocity = [](double rate_deg, double ratio) {
    return autoware_utils_math::deg2rad(rate_deg) / ratio;
  };

  return computeVelocityLimit<double, double>(
    local_steer_rate_velocity_ratio, steer_rate_velocity_ratio_limits,
    base_param_.velocity_thresholds, base_param_.steering_angle_rate_limits, compute_velocity);
}

TrajectoryPoints SmootherBase::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit,
  const bool use_resampling, const double input_points_interval) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input.size() < 3) {
    return input;  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  TrajectoryPoints output;
  const double points_interval =
    use_resampling ? base_param_.sample_ds : input_points_interval;  // [m]
  // since the resampling takes a long time, omit the resampling when it is not requested
  if (use_resampling) {
    std::vector<double> out_arclength;
    const auto traj_length = autoware::motion_utils::calcArcLength(input);
    for (double s = 0; s < traj_length; s += points_interval) {
      out_arclength.push_back(s);
    }
    const auto output_traj = autoware::motion_utils::resampleTrajectory(
      autoware::motion_utils::convertToTrajectory(input), out_arclength);
    output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);
    output.back() = input.back();  // keep the final speed.
  } else {
    output = input;
  }

  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((base_param_.curvature_calculation_distance) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  //  Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));

  const auto latacc_min_vel_arr =
    enable_smooth_limit ? trajectory_utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
                            output, v0, a0, base_param_.min_jerk, base_param_.max_accel,
                            base_param_.min_decel_for_lateral_acc_lim_filter)
                        : std::vector<double>{};

  const auto lateral_acceleration_velocity_square_ratio_limits =
    computeLateralAccelerationVelocitySquareRatioLimits();

  for (size_t i = 0; i < output.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > after_decel_index ? i - after_decel_index : 0;
    const size_t end = std::min(output.size(), i + before_decel_index + 1);
    for (size_t j = start; j < end; ++j) {
      if (j >= curvature_v.size()) return output;
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = computeVelocityLimitFromLateralAcc(
      curvature, lateral_acceleration_velocity_square_ratio_limits);
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);

    if (enable_smooth_limit) {
      if (i >= latacc_min_vel_arr.size()) return output;
      v_curvature_max = std::max(v_curvature_max, latacc_min_vel_arr.at(i));
    }
    if (output.at(i).longitudinal_velocity_mps > v_curvature_max) {
      output.at(i).longitudinal_velocity_mps = v_curvature_max;
    }
  }
  return output;
}

// 根据转向角速度限制来降低轨迹速度。它不是直接平滑纵向速度，而是先根据路径曲率变化，判断“如果按当前速度走这段轨迹，前轮转角变化会不会太快”，如果太快，就把对应轨迹点速度压低。
TrajectoryPoints SmootherBase::applySteeringRateLimit(
  const TrajectoryPoints & input, const bool use_resampling,
  const double input_points_interval) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input.size() < 3) {
    return input;  // cannot calculate the desired velocity. do nothing.
  }

  const auto steer_rate_velocity_ratio_limits = computeSteerRateVelocityRatioLimits(); //在不同速度区间下，允许的最大转角空间变化率 dδ/ds。与横向速度一样的逻辑，不同速度区间给出不同的转角的最大变化率。然后求出曲率区间来给出后续的速度限制

  // Interpolate with constant interval distance for lateral acceleration calculation.
  // 如果 use_resampling = true，会按固定距离间隔重采样，方便计算转角变化率
  const double points_interval = use_resampling ? base_param_.sample_ds : input_points_interval;

  auto output = applyPreProcess(input, points_interval, use_resampling);

  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((base_param_.curvature_calculation_distance) / points_interval), 1));

  // Step1. Calculate curvature assuming the trajectory points interval is constant.
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  // Step2. Calculate steer rate for each trajectory point.
  std::vector<double> steer_rate_velocity_ratio_arr(output.size());
  for (size_t i = 0; i < output.size() - 1; i++) {
    // steer
    auto & steer_front = output.at(i + 1).front_wheel_angle_rad;
    auto & steer_back = output.at(i).front_wheel_angle_rad;

    // calculate the just 2 steering angle
    // 使用简化自行车模型计算转向角：tan(δ) = L / R
    steer_front = std::atan(base_param_.wheel_base * curvature_v.at(i + 1)); //车辆需要的前轮转角
    steer_back = std::atan(base_param_.wheel_base * curvature_v.at(i));

    const auto steering_diff = std::fabs(steer_front - steer_back);

    steer_rate_velocity_ratio_arr.at(i) =
      steering_diff / (points_interval + std::numeric_limits<double>::epsilon());
  }

  steer_rate_velocity_ratio_arr.back() = steer_rate_velocity_ratio_arr.at((output.size() - 2));

  // Step3. Remove noise by mean filter.
  // 简单三点均值滤波，让 dδ/ds 更平滑，避免某一个噪声点导致速度突然被压低
  for (size_t i = 1; i < steer_rate_velocity_ratio_arr.size() - 1; i++) {
    steer_rate_velocity_ratio_arr.at(i) =
      (steer_rate_velocity_ratio_arr.at(i - 1) + steer_rate_velocity_ratio_arr.at(i) +
       steer_rate_velocity_ratio_arr.at(i + 1)) /
      3.0;
  }

  // Step4. Limit velocity by steer rate.
  for (size_t i = 0; i < output.size() - 1; i++) {
    if (fabs(curvature_v.at(i)) < base_param_.curvature_threshold) {
      continue;
    }

    // 转角变化发生在点 i 到 i+1 这段 segment 上，所以用两端点平均速度代表这段的行驶速度。
    const auto mean_vel =
      (output.at(i).longitudinal_velocity_mps + output.at(i + 1).longitudinal_velocity_mps) / 2.0;

    const auto local_velocity_limit = computeVelocityLimitFromSteerRate(
      steer_rate_velocity_ratio_arr.at(i), steer_rate_velocity_ratio_limits); // 与横向加速度类似

    if (mean_vel < local_velocity_limit) {
      continue;
    }

    for (size_t k = 0; k < 2; k++) {
      auto & velocity = output.at(i + k).longitudinal_velocity_mps;
      const float target_velocity = std::max(
        base_param_.min_curve_velocity,
        std::min(local_velocity_limit, velocity * (local_velocity_limit / mean_vel))); // 不是直接把两个点都设成 local_velocity_limit，而是按比例缩放。这样两点速度形状大致保留，只是整体降低。
      velocity = std::min(velocity, target_velocity);
    }
  }

  return output;
}

}  // namespace autoware::velocity_smoother
