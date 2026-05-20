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

#include "autoware/path_smoother/elastic_band_smoother.hpp"

#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_smoother/utils/geometry_utils.hpp"
#include "autoware/path_smoother/utils/trajectory_utils.hpp"
#include "rclcpp/time.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::path_smoother
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

StringStamped createStringStamped(const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

Float64Stamped createFloat64Stamped(const rclcpp::Time & now, const float & data)
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
}  // namespace

ElasticBandSmoother::ElasticBandSmoother(const rclcpp::NodeOptions & node_options)
: Node("autoware_path_smoother", node_options), time_keeper_ptr_(std::make_shared<TimeKeeper>())
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/traj", 1);
  path_pub_ = create_publisher<Path>("~/output/path", 1);

  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&ElasticBandSmoother::onPath, this, std::placeholders::_1));

  // debug publisher
  debug_extended_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_traj", 1);
  debug_calculation_time_str_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);
  debug_calculation_time_float_pub_ =
    create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  {  // parameters
    // parameters for ego nearest search
    ego_nearest_param_ = EgoNearestParam(this);

    // parameters for trajectory
    common_param_ = CommonParam(this);
  }

  eb_path_smoother_ptr_ = std::make_shared<EBPathSmoother>(
    this, enable_debug_info_, ego_nearest_param_, common_param_, time_keeper_ptr_);
  replan_checker_ptr_ = std::make_shared<ReplanChecker>(this, ego_nearest_param_);

  // reset planners
  initializePlanning();

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ElasticBandSmoother::onParam, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

rcl_interfaces::msg::SetParametersResult ElasticBandSmoother::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  // parameters for ego nearest search
  ego_nearest_param_.onParam(parameters);

  // parameters for trajectory
  common_param_.onParam(parameters);

  // parameters for core algorithms
  eb_path_smoother_ptr_->onParam(parameters);
  replan_checker_ptr_->onParam(parameters);

  // reset planners
  initializePlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ElasticBandSmoother::initializePlanning()
{
  RCLCPP_DEBUG(get_logger(), "Initialize planning");

  eb_path_smoother_ptr_->initialize(false, common_param_);
  resetPreviousData();
}

void ElasticBandSmoother::resetPreviousData()
{
  eb_path_smoother_ptr_->resetPreviousData();

  prev_optimized_traj_points_ptr_ = nullptr;
}

/**
 * @brief 平滑输入路径，并发布轨迹和路径两种输出。
 *
 * 该回调将几何平滑、速度恢复、路径延展和消息发布拆成独立步骤，
 * 让 Elastic Band 优化器只负责局部路径形状优化。
 */
void ElasticBandSmoother::onPath(const Path::ConstSharedPtr path_ptr)
{
  // 为本次回调初始化计时日志，并开始统计整个 onPath 流程耗时。
  time_keeper_ptr_->init();
  time_keeper_ptr_->tic(__func__);

  // 取出最新自车里程计；路径平滑需要当前自车位姿和速度。
  const auto ego_state_ptr = odom_sub_.take_data();
  // 如果必要输入缺失，或路径点/边界信息不足，则提前结束本次处理。
  if (!isDataReady(*path_ptr, ego_state_ptr, *get_clock())) {
    return;
  }

  // 0. 如果输入路径是倒车方向，则不执行 Elastic Band 平滑。
  // TODO(murooka): 支持倒车路径。
  const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
  if (!is_driving_forward) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");

    // 当前 Elastic Band 优化器只适用于前进路径，因此这里直接发布透传结果。
    const auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    const auto output_traj_msg =
      autoware::motion_utils::convertToTrajectory(traj_points, path_ptr->header);
    traj_pub_->publish(output_traj_msg);
    path_pub_->publish(*path_ptr);
    published_time_publisher_->publish_if_subscribed(path_pub_, path_ptr->header.stamp);
    return;
  }

  // 将 PathPoint 转为 TrajectoryPoint，作为平滑器和速度处理工具的统一内部格式。
  const auto input_traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);

  // 1. 使用 Elastic Band 计算平滑轨迹。
  // 1.a 判断是否需要重新规划，也就是是否需要重新执行优化。
  // PlannerData 保存本次输入路径、自车位姿和自车速度，供重规划判断使用。
  PlannerData planner_data(
    input_traj_points, ego_state_ptr->pose.pose, ego_state_ptr->twist.twist.linear.x);
  const bool is_replan_required = [&]() {
    if (replan_checker_ptr_->isResetRequired(planner_data)) {
      // NOTE: 重置上一轮优化结果时，必须重新执行优化。
      resetPreviousData();
      return true;
    }
    // 如果路径或时间变化不大，则复用上一轮优化得到的几何形状。
    return !prev_optimized_traj_points_ptr_ ||
           replan_checker_ptr_->isReplanRequired(planner_data, now());
  }();
  // 保存当前输入；如果本轮重新优化，也同步更新上次重规划时间。
  replan_checker_ptr_->updateData(planner_data, is_replan_required, now());

  // 单独统计局部平滑耗时，便于和整个回调耗时区分。
  time_keeper_ptr_->tic(__func__);
  // 只有需要重规划时才运行 Elastic Band，否则直接复用缓存的优化结果。
  auto smoothed_traj_points = is_replan_required ? eb_path_smoother_ptr_->smoothTrajectory(
                                                     input_traj_points, ego_state_ptr->pose.pose)
                                                 : *prev_optimized_traj_points_ptr_;
  time_keeper_ptr_->toc(__func__, "    ");

  // 先缓存局部优化后的几何形状，后续速度恢复和路径延展不会影响该缓存语义。
  prev_optimized_traj_points_ptr_ =
    std::make_shared<std::vector<TrajectoryPoint>>(smoothed_traj_points);

  // 2. 将输入路径的速度分布恢复到平滑后的几何轨迹上。
  applyInputVelocity(smoothed_traj_points, input_traj_points, ego_state_ptr->pose.pose);

  // 3. 将优化后的局部轨迹与后续原始路径平滑连接，形成完整输出。
  auto full_traj_points = extendTrajectory(input_traj_points, smoothed_traj_points);

  // 4. 即使经过插值和重采样，也保证第一个停车点之后的速度全部为 0。
  setZeroVelocityAfterStopPoint(full_traj_points);

  time_keeper_ptr_->toc(__func__, "");
  *time_keeper_ptr_ << "========================================";
  time_keeper_ptr_->endLine();

  // 发布计算耗时。
  // NOTE: 必须在 onPath 总耗时统计结束后再生成该调试消息。
  const auto calculation_time_msg = createStringStamped(now(), time_keeper_ptr_->getLog());
  debug_calculation_time_str_pub_->publish(calculation_time_msg);
  debug_calculation_time_float_pub_->publish(
    createFloat64Stamped(now(), time_keeper_ptr_->getAccumulatedTime()));

  // 使用原始 header 发布平滑后的轨迹，以及由该轨迹生成的 Path 消息。
  const auto output_traj_msg =
    autoware::motion_utils::convertToTrajectory(full_traj_points, path_ptr->header);
  traj_pub_->publish(output_traj_msg);
  const auto output_path_msg = trajectory_utils::create_path(*path_ptr, full_traj_points);
  path_pub_->publish(output_path_msg);
  published_time_publisher_->publish_if_subscribed(path_pub_, path_ptr->header.stamp);
}

bool ElasticBandSmoother::isDataReady(
  const Path & path, const Odometry::ConstSharedPtr ego_state_ptr, rclcpp::Clock clock) const
{
  if (!ego_state_ptr) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Waiting for ego pose and twist.");
    return false;
  }

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

void ElasticBandSmoother::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj_points,
  const std::vector<TrajectoryPoint> & input_traj_points,
  const geometry_msgs::msg::Pose & ego_pose) const
{
  time_keeper_ptr_->tic(__func__);

  // crop forward for faster calculation
  const double output_traj_length = autoware::motion_utils::calcArcLength(output_traj_points);
  constexpr double margin_traj_length = 10.0;
  const auto forward_cropped_input_traj_points = [&]() {
    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_traj_points, ego_pose, ego_nearest_param_);
    return autoware::motion_utils::cropForwardPoints(
      input_traj_points, ego_pose.position, ego_seg_idx, output_traj_length + margin_traj_length);
  }();

  // update velocity
  size_t input_traj_start_idx = 0;
  for (size_t i = 0; i < output_traj_points.size(); i++) {
    // crop backward for efficient calculation
    const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
      forward_cropped_input_traj_points.begin() + input_traj_start_idx,
      forward_cropped_input_traj_points.end()};

    const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
      cropped_input_traj_points, output_traj_points.at(i).pose, ego_nearest_param_);
    input_traj_start_idx = nearest_seg_idx;

    // calculate velocity with zero order hold
    const double velocity = cropped_input_traj_points.at(nearest_seg_idx).longitudinal_velocity_mps;
    output_traj_points.at(i).longitudinal_velocity_mps = velocity;
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

  time_keeper_ptr_->toc(__func__, "    ");
}

std::vector<TrajectoryPoint> ElasticBandSmoother::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  time_keeper_ptr_->tic(__func__);

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
      optimized_traj_points, common_param_.output_delta_arc_length);
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
    full_traj_points, common_param_.output_delta_arc_length);

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

  time_keeper_ptr_->toc(__func__, "  ");
  return resampled_traj_points;
}
}  // namespace autoware::path_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_smoother::ElasticBandSmoother)
