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

#include "autoware/path_smoother/elastic_band.hpp"

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_smoother/type_alias.hpp"
#include "autoware/path_smoother/utils/geometry_utils.hpp"
#include "autoware/path_smoother/utils/trajectory_utils.hpp"
#include "tf2/utils.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

namespace
{
Eigen::SparseMatrix<double> makePMatrix(const int num_points)
{
  std::vector<Eigen::Triplet<double>> triplet_vec;
  const auto assign_value_to_triplet_vec =
    [&](const double row, const double column, const double value) {
      triplet_vec.push_back(Eigen::Triplet<double>(row, column, value));
      triplet_vec.push_back(Eigen::Triplet<double>(row + num_points, column + num_points, value));
    };

  for (int r = 0; r < num_points; ++r) {
    for (int c = 0; c < num_points; ++c) {
      if (r == c) {
        if (r == 0 || r == num_points - 1) {
          assign_value_to_triplet_vec(r, c, 1.0);
        } else if (r == 1 || r == num_points - 2) {
          assign_value_to_triplet_vec(r, c, 5.0);
        } else {
          assign_value_to_triplet_vec(r, c, 6.0);
        }
      } else if (std::abs(c - r) == 1) {
        if (r == 0 || r == num_points - 1 || c == 0 || c == num_points - 1) {
          assign_value_to_triplet_vec(r, c, -2.0);
        } else {
          assign_value_to_triplet_vec(r, c, -4.0);
        }
      } else if (std::abs(c - r) == 2) {
        assign_value_to_triplet_vec(r, c, 1.0);
      } else {
        assign_value_to_triplet_vec(r, c, 0.0);
      }
    }
  }

  Eigen::SparseMatrix<double> sparse_mat(num_points * 2, num_points * 2);
  sparse_mat.setFromTriplets(triplet_vec.begin(), triplet_vec.end());
  return sparse_mat;
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

std_msgs::msg::Header createHeader(const rclcpp::Time & now)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = now;
  return header;
}
}  // namespace

namespace autoware::path_smoother
{
EBPathSmoother::EBParam::EBParam(rclcpp::Node * node)
{
  {  // option
    enable_warm_start = node->declare_parameter<bool>("elastic_band.option.enable_warm_start");
    enable_optimization_validation =
      node->declare_parameter<bool>("elastic_band.option.enable_optimization_validation");
  }

  {  // common
    delta_arc_length = node->declare_parameter<double>("elastic_band.common.delta_arc_length");
    num_points = node->declare_parameter<int>("elastic_band.common.num_points");
  }

  {  // clearance
    num_joint_points = node->declare_parameter<int>("elastic_band.clearance.num_joint_points");
    clearance_for_fix = node->declare_parameter<double>("elastic_band.clearance.clearance_for_fix");
    clearance_for_joint =
      node->declare_parameter<double>("elastic_band.clearance.clearance_for_joint");
    clearance_for_smooth =
      node->declare_parameter<double>("elastic_band.clearance.clearance_for_smooth");
  }

  {  // weight
    smooth_weight = node->declare_parameter<double>("elastic_band.weight.smooth_weight");
    lat_error_weight = node->declare_parameter<double>("elastic_band.weight.lat_error_weight");
  }

  {  // qp
    qp_param.max_iteration = node->declare_parameter<int>("elastic_band.qp.max_iteration");
    qp_param.eps_abs = node->declare_parameter<double>("elastic_band.qp.eps_abs");
    qp_param.eps_rel = node->declare_parameter<double>("elastic_band.qp.eps_rel");
  }

  // validation
  max_validation_error = node->declare_parameter<double>("elastic_band.validation.max_error");
}

void EBPathSmoother::EBParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  {  // option
    update_param<bool>(parameters, "elastic_band.option.enable_warm_start", enable_warm_start);
    update_param<bool>(
      parameters, "elastic_band.option.enable_optimization_validation",
      enable_optimization_validation);
  }

  {  // common
    update_param<double>(parameters, "elastic_band.common.delta_arc_length", delta_arc_length);
    update_param<int>(parameters, "elastic_band.common.num_points", num_points);
  }

  {  // clearance
    update_param<int>(parameters, "elastic_band.clearance.num_joint_points", num_joint_points);
    update_param<double>(parameters, "elastic_band.clearance.clearance_for_fix", clearance_for_fix);
    update_param<double>(
      parameters, "elastic_band.clearance.clearance_for_joint", clearance_for_joint);
    update_param<double>(
      parameters, "elastic_band.clearance.clearance_for_smooth", clearance_for_smooth);
  }

  {  // weight
    update_param<double>(parameters, "elastic_band.weight.smooth_weight", smooth_weight);
    update_param<double>(parameters, "elastic_band.weight.lat_error_weight", lat_error_weight);
  }

  {  // qp
    update_param<int>(parameters, "elastic_band.qp.max_iteration", qp_param.max_iteration);
    update_param<double>(parameters, "elastic_band.qp.eps_abs", qp_param.eps_abs);
    update_param<double>(parameters, "elastic_band.qp.eps_rel", qp_param.eps_rel);
  }
}

EBPathSmoother::EBPathSmoother(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const CommonParam & common_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  common_param_(common_param),
  time_keeper_ptr_(time_keeper_ptr),
  logger_(node->get_logger().get_child("elastic_band_smoother")),
  clock_(*node->get_clock())
{
  // eb param
  eb_param_ = EBParam(node);

  // publisher
  debug_eb_traj_pub_ = node->create_publisher<Trajectory>("~/debug/eb_traj", 1);
  debug_eb_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/eb_fixed_traj", 1);
}

void EBPathSmoother::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  eb_param_.onParam(parameters);
}

void EBPathSmoother::initialize(const bool enable_debug_info, const CommonParam & common_param)
{
  enable_debug_info_ = enable_debug_info;
  common_param_ = common_param;
}

void EBPathSmoother::resetPreviousData()
{
  prev_eb_traj_points_ptr_ = nullptr;
}

/**
 * @brief 使用弹性带算法平滑轨迹
 * 
 * 该函数通过弹性带(Elastic Band)优化算法对输入轨迹进行平滑处理，主要步骤包括：
 * 裁剪轨迹、插入固定点、重采样、填充边界点、更新QP约束、执行优化以及结果转换。
 * 
 * @param traj_points 输入的原始轨迹点序列
 * @param ego_pose 自车当前位置姿态
 * @return std::vector<TrajectoryPoint> 平滑后的轨迹点序列，如果优化失败则返回上一次的成功结果
 */
std::vector<TrajectoryPoint> EBPathSmoother::smoothTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  time_keeper_ptr_->tic(__func__);

  // 定义获取上一次弹性带轨迹的辅助函数，用于优化失败时的回退
  const auto get_prev_eb_traj_points = [&]() {
    if (prev_eb_traj_points_ptr_) {
      return *prev_eb_traj_points_ptr_;
    }
    return traj_points;
  };

  // 1. crop trajectory
  // 根据自车位置裁剪轨迹，保留前方和后方指定长度的轨迹段
  const double forward_traj_length = eb_param_.num_points * eb_param_.delta_arc_length;
  const double backward_traj_length = common_param_.output_backward_traj_length;

  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = autoware::motion_utils::cropPoints(
    traj_points, ego_pose.position, ego_seg_idx, forward_traj_length, backward_traj_length);

  // check if goal is contained in cropped_traj_points
  const bool is_goal_contained =
    geometry_utils::isSamePoint(cropped_traj_points.back(), traj_points.back());

  // 2. insert fixed point
  // NOTE: This should be after cropping trajectory so that fixed point will not be cropped.
  const auto traj_points_with_fixed_point = insertFixedPoint(cropped_traj_points);

  // 3. resample trajectory with delta_arc_length
  // 以固定弧长间隔重采样轨迹点，确保优化过程的数值稳定性
  const auto resampled_traj_points = [&]() {
    // NOTE: If the interval of points is not constant, the optimization is sometimes unstable.
    //       Therefore, we do not resample a stop point here.
    auto tmp_traj_points = trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
      traj_points_with_fixed_point, eb_param_.delta_arc_length);

    // NOTE: The front point is previous optimized one, and the others are the input ones.
    //       There may be a lateral error between the points, which makes orientation unexpected.
    //       Therefore, the front pose is updated after resample.
    tmp_traj_points.front().pose = traj_points_with_fixed_point.front().pose;
    return tmp_traj_points;
  }();

  // 4. pad trajectory points
  // 在轨迹两端填充额外的点，为弹性带优化提供边界约束
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 5. update constraint for elastic band's QP
  // 根据轨迹点和障碍物信息更新二次规划(QP)问题的约束条件
  updateConstraint(padded_traj_points, is_goal_contained, pad_start_idx);

  // 6. get optimization result
  // 执行弹性带优化算法，求解平滑后的轨迹
  const auto optimized_points = calcSmoothedTrajectory();
  if (!optimized_points) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since smoothing failed");
    return get_prev_eb_traj_points();
  }

  // 7. convert optimization result to trajectory
  // 将优化结果转换为轨迹点格式，并验证结果的合理性
  const auto eb_traj_points =
    convertOptimizedPointsToTrajectory(*optimized_points, padded_traj_points, pad_start_idx);
  if (!eb_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since x or y error is too large");
    return get_prev_eb_traj_points();
  }

  prev_eb_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(*eb_traj_points);

  // 8. publish eb trajectory
  // 发布调试用的弹性带轨迹，用于可视化和调试
  const auto eb_traj =
    autoware::motion_utils::convertToTrajectory(*eb_traj_points, createHeader(clock_.now()));
  debug_eb_traj_pub_->publish(eb_traj);

  time_keeper_ptr_->toc(__func__, "      ");
  return *eb_traj_points;
}

std::vector<TrajectoryPoint> EBPathSmoother::insertFixedPoint(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  if (!prev_eb_traj_points_ptr_) {
    return traj_points;
  }

  auto traj_points_with_fixed_point = traj_points;
  // replace the front pose with previous points
  trajectory_utils::updateFrontPointForFix(
    traj_points_with_fixed_point, *prev_eb_traj_points_ptr_, eb_param_.delta_arc_length,
    ego_nearest_param_);

  time_keeper_ptr_->toc(__func__, "        ");
  return traj_points_with_fixed_point;
}

std::tuple<std::vector<TrajectoryPoint>, size_t> EBPathSmoother::getPaddedTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t pad_start_idx =
    std::min(static_cast<size_t>(eb_param_.num_points), traj_points.size());

  std::vector<TrajectoryPoint> padded_traj_points;
  for (size_t i = 0; i < static_cast<size_t>(eb_param_.num_points); ++i) {
    const size_t point_idx = i < pad_start_idx ? i : pad_start_idx - 1;
    padded_traj_points.push_back(traj_points.at(point_idx));
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return {padded_traj_points, pad_start_idx};
}

/**
 * @brief 更新弹性带优化问题的约束条件并配置OSQP求解器
 * 
 * 该函数根据输入轨迹点计算二次规划(QP)问题的约束边界，构建目标函数的P矩阵和q向量，
 * 并初始化或更新OSQP求解器。主要功能包括：
 * - 根据不同位置点设置不同的约束边界（固定点、连接点、平滑点）
 * - 构建基于航向角的坐标变换矩阵
 * - 计算平滑权重和横向误差权重的组合目标函数
 * - 支持热启动以加速连续优化过程
 * 
 * @param traj_points 输入轨迹点序列，用于提取位置和姿态信息
 * @param is_goal_contained 标识目标点是否包含在轨迹中，影响终点附近的约束设置
 * @param pad_start_idx 填充起始索引，用于确定需要保持固定的轨迹段范围
 */
void EBPathSmoother::updateConstraint(
  const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained,
  const int pad_start_idx)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = eb_param_;

  std::vector<TrajectoryPoint> debug_fixed_traj_points;  // for debug

  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(p.num_points, p.num_points);
  std::vector<double> upper_bound(p.num_points, 0.0);
  std::vector<double> lower_bound(p.num_points, 0.0);
  
  // 根据点的位置类型计算不同的约束边界长度
  for (size_t i = 0; i < static_cast<size_t>(p.num_points); ++i) {
    const double constraint_segment_length = [&]() {
      if (i == 0) {
        // NOTE: Only first point can be fixed since there is a lateral deviation
        //       between the two points.
        //       The front point is previous optimized one, and the others are the input ones.
        return p.clearance_for_fix;
      }
      if (is_goal_contained) {
        // NOTE: fix goal and its previous pose to keep goal orientation
        if (p.num_points - 2 <= static_cast<int>(i) || pad_start_idx - 2 <= static_cast<int>(i)) {
          return p.clearance_for_fix;
        }
      }
      if (i < static_cast<size_t>(p.num_joint_points) + 1) {  // 1 is added since index 0 is fixed
                                                              // point
        return p.clearance_for_joint;
      }
      return p.clearance_for_smooth;
    }();

    upper_bound.at(i) = constraint_segment_length;
    lower_bound.at(i) = -constraint_segment_length;

    if (constraint_segment_length == 0.0) {
      debug_fixed_traj_points.push_back(traj_points.at(i));
    }
  }

  // 构建状态向量x_mat和基于航向角的稀疏变换矩阵sparse_theta_mat
  Eigen::VectorXd x_mat(2 * p.num_points);
  std::vector<Eigen::Triplet<double>> theta_triplet_vec;
  for (size_t i = 0; i < static_cast<size_t>(p.num_points); ++i) {
    x_mat(i) = traj_points.at(i).pose.position.x;
    x_mat(i + p.num_points) = traj_points.at(i).pose.position.y;

    const double yaw = tf2::getYaw(traj_points.at(i).pose.orientation);
    theta_triplet_vec.push_back(Eigen::Triplet<double>(i, i, -std::sin(yaw)));
    theta_triplet_vec.push_back(Eigen::Triplet<double>(i, i + p.num_points, std::cos(yaw)));
  }
  Eigen::SparseMatrix<double> sparse_theta_mat(p.num_points, 2 * p.num_points);
  sparse_theta_mat.setFromTriplets(theta_triplet_vec.begin(), theta_triplet_vec.end());

  // 计算QP问题的目标函数矩阵P，结合平滑权重和横向误差权重
  const Eigen::SparseMatrix<double> raw_P_for_smooth = p.smooth_weight * makePMatrix(p.num_points);
  const Eigen::MatrixXd theta_P_mat = sparse_theta_mat * raw_P_for_smooth;
  const Eigen::MatrixXd P_for_smooth = theta_P_mat * sparse_theta_mat.transpose();
  const Eigen::MatrixXd P_for_lat_error =
    p.lat_error_weight * Eigen::MatrixXd::Identity(p.num_points, p.num_points);
  const Eigen::MatrixXd P = P_for_smooth + P_for_lat_error;

  // 计算QP问题的线性项向量q
  const Eigen::VectorXd raw_q_for_smooth = theta_P_mat * x_mat;
  const auto q = toStdVector(raw_q_for_smooth);

  // 根据热启动配置选择更新现有求解器或创建新求解器
  if (p.enable_warm_start && osqp_solver_ptr_) {
    osqp_solver_ptr_->updateP(P);
    osqp_solver_ptr_->updateQ(q);
    osqp_solver_ptr_->updateA(A);
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
  } else {
    osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(
      P, A, q, lower_bound, upper_bound, p.qp_param.eps_abs);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
    osqp_solver_ptr_->updateEpsAbs(p.qp_param.eps_abs);
    osqp_solver_ptr_->updateMaxIter(p.qp_param.max_iteration);
  }

  // publish fixed trajectory
  const auto eb_fixed_traj = autoware::motion_utils::convertToTrajectory(
    debug_fixed_traj_points, createHeader(clock_.now()));
  debug_eb_fixed_traj_pub_->publish(eb_fixed_traj);

  time_keeper_ptr_->toc(__func__, "        ");
}

std::optional<std::vector<double>> EBPathSmoother::calcSmoothedTrajectory()
{
  time_keeper_ptr_->tic(__func__);

  // solve QP
  const auto result = osqp_solver_ptr_->optimize();
  const auto optimized_points = result.primal_solution;

  const auto status = result.solution_status;

  // check status
  if (status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[EB]");
    return std::nullopt;
  }
  const auto has_nan = std::any_of(
    optimized_points.begin(), optimized_points.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return std::nullopt;
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return optimized_points;
}

std::optional<std::vector<TrajectoryPoint>> EBPathSmoother::convertOptimizedPointsToTrajectory(
  const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
  const int pad_start_idx) const
{
  time_keeper_ptr_->tic(__func__);

  std::vector<TrajectoryPoint> eb_traj_points;

  // update only x and y
  for (size_t i = 0; i < static_cast<size_t>(pad_start_idx); ++i) {
    const double lat_offset = optimized_points.at(i);

    // validate optimization result
    if (eb_param_.enable_optimization_validation) {
      if (eb_param_.max_validation_error < std::abs(lat_offset)) {
        return std::nullopt;
      }
    }

    auto eb_traj_point = traj_points.at(i);
    eb_traj_point.pose = autoware_utils::calc_offset_pose(eb_traj_point.pose, 0.0, lat_offset, 0.0);
    eb_traj_points.push_back(eb_traj_point);
  }

  // update orientation
  autoware::motion_utils::insertOrientation(eb_traj_points, true);

  time_keeper_ptr_->toc(__func__, "        ");
  return eb_traj_points;
}
}  // namespace autoware::path_smoother
