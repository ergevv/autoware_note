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
/**
 * @brief 构造轨迹二阶差分平滑项对应的 P 矩阵。
 *
 * @details
 * 该矩阵用于惩罚轨迹点的二阶差分，也就是惩罚局部弯折和抖动。设 N = num_points，
 * 单个坐标轴上的点列为
 *
 *   x = [x_0, x_1, ..., x_{N-1}]^T
 *
 * 二阶差分矩阵 D 的每一行只作用在连续三个点上：
 *
 *   D_i * x = x_i - 2 * x_{i+1} + x_{i+2},  i = 0, ..., N - 3
 *
 * 因此平滑代价可写成
 *
 *   sum_i (x_i - 2*x_{i+1} + x_{i+2})^2 = x^T * D^T * D * x
 *
 * D^T * D 的非零系数具有固定带状结构：
 * - 主对角线：边界点为 1，边界相邻点为 5，中间点为 6。
 * - 一阶邻接对角线：靠边界的相邻项为 -2，其余相邻项为 -4。
 * - 二阶邻接对角线：系数为 1。
 *
 * 轨迹包含 x/y 两个坐标轴，所以最终返回的是 blockdiag(D^T*D, D^T*D)，
 * 维度为 2N x 2N，前 N 维对应 x 坐标，后 N 维对应 y 坐标。
 *
 * @param num_points 参与优化的轨迹点数量 N。
 * @return Eigen::SparseMatrix<double> 用于全局 x/y 坐标平滑代价的稀疏 P 矩阵。
 * 
 * 
  D =
  [ 1 -2  1  0  0
    0  1 -2  1  0
    0  0  1 -2  1 ]

  D^T D =
  [ 1 -2  1  0  0
  -2  5 -4  1  0
    1 -4  6 -4  1
    0  1 -4  5 -2
    0  0  1 -2  1 ]
 */
Eigen::SparseMatrix<double> makePMatrix(const int num_points)
{
  std::vector<Eigen::Triplet<double>> triplet_vec;
  // 同一个系数需要分别写入 x 坐标块和 y 坐标块，形成 blockdiag(D^T*D, D^T*D)。
  const auto assign_value_to_triplet_vec =
    [&](const double row, const double column, const double value) {
      triplet_vec.push_back(Eigen::Triplet<double>(row, column, value));
      triplet_vec.push_back(Eigen::Triplet<double>(row + num_points, column + num_points, value));
    };

  for (int r = 0; r < num_points; ++r) {
    for (int c = 0; c < num_points; ++c) {
      if (r == c) {
        // 主对角线来自所有包含该点的二阶差分项系数平方和。
        if (r == 0 || r == num_points - 1) {
          // 首尾点只出现在一个二阶差分项中：1^2 = 1。
          assign_value_to_triplet_vec(r, c, 1.0);
        } else if (r == 1 || r == num_points - 2) {
          // 靠近边界的点出现在两个项中：(-2)^2 + 1^2 = 5。
          assign_value_to_triplet_vec(r, c, 5.0);
        } else {
          // 中间点出现在三个项中：1^2 + (-2)^2 + 1^2 = 6。
          assign_value_to_triplet_vec(r, c, 6.0);
        }
      } else if (std::abs(c - r) == 1) {
        // 一阶邻接项来自相邻两点在同一个二阶差分项中的交叉乘积。
        if (r == 0 || r == num_points - 1 || c == 0 || c == num_points - 1) {
          // 靠边界只共享一个项：1 * (-2) = -2。
          assign_value_to_triplet_vec(r, c, -2.0);
        } else {
          // 内部相邻点共享两个项：1 * (-2) + (-2) * 1 = -4。
          assign_value_to_triplet_vec(r, c, -4.0);
        }
      } else if (std::abs(c - r) == 2) {
        // 二阶邻接点只会在一个二阶差分项中同时出现：1 * 1 = 1。
        assign_value_to_triplet_vec(r, c, 1.0);
      } else {
        // 更远的点不会出现在同一个二阶差分项中，对应系数为 0。
        assign_value_to_triplet_vec(r, c, 0.0);
      }
    }
  }

  // 将三元组形式写入 Eigen 稀疏矩阵，供后续 QP 目标函数构造使用。
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
 * @brief 使用 Elastic Band 算法对自车附近的局部轨迹进行几何平滑。
 *
 * 该函数只负责局部轨迹的横向几何优化，不在这里重新生成速度，也不拼接完整输出路径。
 * 上层 ElasticBandSmoother::onPath() 会在本函数返回后恢复速度分布并接回后续原始路径。
 *
 * 主要流程包括：裁剪自车附近轨迹、用上一轮结果固定前端、按固定弧长重采样、
 * 将轨迹补齐到固定优化维度、构建并求解 Elastic Band 的 QP 问题、再把优化变量转换回轨迹点。
 *
 * @param traj_points 输入的原始轨迹点序列
 * @param ego_pose 自车当前位置姿态
 * @return std::vector<TrajectoryPoint> 平滑后的局部轨迹；如果优化失败则回退到上一轮成功结果
 */
std::vector<TrajectoryPoint> EBPathSmoother::smoothTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  // 统计 Elastic Band 核心平滑流程的耗时，供上层 debug 计时日志使用。
  time_keeper_ptr_->tic(__func__);

  // 定义优化失败时的回退策略：优先使用上一轮成功的 EB 轨迹，首次失败则退回原始输入。
  const auto get_prev_eb_traj_points = [&]() {
    if (prev_eb_traj_points_ptr_) {
      return *prev_eb_traj_points_ptr_;
    }
    return traj_points;
  };

  // 1. 裁剪轨迹：只优化自车附近的一小段路径，避免把完整长路径放进 QP 求解器。
  // 前向长度由优化点数和采样间隔决定，后向长度来自通用输出参数。
  const double forward_traj_length = eb_param_.num_points * eb_param_.delta_arc_length;
  const double backward_traj_length = common_param_.output_backward_traj_length;

  // 先找到自车所在路径段，再基于该段按弧长裁剪前后轨迹点。
  const size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, ego_pose, ego_nearest_param_);
  const auto cropped_traj_points = autoware::motion_utils::cropPoints(
    traj_points, ego_pose.position, ego_seg_idx, forward_traj_length, backward_traj_length);

  // 判断裁剪后的局部轨迹是否已经包含全局目标点；包含时后续会收紧终点附近约束。
  const bool is_goal_contained =
    geometry_utils::isSamePoint(cropped_traj_points.back(), traj_points.back());

  // 2. 插入固定点。
  // NOTE: 必须在裁剪之后执行，否则新插入的固定点可能又被裁剪掉。
  // 固定点来自上一轮优化结果，用来减少连续规划周期之间的轨迹跳变。
  const auto traj_points_with_fixed_point = insertFixedPoint(cropped_traj_points);

  // 3. 按 delta_arc_length 重采样轨迹。
  // 固定弧长间隔能让平滑代价矩阵对应更稳定的离散几何关系。
  const auto resampled_traj_points = [&]() {
    // NOTE: 如果点间距不均匀，优化有时会变得不稳定。
    //       这里不重采样停车点，避免改变输入规划里显式给出的停车位置。
    auto tmp_traj_points = trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
      traj_points_with_fixed_point, eb_param_.delta_arc_length);

    // NOTE: 首点可能来自上一轮优化结果，其余点来自本轮输入路径。
    //       二者之间可能存在横向误差，直接用重采样结果的首点姿态会导致朝向异常。
    //       因此重采样后显式恢复首点 pose，保持前端连接稳定。
    tmp_traj_points.front().pose = traj_points_with_fixed_point.front().pose;
    return tmp_traj_points;
  }();

  // 4. 补齐轨迹点。
  // QP 的变量维度固定为 eb_param_.num_points；当局部轨迹点不足时用末尾点补齐。
  // pad_start_idx 记录真实轨迹点数量，后续转换结果时只取真实部分。
  const auto [padded_traj_points, pad_start_idx] = getPaddedTrajectoryPoints(resampled_traj_points);

  // 5. 更新 Elastic Band 的 QP 约束和目标函数。
  // 每个点的优化变量是沿该点法向的横向偏移量，约束边界决定该点允许被拉动的范围。
  updateConstraint(padded_traj_points, is_goal_contained, pad_start_idx);

  // 6. 求解 QP，得到每个采样点的横向偏移量。
  const auto optimized_points = calcSmoothedTrajectory();
  if (!optimized_points) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since smoothing failed");
    // 求解失败时不发布不可信的新轨迹，回退到上一轮成功结果以保证输出连续。
    return get_prev_eb_traj_points();
  }

  // 7. 将横向偏移量应用回轨迹点，并重新计算轨迹朝向。
  const auto eb_traj_points =
    convertOptimizedPointsToTrajectory(*optimized_points, padded_traj_points, pad_start_idx);
  if (!eb_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since x or y error is too large");
    // 偏移量超过验证阈值时认为优化结果不可靠，同样回退到上一轮结果。
    return get_prev_eb_traj_points();
  }

  // 保存本轮成功结果，供下一轮固定前端以及失败回退使用。
  prev_eb_traj_points_ptr_ = std::make_shared<std::vector<TrajectoryPoint>>(*eb_traj_points);

  // 8. 发布调试用的 EB 局部轨迹，方便在可视化工具中检查优化结果。
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
 * @brief 构建 Elastic Band 的 QP 问题，并把约束和目标函数写入 OSQP 求解器。
 *
 * @details
 * 该函数不直接修改轨迹点，而是把局部路径平滑问题整理成 OSQP 的标准二次规划：
 *
 *   minimize    0.5 * d^T * P * d + q^T * d
 *   subject to  lower_bound <= A * d <= upper_bound
 *
 * @par 1. 优化变量和几何模型
 * 令 N = p.num_points。第 i 个参考轨迹点写为
 *
 *   r_i = [x_i, y_i]^T,    i = 0, ..., N - 1
 *
 * 其航向角为 yaw_i。沿轨迹左侧的单位法向量为
 *
 *   n_i = [-sin(yaw_i), cos(yaw_i)]^T
 *
 * Elastic Band 只优化每个点沿该法向的有符号横向偏移 d_i：
 *
 *   d = [d_0, d_1, ..., d_{N-1}]^T
 *
 * 因此优化后的第 i 个点为
 *
 *   r_i'(d_i) = r_i + n_i * d_i
 *
 * 展开到 x/y 坐标就是
 *
 *   x_i' = x_i - sin(yaw_i) * d_i
 *   y_i' = y_i + cos(yaw_i) * d_i
 *
 * @par 2. x_mat 和 sparse_theta_mat 的构造
 * 代码中的 x_mat 把所有原始点坐标堆成一个 2N 维列向量：
 *
 *   x_mat = [x_0, ..., x_{N-1}, y_0, ..., y_{N-1}]^T
 *
 * sparse_theta_mat 记为 Θ，维度为 N x 2N。它的第 i 行只有两个非零元素：
 *
 *   Θ(i, i)     = -sin(yaw_i)
 *   Θ(i, i + N) =  cos(yaw_i)
 *
 * 所以 Θ^T * d 是所有点横向偏移在全局 x/y 坐标中的增量：
 *
 *   Θ^T d =
 *     [-sin(yaw_0)d_0, ..., -sin(yaw_{N-1})d_{N-1},
 *       cos(yaw_0)d_0, ...,  cos(yaw_{N-1})d_{N-1}]^T
 *
 * 全部优化后坐标可统一写成
 *
 *   x_smooth(d) = x_mat + Θ^T * d
 *
 * @par 3. 约束 A、lower_bound、upper_bound
 * 代码中 A = I_N，因此 OSQP 约束
 *
 *   lower_bound <= A * d <= upper_bound
 *
 * 等价于逐点的横向偏移约束
 *
 *   -c_i <= d_i <= c_i
 *
 * c_i 由点的位置决定：
 *
 *   c_i = clearance_for_fix    固定点、目标点及其前一点
 *   c_i = clearance_for_joint  前端连接区域
 *   c_i = clearance_for_smooth 普通平滑区域
 *
 * 当 c_i = 0 时，该点完全固定，debug_fixed_traj_points 会记录这些点用于调试显示。
 *
 * @par 4. 平滑代价 P_for_smooth 和 q 的推导
 * makePMatrix(N) 构造的是全局 x/y 坐标下的二阶差分矩阵 M。它等价于
 * blockdiag(D^T D, D^T D)，其中 D 的每一行对应相邻三点的二阶差分：
 *
 *   [1, -2, 1]
 *
 * 因此二阶差分代价近似为
 *
 *   sum ||r_i - 2 * r_{i+1} + r_{i+2}||^2
 *
 * 令
 *
 *   R = smooth_weight * M
 *
 * 平滑代价写成优化变量 d 的函数：
 *
 *   J_smooth(d)
 *     = 0.5 * x_smooth(d)^T * R * x_smooth(d)
 *     = 0.5 * (x_mat + Θ^T d)^T * R * (x_mat + Θ^T d)
 *
 * 展开：
 *
 *   J_smooth(d)
 *     = 0.5 * d^T * Θ * R * Θ^T * d
 *       + d^T * Θ * R * x_mat
 *       + 0.5 * x_mat^T * R * x_mat
 *
 * 最后一项与 d 无关，不影响最优解，所以丢弃。由此得到代码中的：
 *
 *   raw_P_for_smooth = R
 *   theta_P_mat      = Θ * R
 *   P_for_smooth     = Θ * R * Θ^T
 *   q                = Θ * R * x_mat
 *
 * @par 5. 横向偏移代价和最终 P/q
 * 为了避免轨迹为了追求平滑而离原始路径过远，代码还加入横向偏移惩罚：
 *
 *   J_lat(d) = 0.5 * d^T * (lat_error_weight * I_N) * d
 *
 * 因此
 *
 *   P_for_lat_error = lat_error_weight * I_N
 *   P               = P_for_smooth + P_for_lat_error
 *   q               = Θ * R * x_mat
 *
 * 这些 P、q、A、lower_bound、upper_bound 就是本函数最终写入 OSQP 的内容。
 *
 * @param traj_points 已裁剪、重采样并补齐后的轨迹点，用于提取参考位置和航向角。
 * @param is_goal_contained 局部轨迹是否包含目标点；包含时会收紧终点附近约束。
 * @param pad_start_idx 补齐前的真实轨迹点数量，用于判断末尾真实点和补齐点的位置。
 */
void EBPathSmoother::updateConstraint(
  const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained,
  const int pad_start_idx)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = eb_param_;

  // 记录被完全固定的轨迹点，仅用于发布 debug 轨迹。
  std::vector<TrajectoryPoint> debug_fixed_traj_points;

  // A 为单位矩阵，所以 QP 约束直接退化为每个横向偏移量 d_i 的上下界。
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(p.num_points, p.num_points);
  std::vector<double> upper_bound(p.num_points, 0.0);
  std::vector<double> lower_bound(p.num_points, 0.0);

  // 根据点在局部轨迹中的位置类型，设置不同的横向可移动范围。
  for (size_t i = 0; i < static_cast<size_t>(p.num_points); ++i) {
    const double constraint_segment_length = [&]() {
      if (i == 0) {
        // NOTE: 只有第一个点可以强约束为固定点。
        //       首点可能来自上一轮优化结果，其余点来自本轮输入路径，
        //       二者之间可能存在横向偏差，固定更多前端点反而会造成不连续。
        return p.clearance_for_fix;
      }
      if (is_goal_contained) {
        // NOTE: 局部轨迹包含目标点时，固定目标点和其前一个真实点以保持目标姿态。
        if (p.num_points - 2 <= static_cast<int>(i) || pad_start_idx - 2 <= static_cast<int>(i)) {
          return p.clearance_for_fix;
        }
      }
      if (i < static_cast<size_t>(p.num_joint_points) + 1) {
        // 前端连接区域使用较小的允许偏移，+1 是因为 index 0 已作为固定点处理。
        return p.clearance_for_joint;
      }
      // 普通平滑区域允许更自由的横向调整，以便 Elastic Band 拉直局部形状。
      return p.clearance_for_smooth;
    }();

    // 对每个优化变量 d_i 设置对称边界：-clearance <= d_i <= clearance。
    upper_bound.at(i) = constraint_segment_length;
    lower_bound.at(i) = -constraint_segment_length;

    // clearance 为 0 表示该点完全固定，收集出来方便在调试轨迹中可视化。
    if (constraint_segment_length == 0.0) {
      debug_fixed_traj_points.push_back(traj_points.at(i));
    }
  }

  // 构建原始坐标向量 x_mat = [x_0 ... x_n, y_0 ... y_n]^T。
  Eigen::VectorXd x_mat(2 * p.num_points);
  // sparse_theta_mat 将全局 XY 坐标投影到每个点的横向法向方向。
  // 第 i 行为 [-sin(yaw_i), cos(yaw_i)]，对应轨迹左法向。
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

  // 从数学概念看，QP 的目标函数就是多维二次函数：
  //   f(d) = 0.5 * d^T * P * d + q^T * d
  // 类似一维函数 f(d) = 0.5 * P * d^2 + q * d，其中 P 决定曲率，q 决定线性拉力。
  //
  // 这里真正想惩罚的是平滑后的全局坐标 x_smooth 的二阶差分：
  //   J_smooth = 0.5 * x_smooth^T * R * x_smooth
  // 其中：
  //   R = smooth_weight * makePMatrix(N)，维度为 2N x 2N
  //   x_smooth = x_mat + Θ^T * d
  //   Θ = sparse_theta_mat，维度为 N x 2N
  //
  // 将 x_smooth 代入并展开：
  //   J_smooth
  //     = 0.5 * (x_mat + Θ^T d)^T * R * (x_mat + Θ^T d)
  //     = 0.5 * d^T * Θ * R * Θ^T * d
  //       + d^T * Θ * R * x_mat
  //       + 0.5 * x_mat^T * R * x_mat
  //
  // 最后一项不含优化变量 d，对最优解没有影响，所以不传给 OSQP。
  // 因此与 OSQP 标准形式对应为：
  //   P_for_smooth = Θ * R * Θ^T
  //   q            = Θ * R * x_mat
  const Eigen::SparseMatrix<double> raw_P_for_smooth = p.smooth_weight * makePMatrix(p.num_points);

  // theta_P_mat = Θ * R，维度为 N x 2N；先缓存这个中间量，后面 P 和 q 都会用到。
  const Eigen::MatrixXd theta_P_mat = sparse_theta_mat * raw_P_for_smooth;

  // P_for_smooth = Θ * R * Θ^T，维度为 N x N，是横向偏移变量 d 的平滑二次项。
  const Eigen::MatrixXd P_for_smooth = theta_P_mat * sparse_theta_mat.transpose();

  // 横向误差代价 J_lat = 0.5 * lat_error_weight * d^T * d。
  // 这相当于给 P 增加 lat_error_weight * I，抑制过大的横向偏移。
  const Eigen::MatrixXd P_for_lat_error =
    p.lat_error_weight * Eigen::MatrixXd::Identity(p.num_points, p.num_points);

  // 最终 P 同时包含“轨迹要平滑”和“不要偏离原始路径太远”这两个二次代价。
  const Eigen::MatrixXd P = P_for_smooth + P_for_lat_error;

  // q = Θ * R * x_mat，维度为 N。
  // 它来自平滑代价展开后的交叉项 d^T * Θ * R * x_mat。
  const Eigen::VectorXd raw_q_for_smooth = theta_P_mat * x_mat;
  const auto q = toStdVector(raw_q_for_smooth);

  // 若启用 warm start 且已有求解器，则只更新矩阵和边界，复用上一轮求解状态。
  if (p.enable_warm_start && osqp_solver_ptr_) {
    osqp_solver_ptr_->updateP(P);
    osqp_solver_ptr_->updateQ(q);
    osqp_solver_ptr_->updateA(A);
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
  } else {
    // 首次运行或未启用 warm start 时，重新创建 OSQP 求解器并配置收敛参数。
    osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(
      P, A, q, lower_bound, upper_bound, p.qp_param.eps_abs);
    osqp_solver_ptr_->updateEpsRel(p.qp_param.eps_rel);
    osqp_solver_ptr_->updateEpsAbs(p.qp_param.eps_abs);
    osqp_solver_ptr_->updateMaxIter(p.qp_param.max_iteration);
  }

  // 发布被完全固定的点，便于检查 clearance_for_fix 是否符合预期。
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
