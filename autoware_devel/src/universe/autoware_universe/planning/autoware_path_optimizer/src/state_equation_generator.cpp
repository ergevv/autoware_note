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

#include "autoware/path_optimizer/state_equation_generator.hpp"

#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include <vector>

namespace autoware::path_optimizer
{
/**
 * @brief 计算整个时间序列的状态方程矩阵
 * 
 * 该函数基于参考点序列和车辆运动学模型，构建离散化的状态方程矩阵。
 * 状态方程形式为: x = A*x + B*u + W，其中x为状态向量，u为控制输入向量。
 * 通过将单步状态方程(x_{t+1} = Ad*x_t + Bd*u + Wd)在整个参考点序列上展开，
 * 得到整体的状态转移矩阵A、控制矩阵B和偏移向量W。
 * 
 * @param ref_points 参考点序列，包含路径上的离散化参考点信息（如弧长增量等）
 * @return Matrix 包含三个矩阵的结构体：
 *         - A: 状态转移矩阵 (N_x × N_x)，描述状态之间的转移关系
 *         - B: 控制输入矩阵 (N_x × N_u)，描述控制输入对状态的影响
 *         - W: 偏移向量 (N_x × 1)，包含非线性项和常数项
 * 
 * @note 矩阵维度说明：
 *       - D_x: 状态变量维度（由车辆模型决定）
 *       - D_u: 控制输入维度（由车辆模型决定）
 *       - N_ref: 参考点数量
 *       - N_x = N_ref * D_x: 总状态维度
 *       - N_u = (N_ref - 1) * D_u: 总控制输入维度
 */
StateEquationGenerator::Matrix StateEquationGenerator::calcMatrix(
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // 初始化整体状态方程的矩阵
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_x, N_x);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N_x, N_u);
  Eigen::VectorXd W = Eigen::VectorXd::Zero(N_x);

  // 初始化单步状态方程的临时矩阵
  Eigen::MatrixXd Ad(D_x, D_x);
  Eigen::MatrixXd Bd(D_x, D_u);
  Eigen::MatrixXd Wd(D_x, 1);

  // 设置初始状态的自转移为单位矩阵
  A.block(0, 0, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

  // 遍历所有参考点，逐段计算并组装整体状态方程矩阵
  for (size_t i = 1; i < N_ref; ++i) {
    // 获取当前段的参考点信息
    const auto & p = ref_points.at(i - 1);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, p.curvature,
    // p.delta_arc_length);
    vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, 0.0, p.delta_arc_length);

    // 将单步状态方程矩阵填充到整体矩阵的对应位置
    A.block(i * D_x, (i - 1) * D_x, D_x, D_x) = Ad;
    B.block(i * D_x, (i - 1) * D_u, D_x, D_u) = Bd;
    W.segment(i * D_x, D_x) = Wd;
  }

  return Matrix{A, B, W};
}

Eigen::VectorXd StateEquationGenerator::predict(
  const StateEquationGenerator::Matrix & mat, const Eigen::VectorXd U) const
{
  return mat.B * U + mat.W;
}
}  // namespace autoware::path_optimizer
