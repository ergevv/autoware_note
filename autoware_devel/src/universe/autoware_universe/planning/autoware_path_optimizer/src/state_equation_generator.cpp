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
 * 该函数基于参考点序列和车辆运动学模型，构建离散化的整段状态方程矩阵。
 * 单步状态方程为：
 *
 *   x_i = Ad_{i-1} * x_{i-1} + Bd_{i-1} * u_{i-1} + Wd_{i-1}
 *
 * 其中 x_i 通常为 [横向误差, 航向误差]^T，u_i 为转向输入。
 *
 * 注意：这里返回的 A/B/W 不是完全消元后的预测形式 X = B U + W，而是把每一段
 * 单步递推堆叠成：
 *
 *   X = A * X + B * U + W
 *
 * 后续 calcConstraintMatrix() 会把它改写成 QP 等式约束：
 *
 *   (I - A) * X - B * U = W
 *
 * 这样 X 和 U 都作为优化变量，可以直接对每个状态点添加边界、固定点等约束。
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

  // 单点状态维度和输入维度由车辆模型决定。
  // 当前自行车模型中 D_x = 2，对应 [lat_error, yaw_error]；
  // D_u = 1，对应一段轨迹上的转向输入。
  const size_t D_x = vehicle_model_ptr_->getDimX();
  const size_t D_u = vehicle_model_ptr_->getDimU();

  // N_ref 个参考点会产生 N_ref 个状态块 x_0 ... x_{N-1}。
  // 控制输入作用在相邻参考点之间，因此只有 N_ref - 1 个输入块 u_0 ... u_{N-2}。
  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // 初始化整段递推矩阵：
  //   X = A * X + B * U + W
  //
  // X = [x_0, x_1, ..., x_{N-1}]^T
  // U = [u_0, u_1, ..., u_{N-2}]^T
  //
  // A 的非零块描述 x_i 对 x_{i-1} 的依赖；
  // B 的非零块描述 x_i 对 u_{i-1} 的依赖；
  // W 存每段递推的常数/曲率偏置项。
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_x, N_x);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N_x, N_u);
  Eigen::VectorXd W = Eigen::VectorXd::Zero(N_x);

  // 单步离散车辆模型的临时矩阵：
  //   x_i = Ad * x_{i-1} + Bd * u_{i-1} + Wd
  Eigen::MatrixXd Ad(D_x, D_x);
  Eigen::MatrixXd Bd(D_x, D_u);
  Eigen::MatrixXd Wd(D_x, 1);

  // 第 0 个状态 x_0 不由上一段状态/输入递推而来。
  // 这里设置 A(0,0)=I，使第一行块表示恒等式：
  //   x_0 = x_0
  // 换到 QP 约束 (I-A)X - BU = W 后，这一行会变成 0=0，不会固定 x_0。
  // 如果需要固定前端状态，会由 MPTOptimizer 的 fixed point 约束单独添加。
  A.block(0, 0, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

  // 从第 1 个参考点开始，逐段组装：
  //   x_i = Ad_{i-1} x_{i-1} + Bd_{i-1} u_{i-1} + Wd_{i-1}
  for (size_t i = 1; i < N_ref; ++i) {
    // 第 i 个状态由第 i-1 个参考点到第 i 个参考点这一段产生。
    // p.delta_arc_length 是该段空间离散步长 ds。
    const auto & p = ref_points.at(i - 1);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, p.curvature,
    // p.delta_arc_length);
    // 目前这里暂时把曲率传成 0.0，而不是 p.curvature。
    // 因此实际使用的是近似直线路径上的误差动力学：
    //   lat_i = lat_{i-1} + ds * yaw_{i-1}
    //   yaw_i = yaw_{i-1} + ds / wheel_base * steer_{i-1}
    // 若恢复使用 p.curvature，Wd 会包含参考曲率带来的偏置项。delta_arc_length是在updateDeltaArcLength计算的相邻点距离
    vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, 0.0, p.delta_arc_length);

    // 把单步方程写入整段矩阵对应的块：
    //
    // A 的第 i 行块、第 i-1 列块 = Ad
    // B 的第 i 行块、第 i-1 列块 = Bd
    // W 的第 i 个状态块 = Wd
    //
    // 因而整段矩阵中第 i 个状态块表示：
    //   X_i = A(i,i-1) * X_{i-1} + B(i,i-1) * U_{i-1} + W_i
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
