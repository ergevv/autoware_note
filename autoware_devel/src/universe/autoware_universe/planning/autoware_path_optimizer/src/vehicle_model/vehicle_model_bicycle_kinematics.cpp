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

#include "autoware/path_optimizer/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <iostream>
#include <limits>
#include <vector>

KinematicsBicycleModel::KinematicsBicycleModel(const double wheelbase, const double steer_limit)
: VehicleModelInterface(2, 1, 2, wheelbase, steer_limit)
{
}

/**
 * @brief 计算自行车运动学模型的状态方程矩阵（离散化线性化形式）
 * 
 * 该函数基于给定的路径曲率和步长，计算离散化的状态空间模型矩阵。
 * 状态方程形式为: x[k+1] = Ad * x[k] + Bd * u[k] + Wd
 * 
 * 其中状态向量 x = [横向误差, 航向角误差]^T
 *       控制输入 u = [前轮转角变化量]
 * 
 * 该模型用于路径跟踪控制中的模型预测控制(MPC)或线性二次调节器(LQR)。
 * 
 * @param[out] Ad 状态转移矩阵 (2x2)，描述状态变量的自然演化
 * @param[out] Bd 控制输入矩阵 (2x1)，描述控制输入对状态的影响
 * @param[out] Wd 扰动/偏移向量 (2x1)，包含由参考路径曲率引起的非线性项
 * @param[in] curvature 参考路径在当前点的曲率 (1/m)
 * @param[in] ds 离散化步长，即沿路径的采样间隔 (m)
 * 
 * @note 矩阵Ad和Bd的计算基于理想的前轮转角 delta_r = atan(wheelbase * curvature)
 * @note 矩阵Wd中使用了限幅后的转角 cropped_delta_r 以确保物理可行性
 * @note cos(delta_r) 不会为零，因为曲率不会是无穷大
 */
void KinematicsBicycleModel::calculateStateEquationMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double curvature,
  const double ds) const
{
  // 计算参考路径对应的前轮转角，并进行限幅处理
  const double delta_r = std::atan(wheelbase_ * curvature);
  const double cropped_delta_r = std::clamp(delta_r, -steer_limit_, steer_limit_);

  // NOTE: cos(delta_r) will not be zero since curvature will not be infinity
  Ad << 1.0, ds, 0.0, 1.0;

  Bd << 0.0, ds / wheelbase_ / std::pow(std::cos(delta_r), 2.0);

  Wd << 0.0, -ds * curvature + ds / wheelbase_ *
                                 (std::tan(cropped_delta_r) -
                                  cropped_delta_r / std::pow(std::cos(cropped_delta_r), 2.0));
}
