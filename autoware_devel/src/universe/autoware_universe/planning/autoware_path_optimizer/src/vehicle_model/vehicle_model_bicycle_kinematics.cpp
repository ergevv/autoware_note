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
 * 状态方程形式为:
 *
 *   x[k+1] = Ad * x[k] + Bd * u[k] + Wd
 * 
 * 其中：
 *   x = [lat_error, yaw_error]^T
 *     - lat_error: 车辆相对参考路径的横向误差
 *     - yaw_error: 车辆航向角相对参考路径切线方向的航向误差
 *   u = steer
 *     - steer: 前轮转角本身，而不是转角变化量
 *
 * 推导过程：
 *
 * 1. 使用以参考路径弧长 s 为自变量的误差模型。对于小横向误差和小航向误差，
 *    自行车模型可以近似为：
 *
 *      d(lat_error) / ds = yaw_error
 *      d(yaw_error) / ds = tan(steer) / wheelbase - curvature
 *
 *    第一式表示：如果车身航向比参考线多偏了 yaw_error，沿路径前进 ds 后，
 *    横向误差大约增加 ds * yaw_error。
 *
 * 2. 为了得到 QP 可使用的线性模型，需要把非线性项 tan(steer) 线性化。
 *    参考路径曲率 curvature 对应的理想前轮转角为：
 *
 *      delta_r = atan(wheelbase * curvature)
 *
 *    因为理想情况下 tan(delta_r) / wheelbase = curvature。
 *
 * 3. 在 delta_r 附近对 tan(steer) 做一阶泰勒展开：
 *
 *      tan(steer) ~= tan(delta_r) + sec^2(delta_r) * (steer - delta_r)
 *                  = steer / cos^2(delta_r)
 *                    + tan(delta_r) - delta_r / cos^2(delta_r)
 *
 *    代入 d(yaw_error)/ds 后得到仿射形式：
 *
 *      d(yaw_error) / ds
 *        = steer / (wheelbase * cos^2(delta_r))
 *          - curvature
 *          + (tan(delta_r) - delta_r / cos^2(delta_r)) / wheelbase
 *
 * 4. 使用前向欧拉法按空间步长 ds 离散化：
 *
 *      lat_error[k+1] = lat_error[k] + ds * yaw_error[k]
 *      yaw_error[k+1] = yaw_error[k]
 *        + ds / (wheelbase * cos^2(delta_r)) * steer[k]
 *        + ds * (-curvature
 *                + (tan(delta_r) - delta_r / cos^2(delta_r)) / wheelbase)
 *
 *    因此可以写成矩阵：
 *
 *      Ad = [1, ds]
 *           [0,  1]
 *
 *      Bd = [0]
 *           [ds / (wheelbase * cos^2(delta_r))]
 *
 *      Wd = [0]
 *           [-ds * curvature
 *            + ds / wheelbase * (tan(delta_r) - delta_r / cos^2(delta_r))]
 *
 *    代码中 Wd 的常数项使用 cropped_delta_r，是为了在参考曲率对应的理想转角超过
 *    物理转角上限时，不让常数偏置项继续使用不可执行的参考转角。
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
 * 
 * 
 * 当前故意传 0.0。注释里说使用曲率后 sample_map 上优化结果会异常，所以暂时关闭曲率项。直观理解是：优化器目前不把参考路径曲率作为车辆模型的前馈项，而是用一个更简单、更稳定的直线误差传播模型来约束横向误差和航向误差的变化。
 */
void KinematicsBicycleModel::
calculateStateEquationMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double curvature,
  const double ds) const
{
  // delta_r 是“如果车辆完全贴着参考曲率行驶”所需要的理想前轮转角：
  //   curvature = tan(delta_r) / wheelbase
  // => delta_r = atan(wheelbase * curvature)
  const double delta_r = std::atan(wheelbase_ * curvature);

  // cropped_delta_r 是物理转角范围内可执行的参考转角。
  // 当参考曲率过大时，理想 delta_r 可能超过车辆最大转角，因此常数偏置项使用限幅值。
  const double cropped_delta_r = std::clamp(delta_r, -steer_limit_, steer_limit_);

  // NOTE: cos(delta_r) will not be zero since curvature will not be infinity
  // lat_error[k+1] = lat_error[k] + ds * yaw_error[k]
  // yaw_error[k+1] 的自状态项为 yaw_error[k]，因此第二行是 [0, 1]。
  Ad << 1.0, ds, 0.0, 1.0;

  // steer 对 yaw_error 的影响来自 tan(steer) 对 steer 的导数：
  //   d(tan(steer))/d(steer) = 1 / cos^2(steer)
  // 在 delta_r 处线性化，并乘上空间离散步长 ds 和 1 / wheelbase。
  Bd << 0.0, ds / wheelbase_ / std::pow(std::cos(delta_r), 2.0);

  // Wd 是 yaw_error 方程里的常数项：
  //   -ds * curvature
  // 来自参考路径本身的转向；
  //   ds / wheelbase * (tan(delta) - delta / cos^2(delta))
  // 来自 tan(steer) 在参考转角附近一阶展开后的截距项。
  //
  // 若 curvature = 0，则 delta_r = cropped_delta_r = 0，Wd = [0, 0]^T，
  // 模型退化为直线路径附近的误差模型。
  Wd << 0.0, -ds * curvature + ds / wheelbase_ *
                                 (std::tan(cropped_delta_r) -
                                  cropped_delta_r / std::pow(std::cos(cropped_delta_r), 2.0));
}
