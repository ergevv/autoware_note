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

#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <vector>

namespace autoware::velocity_smoother
{
namespace analytical_velocity_planning_utils
{
// 给定初始速度 v0、初始加速度 a0、减速 jerk、恢复 jerk、最小加速度 min_acc、
// 目标速度 target_vel，解析计算需要多少距离才能平滑降到目标速度，并且终点加速度回到 0。
bool calcStopDistWithJerkAndAccConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, int & type, std::vector<double> & times,
  double & stop_dist)
{
  // 这个函数用解析法计算：在分段常 jerk 约束下，从状态 (v0, a0) 变化到
  // (target_vel, 0) 所需要的距离。
  //
  // 推导从 jerk 的定义开始：
  //   j = da/dt
  // 对时间积分一次得到加速度：
  //   a(t) = a0 + j * t
  // 再积分一次得到速度：
  //   v(t) = v0 + a0 * t + 1/2 * j * t^2
  // 再积分一次得到位移：
  //   x(t) = x0 + v0 * t + 1/2 * a0 * t^2 + 1/6 * j * t^3
  //
  // jerk_dec 为负值，用于让加速度变得更负，从而开始减速。
  // jerk_acc 为正值，用于让加速度从负值恢复到 0。
  // min_acc 是加速度下限；当需要恒定减速度阶段时，加速度会保持在该值。

  // 首先假设加速度曲线是“梯形”：
  //   1) jerk_dec：a0 -> min_acc
  //   2) 0 jerk：  保持 min_acc，持续 t_min
  //   3) jerk_acc：min_acc -> 0
  //
  // 对速度方程求解第二阶段持续时间：
  //   target_vel - v0 = dv_phase1 + min_acc * t_min + dv_phase3
  // 其中每个 jerk 阶段的速度变化量，等于该阶段加速度曲线下方的面积。
  //
  // 对于常 jerk 阶段，加速度随时间线性变化，所以速度变化量也可以写成梯形面积：
  //   dv = integral(a dt) = (a_start + a_end) / 2 * dt
  //   dt = (a_end - a_start) / j
  //
  // 阶段 1 从 a0 到 min_acc，可拆成 a0 -> 0 和 0 -> min_acc 两个三角面积：
  //   dv_phase1 =
  //     1/2 * ((0 - a0) / jerk_dec) * a0
  //     + 1/2 * ((min_acc - 0) / jerk_dec) * min_acc
  // 阶段 3 从 min_acc 到 0：
  //   dv_phase3 = 1/2 * ((0 - min_acc) / jerk_acc) * min_acc
  //
  // 把这些代入 target_vel - v0 = dv_phase1 + min_acc * t_min + dv_phase3，
  // 即可得到下面的 t_min。
  // 如果 t_min > 0，说明加速度确实会到达 min_acc，并存在恒定减速度平台。
  const double t_min =
    (target_vel - v0 - 0.5 * (0 - a0) / jerk_dec * a0 - 0.5 * min_acc / jerk_dec * min_acc -
     0.5 * (0 - min_acc) / jerk_acc * min_acc) /
    min_acc;

  if (t_min > 0) {
    // Type 1：梯形加速度曲线。
    // 阶段 1：使用负 jerk，将加速度从 a0 降到 min_acc。
    // 由 a(t) = a0 + jerk_dec * t，令 a(t1) = min_acc，可得：
    //   t1 = (min_acc - a0) / jerk_dec
    double t1 = (min_acc - a0) / jerk_dec;
    if (t1 < 0.01) {
      t1 = 0;
    }

    // 阶段 1 结束时的状态。
    const double a1 = a0 + jerk_dec * t1;
    const double v1 = v0 + a0 * t1 + 0.5 * jerk_dec * t1 * t1;
    const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_dec * t1 * t1 * t1;

    // 阶段 2：保持加速度为 min_acc。此时 jerk 为 0，
    // 所以从 (x1, v1, a1) 开始做普通的匀加速度积分：
    //   a2 = a1
    //   v2 = v1 + a1 * t2
    //   x2 = x1 + v1 * t2 + 1/2 * a1 * t2^2
    double t2 = t_min;
    if (t2 < 0.01) {
      t2 = 0;
    }

    const double a2 = a1;
    const double v2 = v1 + a1 * t2;
    const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2;

    // 阶段 3：使用正 jerk，将加速度从 min_acc 恢复到 0，
    // 让车辆到达目标速度时加速度也回到 0。
    // 由 a(t) = min_acc + jerk_acc * t，令 a(t3) = 0，可得：
    //   t3 = (0 - min_acc) / jerk_acc
    double t3 = (0 - min_acc) / jerk_acc;
    if (t3 < 0.01) {
      t3 = 0;
    }

    // 完整减速曲线结束时的状态。
    const double a3 = a2 + jerk_acc * t3;
    const double v3 = v2 + a2 * t3 + 0.5 * jerk_acc * t3 * t3;
    const double x3 = x2 + v2 * t3 + 0.5 * a2 * t3 * t3 + (1.0 / 6.0) * jerk_acc * t3 * t3 * t3;

    // 这里允许一定误差，因为解析计算之后仍然会受到浮点数运算和小时间段截断的影响。
    const double a_target = 0.0;
    const double v_margin = 0.3;  // [m/s]
    const double a_margin = 0.1;  // [m/s^2]
    if (!validCheckCalcStopDist(v3, a3, target_vel, a_target, v_margin, a_margin)) {
      RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 1");
      return false;
    }

    type = 1;
    times.push_back(t1);
    times.push_back(t2);
    times.push_back(t3);
    stop_dist = x3;
  } else {
    // 如果 t_min <= 0，说明所需速度下降量不大，不需要真的降到 min_acc。
    // 此时曲线可能是：
    //   Type 2：三角形加速度曲线，a0 -> a1 -> 0
    //   Type 3：仅恢复加速度；当前减速已经足够，只需要让 a 回到 0
    //
    // is_decel_needed 用来比较：
    //   “仅用正 jerk 将 a0 恢复到 0 造成的速度变化”
    // 和
    //   “从 v0 到 target_vel 所需的速度变化”。
    //
    // 仅恢复加速度时：
    //   t_recover = (0 - a0) / jerk_acc
    //   dv_recover = (a0 + 0) / 2 * t_recover
    //              = 1/2 * ((0 - a0) / jerk_acc) * a0
    // 目标速度变化为：
    //   dv_required = target_vel - v0
    // 所以：
    //   is_decel_needed = dv_recover - dv_required
    //
    // 当 is_decel_needed > 0 时，说明“仅恢复加速度”的减速量还不够；
    // 当 a0 > 0 时，车辆还在加速，也必须先用负 jerk 把加速度压下来。
    // 如果仅恢复加速度还不够减速，或者 a0 本身为正，就必须先增加一个负 jerk 阶段。
    const double is_decel_needed = 0.5 * (0 - a0) / jerk_acc * a0 - (target_vel - v0);
    if (is_decel_needed > 0 || a0 > 0) {
      // Type 2：三角形加速度曲线。
      // 令 a1 为负加速度峰值。速度变化量满足：
      //   target_vel - v0 =
      //     使用 jerk_dec 从 a0 到 0 的速度变化
      //     + 使用 jerk_dec 从 0 到 a1 的速度变化
      //     + 使用 jerk_acc 从 a1 到 0 的速度变化
      //
      // 其中：
      //   dv_a0_to_0 = 1/2 * ((0 - a0) / jerk_dec) * a0
      //   dv_0_to_a1 = 1/2 * ((a1 - 0) / jerk_dec) * a1
      //               = 1/2 * a1^2 / jerk_dec
      //   dv_a1_to_0 = 1/2 * ((0 - a1) / jerk_acc) * a1
      //               = -1/2 * a1^2 / jerk_acc
      //
      // 移项得到：
      //   target_vel - v0 - dv_a0_to_0
      //     = 1/2 * a1^2 * (1 / jerk_dec - 1 / jerk_acc)
      //     = a1^2 * (jerk_acc - jerk_dec) / (2 * jerk_acc * jerk_dec)
      //
      // 因此：
      //   a1^2 =
      //     (target_vel - v0 - dv_a0_to_0)
      //     * (2 * jerk_acc * jerk_dec / (jerk_acc - jerk_dec))
      // 由于这是减速曲线，所以 a1 取负根。
      const double a1_square = (target_vel - v0 - 0.5 * (0 - a0) / jerk_dec * a0) *
                               (2 * jerk_acc * jerk_dec / (jerk_acc - jerk_dec));
      const double a1 = -std::sqrt(a1_square);

      // 阶段 1：使用负 jerk，从 a0 到 a1。
      // 由 a(t) = a0 + jerk_dec * t，令 a(t1) = a1，可得：
      //   t1 = (a1 - a0) / jerk_dec
      double t1 = (a1 - a0) / jerk_dec;
      if (t1 < 0.01) {
        t1 = 0;
      }

      const double v1 = v0 + a0 * t1 + 0.5 * jerk_dec * t1 * t1;
      const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_dec * t1 * t1 * t1;

      // 阶段 2：使用正 jerk，从 a1 恢复到 0。
      // 由 a(t) = a1 + jerk_acc * t，令 a(t2) = 0，可得：
      //   t2 = (0 - a1) / jerk_acc
      double t2 = (0 - a1) / jerk_acc;
      if (t2 < 0.01) {
        t2 = 0;
      }

      const double a2 = a1 + jerk_acc * t2;
      const double v2 = v1 + a1 * t2 + 0.5 * jerk_acc * t2 * t2;
      const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2 + (1.0 / 6.0) * jerk_acc * t2 * t2 * t2;

      const double a_target = 0.0;
      const double v_margin = 0.3;
      const double a_margin = 0.1;
      if (!validCheckCalcStopDist(v2, a2, target_vel, a_target, v_margin, a_margin)) {
        RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 2");
        return false;
      }

      type = 2;
      times.push_back(t1);
      times.push_back(t2);
      stop_dist = x2;
    } else {
      // Type 3：仅恢复加速度曲线。
      // 车辆当前已经在充分减速；只需要施加正 jerk，直到 a = 0，
      // 不需要再增加负 jerk 阶段，就可以达到目标速度。
      // 这里仍然使用同一套积分公式：
      //   t1 = (0 - a0) / jerk_acc
      //   v1 = v0 + a0 * t1 + 1/2 * jerk_acc * t1^2
      //   x1 = v0 * t1 + 1/2 * a0 * t1^2 + 1/6 * jerk_acc * t1^3
      double t1 = (0 - a0) / jerk_acc;
      if (t1 < 0) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("velocity_planning_utils"), "t1 < 0. unexpected condition.");
        return false;
      }
      if (t1 < 0.01) {
        t1 = 0;
      }

      const double a1 = a0 + jerk_acc * t1;
      const double v1 = v0 + a0 * t1 + 0.5 * jerk_acc * t1 * t1;
      const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_acc * t1 * t1 * t1;

      const double a_target = 0.0;
      const double v_margin = 0.3;
      const double a_margin = 0.1;
      if (!validCheckCalcStopDist(v1, a1, target_vel, a_target, v_margin, a_margin)) {
        RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 3");
        return false;
      }

      type = 3;
      times.push_back(t1);
      stop_dist = x1;
    }
  }
  return true;
}

bool validCheckCalcStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);
  if (v_end < v_min || v_max < v_end) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("velocity_planning_utils"), "Valid check error! v_target = %f, v_end = %f",
      v_target, v_end);
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("velocity_planning_utils"), "Valid check error! a_target = %f, a_end = %f",
      a_target, a_end);
    return false;
  }
  return true;
}

bool calcStopVelocityWithConstantJerkAccLimit(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double decel_target_vel, const int type,
  const std::vector<double> & times, const size_t start_index, TrajectoryPoints & output_trajectory)
{
  // 根据 calcStopDistWithJerkAndAccConstraints() 已经求出的曲线类型 type 和各阶段时间 times，
  // 生成一条连续的减速曲线，然后把这条曲线按照轨迹点的弧长位置插值回 output_trajectory。
  //
  // 输入的 type/times 表示三种可能的加速度曲线：
  //   type 1：梯形加速度，负 jerk -> 恒定 min_acc -> 正 jerk
  //   type 2：三角形加速度，负 jerk -> 正 jerk
  //   type 3：仅恢复加速度，正 jerk
  //
  // 本函数的输出效果是：
  //   start_index 之后的一段轨迹点速度/加速度被解析减速曲线覆盖；
  //   减速曲线结束之后的点被固定为 decel_target_vel，acc = 0。
  const double t_total = std::accumulate(times.begin(), times.end(), 0.0);

  // ts/xs/vs/as/js 是连续解析曲线的离散采样表：
  //   ts: 采样时刻
  //   xs: 从减速起点出发的累计距离
  //   vs: 对应时刻速度
  //   as: 对应时刻加速度
  //   js: 对应时刻 jerk
  //
  // 后面会以 xs 为横轴，将 vs/as/js 插值到每个轨迹点的距离位置上。
  std::vector<double> ts, xs, vs, as, js;
  const double dt = 0.1;
  double x = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;

  // 以固定时间间隔 dt 采样解析曲线。
  // updateStopVelocityStatus() 会根据 type 和 times，自动判断当前 t 落在哪个阶段，
  // 并用常 jerk 积分公式计算此刻的 x/v/a/j。
  for (double t = 0.0; t < t_total; t += dt) {
    updateStopVelocityStatus(v0, a0, jerk_acc, jerk_dec, type, times, t, x, v, a, j);
    if (v > 0.0) {
      // 理论上 calcStopDistWithJerkAndAccConstraints() 已经按 min_acc 构造曲线。
      // 这里再 clamp 一次，避免数值误差让加速度低于允许下限。
      a = std::max(a, min_acc);
      ts.push_back(t);
      xs.push_back(x);
      vs.push_back(v);
      as.push_back(a);
      js.push_back(j);
    }
  }

  // 固定 dt 的 for 循环不一定正好采到 t_total，因此额外采样终点。
  // xs.back() < x 用来避免重复加入相同位置的点，否则后面的线性插值要求会被破坏。
  updateStopVelocityStatus(v0, a0, jerk_acc, jerk_dec, type, times, t_total, x, v, a, j);
  if (v > 0.0 && !xs.empty() && xs.back() < x) {
    a = std::max(a, min_acc);
    ts.push_back(t_total);
    xs.push_back(x);
    vs.push_back(v);
    as.push_back(a);
    js.push_back(j);
  }

  // for debug
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Calculate stop velocity.");
  for (unsigned int i = 0; i < ts.size(); ++i) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("velocity_planning_utils"), "--- t: %f, x: %f, v: %f, a: %f, j: %f",
      ts.at(i), xs.at(i), vs.at(i), as.at(i), js.at(i));
  }

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;
  // 检查解析曲线终点是否确实到达目标速度，且加速度回到 0。
  // 允许 margin 是因为采样、浮点计算和很小时间段截断会带来数值误差。
  if (!validCheckCalcStopDist(v, a, decel_target_vel, a_target, v_margin, a_margin)) {
    return false;
  }

  // 如果没有有效采样点，说明减速曲线长度过短或速度已经没有正值。
  // 这种情况下直接把 start_index 之后都设为目标速度和 0 加速度。
  if (xs.empty()) {
    for (size_t i = start_index; i < output_trajectory.size(); ++i) {
      output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
      output_trajectory.at(i).acceleration_mps2 = 0.0;
    }
    return true;
  }

  // 计算每个轨迹点相对 start_index 的累计距离。
  // distances[k] 与 output_trajectory[start_index + k] 对应。
  // 只收集到解析曲线覆盖的最大距离 xs.back() 为止；之后的轨迹点会统一设为目标速度。
  double distance = 0.0;
  std::vector<double> distances;
  distances.push_back(distance);
  for (size_t i = start_index; i < output_trajectory.size() - 1; ++i) {
    distance += autoware_utils_geometry::calc_distance2d(
      output_trajectory.at(i), output_trajectory.at(i + 1));
    if (distance > xs.back()) {
      break;
    }
    distances.push_back(distance);
  }

  // 线性插值要求横轴单调。这里同时检查严格递增和非递减：
  //   xs 是解析曲线中的距离采样点；
  //   distances 是轨迹离散点的距离位置。
  // 如果存在重复距离、倒退距离或异常点，插值会不可靠，因此直接失败。
  if (
    !autoware::interpolation::isIncreasing(xs) ||
    !autoware::interpolation::isIncreasing(distances) ||
    !autoware::interpolation::isNotDecreasing(xs) ||
    !autoware::interpolation::isNotDecreasing(distances)) {
    return false;
  }

  // 插值还要求：
  //   1) 至少有两个采样点，否则无法线性插值；
  //   2) 轨迹点距离范围必须落在解析曲线距离范围 [xs.front(), xs.back()] 内。
  if (
    xs.size() < 2 || vs.size() < 2 || as.size() < 2 || js.size() < 2 || distances.empty() ||
    distances.front() < xs.front() || xs.back() < distances.back()) {
    return false;
  }

  // 以距离 x 为自变量，将连续曲线的速度、加速度、jerk 插值到轨迹点位置。
  // 注意 jerk_at_wp 当前只计算未写回，保留主要用于调试或后续扩展。
  const auto vel_at_wp = autoware::interpolation::lerp(xs, vs, distances);
  const auto acc_at_wp = autoware::interpolation::lerp(xs, as, distances);
  const auto jerk_at_wp = autoware::interpolation::lerp(xs, js, distances);

  // 将解析减速曲线覆盖到轨迹点。
  for (size_t i = 0; i < vel_at_wp.size(); ++i) {
    output_trajectory.at(start_index + i).longitudinal_velocity_mps = vel_at_wp.at(i);
    output_trajectory.at(start_index + i).acceleration_mps2 = acc_at_wp.at(i);
  }

  // 解析曲线结束后的点已经达到目标速度，并且加速度为 0，
  // 因此后续轨迹保持 decel_target_vel。
  for (size_t i = start_index + vel_at_wp.size(); i < output_trajectory.size(); ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }

  return true;
}

void updateStopVelocityStatus(
  double v0, double a0, double jerk_acc, double jerk_dec, int type,
  const std::vector<double> & times, double t, double & x, double & v, double & a, double & j)
{
  if (type == 1) {
    if (0 <= t && t < times.at(0)) {
      j = jerk_dec;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    } else if (times.at(0) <= t && t < times.at(0) + times.at(1)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double dt = t - t1;
      j = 0;
      a = integ_a(a1, j, dt);
      v = integ_v(v1, a1, j, dt);
      x = integ_x(x1, v1, a1, j, dt);
    } else if (times.at(0) + times.at(1) <= t && t <= times.at(0) + times.at(1) + times.at(2)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double t2 = times.at(1);
      const double a2 = integ_a(a1, 0, t2);
      const double v2 = integ_v(v1, a1, 0, t2);
      const double x2 = integ_x(x1, v1, a1, 0, t2);

      const double dt = t - (t1 + t2);
      j = jerk_acc;
      a = integ_a(a2, j, dt);
      v = integ_v(v2, a2, j, dt);
      x = integ_x(x2, v2, a2, j, dt);
    }
  } else if (type == 2) {
    if (0 <= t && t < times.at(0)) {
      j = jerk_dec;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    } else if (times.at(0) <= t && t <= times.at(0) + times.at(1)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double dt = t - t1;
      j = jerk_acc;
      a = integ_a(a1, j, dt);
      v = integ_v(v1, a1, j, dt);
      x = integ_x(x1, v1, a1, j, dt);
    }
  } else if (type == 3) {
    if (0 <= t && t <= times.at(0)) {
      j = jerk_acc;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    }
  } else {
  }
}

double integ_x(double x0, double v0, double a0, double j0, double t)
{
  return x0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j0 * t * t * t;
}

double integ_v(double v0, double a0, double j0, double t)
{
  return v0 + a0 * t + 0.5 * j0 * t * t;
}

double integ_a(double a0, double j0, double t)
{
  return a0 + j0 * t;
}

}  // namespace analytical_velocity_planning_utils
}  // namespace autoware::velocity_smoother
