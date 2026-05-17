// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/pure_pursuit/autoware_pure_pursuit.hpp"

#include "autoware/pure_pursuit/util/planning_utils.hpp"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::pure_pursuit
{
bool PurePursuit::isDataReady()
{
  if (!curr_wps_ptr_) {
    return false;
  }
  if (!curr_pose_ptr_) {
    return false;
  }
  return true;
}

std::pair<bool, double> PurePursuit::run()
{
  // run() 是 Pure Pursuit 几何求解的入口，调用前必须已经设置当前位姿、路径点和前瞻距离。
  if (!isDataReady()) {
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // 先根据距离和航向角阈值寻找离车辆当前位姿最近的路径点，避免匹配到距离近但方向明显不一致的点。
  auto closest_pair = planning_utils::findClosestIdxWithDistAngThr(
    *curr_wps_ptr_, *curr_pose_ptr_, closest_thr_dist_, closest_thr_ang_);

  if (!closest_pair.first) {
    RCLCPP_WARN(
      logger, "cannot find, curr_bool: %d, closest_idx: %d", closest_pair.first,
      closest_pair.second);
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // 从最近点开始向轨迹前方搜索，找到第一个距离车辆超过 lookahead_distance_ 的路径点。不是沿着轨迹的弧长，而是两个坐标点的距离
  // 这个点只是包围前瞻圆交点的离散路径点，真正的目标点会在下一步通过插值求出。
  int32_t next_wp_idx = findNextPointIdx(closest_pair.second);
  if (next_wp_idx == -1) {
    RCLCPP_WARN(logger, "lost next waypoint");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  // 保存离散前瞻路径点位置，主要用于调试显示。
  loc_next_wp_ = curr_wps_ptr_->at(next_wp_idx).position;

  geometry_msgs::msg::Point next_tgt_pos;
  // 如果第 0 个路径点就已经满足前瞻距离，前面没有线段可插值，只能直接使用该点作为目标点。
  if (next_wp_idx == 0) {
    next_tgt_pos = curr_wps_ptr_->at(next_wp_idx).position;
  } else {
    // 在 next_wp_idx - 1 与 next_wp_idx 之间做线性插值，求路径线段与前瞻圆的交点。
    // 这样得到的目标点距离车辆正好约等于 lookahead_distance_，比直接使用离散路径点更平滑。
    std::pair<bool, geometry_msgs::msg::Point> lerp_pair = lerpNextTarget(next_wp_idx);

    if (!lerp_pair.first) {
      RCLCPP_WARN(logger, "lost target! ");
      return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
    }

    next_tgt_pos = lerp_pair.second;
  }
  // 保存最终前瞻目标点，控制器会通过 getter 发布 RViz 调试 marker。
  loc_next_tgt_ = next_tgt_pos;

  // Pure Pursuit 假设车辆沿一个圆弧到达前瞻目标点，这里计算该圆弧曲率 kappa。
  // 上层控制器会把 kappa 转换成前轮转向角。
  double kappa = planning_utils::calcCurvature(next_tgt_pos, *curr_pose_ptr_);

  return std::make_pair(true, kappa);
}

// 在离散前瞻点前后的轨迹线段上插值，求出距离车辆正好为 lookahead_distance_ 的目标点。
std::pair<bool, geometry_msgs::msg::Point> PurePursuit::lerpNextTarget(int32_t next_wp_idx)
{
  // 几何计算中的容差，用于判断路径点重合、圆与线段相切等近似情况。
  constexpr double ERROR2 = 1e-5;  // 0.00001
  // vec_start 和 vec_end 分别是前瞻圆交点所在轨迹线段的起点 A 和终点 B。
  const geometry_msgs::msg::Point & vec_end = curr_wps_ptr_->at(next_wp_idx).position;
  const geometry_msgs::msg::Point & vec_start = curr_wps_ptr_->at(next_wp_idx - 1).position;
  const geometry_msgs::msg::Pose & curr_pose = *curr_pose_ptr_;

  // 轨迹线段向量 AB，后续会归一化为线段方向单位向量 u。
  Eigen::Vector3d vec_a(
    (vec_end.x - vec_start.x), (vec_end.y - vec_start.y), (vec_end.z - vec_start.z));

  if (vec_a.norm() < ERROR2) {
    // 相邻路径点几乎重合时，无法确定线段方向，也就无法做线段与圆的交点计算。
    RCLCPP_ERROR(logger, "waypoint interval is almost 0");
    return std::make_pair(false, geometry_msgs::msg::Point());
  }

  // lateral_error 是车辆当前位置到线段 AB 的有符号垂直距离，对应几何推导中的 h。
  const double lateral_error =
    planning_utils::calcLateralError2D(vec_start, vec_end, curr_pose.position);

  // 如果车辆到线段 AB 的垂直距离已经大于前瞻半径，说明前瞻圆与该线段没有交点。
  if (fabs(lateral_error) > lookahead_distance_) {
    RCLCPP_ERROR(logger, "lateral error is larger than lookahead distance");
    RCLCPP_ERROR(
      logger, "lateral error: %lf, lookahead distance: %lf", lateral_error, lookahead_distance_);
    return std::make_pair(false, geometry_msgs::msg::Point());
  }

  // uva2d 是线段 AB 在 xy 平面内的单位方向向量 u。
  Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
  uva2d.normalize();
  // 将线段方向向量旋转 90 度，得到从车辆位置指向垂足 H 的方向。
  // lateral_error 的符号表示车辆在线段哪一侧，因此旋转方向要随符号切换。
  Eigen::Rotation2Dd rot =
    (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
  Eigen::Vector2d uva2d_rot = rot * uva2d;

  // 计算车辆当前位置到线段 AB 的垂足 H。
  geometry_msgs::msg::Point h;
  h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
  h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
  h.z = curr_pose.position.z;

  // 如果 |h| 与 lookahead_distance_ 基本相等，前瞻圆与线段相切，垂足 H 就是目标点。
  if (fabs(fabs(lateral_error) - lookahead_distance_) < ERROR2) {
    return std::make_pair(true, h);
  } else {
    // 前瞻圆与线段有两个交点时，取沿轨迹前进方向的交点：
    // 从垂足 H 沿单位方向 u 前进 s，其中 s = sqrt(Ld^2 - h^2)。
    const double s = sqrt(pow(lookahead_distance_, 2) - pow(lateral_error, 2));
    geometry_msgs::msg::Point res;
    res.x = h.x + s * uva2d.x();
    res.y = h.y + s * uva2d.y();
    res.z = curr_pose.position.z;
    return std::make_pair(true, res);
  }
}

int32_t PurePursuit::findNextPointIdx(int32_t search_start_idx)  //从指定索引开始遍历路径点，排除不符合行驶方向（前/后）的点，找到第一个距离车辆超过预瞄距离 lookahead_distance_ 的路径点作为控制目标
{
  // if waypoints are not given, do nothing.
  if (curr_wps_ptr_->empty() || search_start_idx == -1) {
    return -1;
  }

  // look for the next waypoint.
  for (int32_t i = search_start_idx; i < static_cast<int32_t>(curr_wps_ptr_->size()); i++) {
    // if search waypoint is the last
    if (i == (static_cast<int32_t>(curr_wps_ptr_->size()) - 1)) {
      return i;
    }

    // if waypoint direction is forward
    const auto gld = planning_utils::getLaneDirection(*curr_wps_ptr_, 0.05); //获取车道方向（0 表示向前，1 表示向后）。
    if (gld == 0) {
      // if waypoint is not in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D( //当前路径点坐标转换到车辆相对坐标系下。
        curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
      if (ret.x < 0) {
        continue;
      }
    } else if (gld == 1) {
      // waypoint direction is backward

      // if waypoint is in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D(
        curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
      if (ret.x > 0) {
        continue;
      }
    } else {
      return -1;
    }

    const geometry_msgs::msg::Point & curr_motion_point = curr_wps_ptr_->at(i).position;
    const geometry_msgs::msg::Point & curr_pose_point = curr_pose_ptr_->position;
    // if there exists an effective waypoint
    const double ds = planning_utils::calcDistSquared2D(curr_motion_point, curr_pose_point);
    if (ds > std::pow(lookahead_distance_, 2)) {
      return i;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  return -1;
}

void PurePursuit::setCurrentPose(const geometry_msgs::msg::Pose & msg)
{
  curr_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>();
  *curr_pose_ptr_ = msg;
}

void PurePursuit::setWaypoints(const std::vector<geometry_msgs::msg::Pose> & msg)
{
  curr_wps_ptr_ = std::make_shared<std::vector<geometry_msgs::msg::Pose>>();
  *curr_wps_ptr_ = msg;
}

}  // namespace autoware::pure_pursuit
