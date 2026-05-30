// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <queue>
#include <utility>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
using autoware_utils::calc_distance2d;

double calcReedsSheppDistance(const Pose & p1, const Pose & p2, double radius)
{
  const auto rs_space = ReedsSheppStateSpace(radius);
  const ReedsSheppStateSpace::StateXYT pose0{
    p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  const ReedsSheppStateSpace::StateXYT pose1{
    p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

Pose calcRelativePose(const Pose & base_pose, const Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param)
: AbstractPlanningAlgorithm(
    planner_common_param, std::make_shared<rclcpp::Clock>(RCL_ROS_TIME), collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param, const rclcpp::Clock::SharedPtr & clock)
: AbstractPlanningAlgorithm(planner_common_param, clock, collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

void AstarSearch::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);

  // ensure minimum expansion distance is larger then grid cell diagonal length
  min_expansion_dist_ = std::max(astar_param_.expansion_distance, 1.5 * costmap_.info.resolution);
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);
}

void AstarSearch::resetData()
{
  // clearing openlist is necessary because otherwise remaining elements of openlist
  // point to deleted node.
  openlist_ = std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison>();
  const int nb_of_grid_nodes = costmap_.info.width * costmap_.info.height;
  const int total_astar_node_count = nb_of_grid_nodes * planner_common_param_.theta_size;
  graph_.assign(total_astar_node_count, AstarNode{});
  col_free_distance_map_.assign(nb_of_grid_nodes, std::numeric_limits<double>::max());
  shifted_goal_pose_ = {};
}

/// @brief Hybrid A* 单目标规划入口：从一个起点位姿搜索到一个目标位姿。
/// @param start_pose 全局坐标系下的搜索起点，通常是当前车辆位姿。
/// @param goal_pose 全局坐标系下的搜索目标，通常是道路中心线上的回归位姿。
/// @return 搜索成功返回 true；起终点非法或搜索失败时抛出异常。
///
/// 这个函数负责搭建一次 Hybrid A* 搜索的上下文：
/// 1. 清空上一次搜索留下的 open list、节点图、启发距离图等临时数据。
/// 2. 将起点和终点从全局坐标系转换到 costmap 局部坐标系。
/// 3. 检查起点和终点的完整车身矩形是否与障碍物或地图边界碰撞。
/// 4. 根据配置决定是否交换起终点，进入反向搜索模式。
/// 5. 构建从目标出发的二维自由空间距离图，作为启发函数的一部分。
/// 6. 创建起点节点并放入 open list。
/// 7. 运行 A* 主循环，成功后内部会生成 waypoints_。
bool AstarSearch::makePlan(const Pose & start_pose, const Pose & goal_pose)
{
  // 每次规划都是一次独立搜索，必须清空上一轮 open list、graph_ 和启发距离图。
  resetData();

  // costmap 自带 origin。Hybrid A* 在 costmap 局部坐标系中搜索，先做坐标变换。
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  // 起点或终点本身发生碰撞时，后续搜索没有意义，直接认为输入非法。这里不是只检查一个点，而是检查车辆矩形 footprint 是否碰撞。如果起点或终点已经撞障碍物、越界、进入不可通行区域，就直接抛异常。
  if (detectCollision(start_pose_) || detectCollision(goal_pose_)) {
    throw std::logic_error("Invalid start or goal pose");
  }

  // 反向搜索会从目标往起点扩展。几何路径最终仍会在 setPath() 中整理回执行顺序。
  if (is_backward_search_) std::swap(start_pose_, goal_pose_);

  // 从 goal_pose_ 开始在二维栅格上扩散，得到每个格子到目标的无碰撞近似距离。
  // estimateCost() 会把这个距离与 Reeds-Shepp 距离结合，形成 Hybrid A* 启发函数。
  setCollisionFreeDistanceMap();

  // 当前重载只处理单目标；多目标版本会把该标志置为 true 并维护 alternate_goals_。
  is_multiple_goals_ = false;

  // 将 start_pose_ 离散成 (x, y, theta) 索引，创建第一个 AstarNode 并放入 open list。
  setStartNode();

  // 执行 A* 主循环：不断弹出总代价 fc 最小的节点，扩展运动原语，直到到达目标。
  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
  }

  // search() 成功时已经通过 setPath() 回溯父节点，并写入 waypoints_。
  return true;
}

bool AstarSearch::makePlan(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<geometry_msgs::msg::Pose> & goal_candidates)
{
  if (goal_candidates.empty()) return false;

  if (goal_candidates.size() == 1) {
    return makePlan(start_pose, goal_candidates.front());
  }

  resetData();

  start_pose_ = global2local(costmap_, start_pose);

  std::vector<Pose> goals_local;
  for (const auto & goal : goal_candidates) {
    const auto goal_local = global2local(costmap_, goal);
    if (detectCollision(goal_local)) continue;
    goals_local.push_back(goal_local);
  }

  if (detectCollision(start_pose_) || goals_local.empty()) {
    throw std::logic_error("Invalid start or goal pose");
  }

  goal_pose_ = is_backward_search_ ? start_pose_ : goals_local.front();

  setCollisionFreeDistanceMap();

  is_multiple_goals_ = true;

  if (is_backward_search_) {
    double cost_offset = 0.0;
    for (const auto & pose : goals_local) {
      start_pose_ = pose;
      setStartNode(cost_offset);
      cost_offset += multi_goal_backward_cost_offset;
    }
  } else {
    setStartNode();
    alternate_goals_ = goals_local;
  }

  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
  }

  return true;
}

/// @brief 从目标点反向计算二维自由空间距离图，作为 Hybrid A* 启发函数的一部分。
///
/// col_free_distance_map_[id] 表示：在二维 costmap 上，从某个栅格到目标栅格的近似最短
/// 无碰撞距离。这里使用的是 Dijkstra 风格的传播：
/// - 从 goal_pose_ 对应的二维栅格开始，距离为 0。
/// - 向 8 邻域扩展。
/// - 障碍物、越界格子、离障碍物太近的格子不参与传播。
/// - 横纵移动代价为 resolution，对角移动代价为 sqrt(2) * resolution。
///
/// 注意：这里是二维启发距离图，不是最终的完整车辆碰撞检测。真正的车辆矩形 footprint
/// 碰撞检查仍然发生在 expandNodes() 中的 detectCollision(next_index)。
/// 这里用半车宽净空过滤格子，是为了让启发函数避开明显过窄的区域。
void AstarSearch::setCollisionFreeDistanceMap()
{
  // 堆中的元素：二维栅格索引 + 从该栅格到目标的当前最短距离估计。
  using Entry = std::pair<IndexXY, double>;
  struct CompareEntry
  {
    // priority_queue 默认是大顶堆；这里反过来比较，使距离最小的元素优先弹出。
    bool operator()(const Entry & a, const Entry & b) const { return a.second > b.second; }
  };
  std::priority_queue<Entry, std::vector<Entry>, CompareEntry> heap;
  // closed[id] 表示该二维格子到目标的最短距离已经确定，不需要再次扩展。
  std::vector<bool> closed(col_free_distance_map_.size(), false);

  // 将目标位姿离散成栅格索引。这里只使用 x/y，theta 不参与二维距离传播。
  auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  // Dijkstra 从目标格子开始反向传播，因此目标到目标的距离为 0。
  col_free_distance_map_[indexToId(goal_index)] = 0.0;
  heap.push({IndexXY{goal_index.x, goal_index.y}, 0.0});

  Entry current;
  // offsets = {-1, 0, 1}，双层循环后会形成当前格子的 8 邻域和自身。
  std::array<int, 3> offsets = {1, 0, -1};
  while (!heap.empty()) {
    // 每次取出当前距离最小的格子，这就是 Dijkstra 的核心。
    current = heap.top();
    heap.pop();
    const int id = indexToId(current.first);
    if (closed[id]) continue;
    closed[id] = true;

    const auto & index = current.first;
    for (const auto & offset_x : offsets) {
      const int x = index.x + offset_x;
      for (const auto & offset_y : offsets) {
        const int y = index.y + offset_y;
        const IndexXY n_index{x, y};
        // offset 用于区分自身、横纵邻居、对角邻居：
        // offset == 0 表示自身；offset == 1 表示横纵移动；offset == 2 表示对角移动。
        const double offset = std::abs(offset_x) + std::abs(offset_y);
        // 跳过越界格、障碍物格和自身格。
        if (isOutOfRange(n_index) || isObs(n_index) || offset < 1) continue;
        // 跳过离障碍物小于半车宽的格子，避免启发距离穿过明显过窄的通道。
        if (getObstacleEDT(n_index).distance < 0.5 * collision_vehicle_shape_.width) continue;
        const int n_id = indexToId(n_index);
        // 横纵移动增加 1 个 resolution；对角移动增加 sqrt(2) 个 resolution。
        const double dist = current.second + (sqrt(offset) * costmap_.info.resolution);
        // 如果该格子已确定最短距离，或者已有更短路径，则无需更新。
        if (closed[n_id] || col_free_distance_map_[n_id] < dist) continue;
        // 找到更短距离，更新距离图并放入堆，等待后续继续向外传播。
        col_free_distance_map_[n_id] = dist;
        heap.push({n_index, dist});
      }
    }
  }
}

void AstarSearch::setStartNode(const double cost_offset)
{
  /// @brief 创建并初始化 A* 的起点节点。
  /// @param cost_offset 起点初始代价偏移量，主要用于多目标/反向搜索时给不同起点加区分度，
  ///        避免多个起点在 open list 中完全同价导致的搜索偏置。
  ///
  /// 这个函数不做搜索，只做初始化：
  /// 1. 将 start_pose_ 离散到 costmap 栅格索引。
  /// 2. 在 graph_ 中找到该索引对应的 AstarNode。
  /// 3. 计算起点的初始总代价 initial_cost = 启发代价 + cost_offset。
  /// 4. 将起点节点标记为 Open，并设置 parent = nullptr。
  /// 5. 把起点压入 openlist_，作为 search() 的第一个待扩展节点。
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  // graph_ 预先按所有 (x, y, theta) 组合分配好了空间，这里直接取出对应节点地址。
  AstarNode * start_node = &graph_[getKey(index)];
  // 起点的总初始代价由启发函数 estimateCost() 给出，再叠加 cost_offset。
  // 这样做可以让多目标或多起点场景下，不同起点的优先级稍微拉开。
  const double initial_cost = estimateCost(start_pose_, index) + cost_offset;
  // move_cost=0.0 表示还没有走任何实际运动；steering=0 表示初始转角未知/默认直行。
  start_node->set(start_pose_, 0.0, initial_cost, 0, false);
  // 起点刚加入搜索时，方向累计距离为 0，后续用于估计换挡惩罚。
  start_node->dir_distance = 0.0;
  // 记录起点到目标的几何距离，后续扩展时会用来调整扩展步长。
  start_node->dist_to_goal = calc_distance2d(start_pose_, goal_pose_);
  // 记录起点到最近障碍物的 EDT 距离，后续也会用于自适应扩展和障碍物代价。
  start_node->dist_to_obs = getObstacleEDT(index).distance;
  // 起点已经进入 open list 等待扩展。
  start_node->status = NodeStatus::Open;
  // 起点没有父节点，因为它就是搜索树的根。
  start_node->parent = nullptr;

  // 把起点节点压入优先队列，search() 会从这里开始扩展。
  openlist_.push(start_node);
}

double AstarSearch::estimateCost(const Pose & pose, const IndexXYT & index) const
{
  double total_cost = col_free_distance_map_[indexToId(index)];
  // Temporarily, until reeds_shepp gets stable.
  if (use_reeds_shepp_) {
    total_cost =
      std::max(total_cost, calcReedsSheppDistance(pose, goal_pose_, avg_turning_radius_));
  }
  return astar_param_.distance_heuristic_weight * total_cost;
}

bool AstarSearch::search()
{
  /// @brief A* / Hybrid A* 主搜索循环。
  ///
  /// 这个函数只做搜索控制，不直接生成几何边：
  /// 1. 记录搜索开始时间，用于超时保护。
  /// 2. 不断从 openlist_ 中取出 fc 最小的节点。
  /// 3. 跳过已经被关闭的重复节点。
  /// 4. 检查当前节点是否已经满足目标条件。
  /// 5. 若未到达目标，则扩展前进方向；必要时也扩展倒车方向。
  /// 6. openlist_ 为空时仍未找到目标，则返回失败。
  ///
  /// 成功结束时，setPath() 已经把目标到起点的父链回溯成 waypoints_。
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // openlist_ 为空意味着没有待扩展节点，通常表示搜索失败或已经结束。
  while (!openlist_.empty()) {
    // 每轮都检查耗时，防止在复杂障碍环境里搜索过久影响实时性。
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    // 取出当前 open list 中总代价 fc 最小的节点。
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    // 同一节点可能因为更优路径被重新放入 open list，旧条目会留在堆里。
    // 如果它已经被关闭，说明更优版本已处理过，直接跳过。
    if (current_node->status == NodeStatus::Closed) continue;
    // 当前节点即将被展开，标记为 Closed，防止重复扩展。
    current_node->status = NodeStatus::Closed;

    // 若当前节点已经进入目标窗口，就可以停止搜索并回溯路径。
    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    // 扩展前进方向的运动原语：转角离散 + 自行车模型积分 + 碰撞检测 + 代价更新。
    expandNodes(*current_node);
    // 如果允许倒车，再额外扩展一组倒车运动原语。
    if (astar_param_.use_back) expandNodes(*current_node, true);
  }

  // openlist_ 被耗尽仍未找到目标，说明当前地图/参数下不存在可行路径或搜索没能覆盖到。
  return false;
}

void AstarSearch::expandNodes(AstarNode & current_node, const bool is_back)
{
  /// @brief 从当前节点向外扩展一组车辆运动原语。
  ///
  /// @param current_node 当前正在展开的 AstarNode。
  /// @param is_back false 表示扩展前进动作，true 表示扩展倒车动作。
  ///
  /// 这个函数是 Hybrid A* 的“车辆化扩展”核心，流程如下：
  /// 1. 把当前节点转回连续位姿。
  /// 2. 决定本次扩展是前进还是倒车，以及对应的行驶方向符号。
  /// 3. 枚举所有离散转角。
  /// 4. 用运动学自行车模型积分出下一个位姿。
  /// 5. 将新位姿离散成栅格索引并检查越界/占据/整车碰撞。
  /// 6. 计算新的实际代价 gc 和总代价 fc。
  /// 7. 如果这个节点更优，就更新它的父节点并放入 openlist_。
  const auto current_pose = node2pose(current_node);
  // is_back 表示这次扩展选择的是“前进还是倒车”。
  // is_backward_search_ 表示当前搜索本身是不是反向搜索。
  // 两者相同则说明本次扩展的运动方向与搜索方向一致，距离取正；否则取负。
  const double direction = (is_back == is_backward_search_) ? 1.0 : -1.0;
  // 当前扩展步长可能自适应变化；再乘方向符号后，得到带正负号的行驶距离。
  const double distance = getExpansionDistance(current_node) * direction;
  // 从最左转到最右转枚举离散转角。
  int steering_index = -1 * planner_common_param_.turning_steps;
  for (; steering_index <= planner_common_param_.turning_steps; ++steering_index) {
    // 如果当前节点不是根节点，且这次扩展切换了前进/倒车方向，
    // 同时转角又和父节点完全一致，那么大概率只是“原路倒回去”，直接跳过。
    if (
      current_node.parent != nullptr && is_back != current_node.is_back &&
      steering_index == current_node.steering_index) {
      continue;
    }

    // 把整数转角索引转换成真实前轮转角。
    const double steering = static_cast<double>(steering_index) * steering_resolution_;
    // 用运动学自行车模型积分出下一时刻的连续位姿。
    const auto next_pose = kinematic_bicycle_model::getPose(
      current_pose, collision_vehicle_shape_.base_length, steering, distance);
    // 将连续位姿离散到 costmap 的 (x, y, theta) 索引。
    const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    // 先做最便宜的过滤：越界或目标格本身是障碍物，直接丢弃。
    if (isOutOfRange(next_index) || isObs(next_index)) continue;

    // next_node 指向 graph_ 中对应离散状态的槽位。
    // 如果该状态已经被 Closed，或者完整车身碰撞，则不再继续扩展它。
    AstarNode * next_node = &graph_[getKey(next_index)];
    if (next_node->status == NodeStatus::Closed || detectCollision(next_index)) continue;

    // 获取该格子的 EDT 距离和方向信息，用于障碍物距离代价。
    const auto obs_edt = getObstacleEDT(next_index);
    // 如果这次扩展的方向和父节点不同，说明发生了“前进/倒车切换”。
    const bool is_direction_switch =
      (current_node.parent != nullptr) && (is_back != current_node.is_back);

    // total_weight 是当前一步的基础距离权重：
    // 1.0 表示基础行驶距离
    // getSteeringCost() 让大转角略贵
    // 如果是倒车，再额外乘以 reverse_weight
    double total_weight = 1.0;
    total_weight += getSteeringCost(steering_index);
    if (is_back) total_weight *= (1.0 + planner_common_param_.reverse_weight);

    // 从父节点累计实际代价 gc 开始，叠加这一步的各种局部代价。
    double move_cost = current_node.gc + (total_weight * std::abs(distance));
    // 转角变化越大，代价越高，鼓励方向盘动作更平滑。
    move_cost += getSteeringChangeCost(steering_index, current_node.steering_index);
    // 越靠近障碍物，代价越高，即使还没碰撞也会被惩罚。
    move_cost += getObsDistanceCost(next_index, obs_edt);
    // 接近目标时，横向偏差越大，代价越高，鼓励最后一段对准目标。
    move_cost += getLatDistanceCost(next_pose);
    // 若刚发生前进/倒车切换，再加一次方向切换惩罚。
    if (is_direction_switch) move_cost += getDirectionChangeCost(current_node.dir_distance);

    // f = g + h。这里 g 是实际累计代价，h 是启发式估计剩余代价。
    double total_cost = move_cost + estimateCost(next_pose, next_index);
    // 如果这个离散状态从未访问过，或者新路径比旧路径更优，就更新它。
    if (next_node->status == NodeStatus::None || next_node->fc > total_cost) {
      // 先把该节点标记为 Open，表示它进入待扩展集合。
      next_node->status = NodeStatus::Open;
      // 保存下一位姿、实际代价、总代价、转角索引和前进/倒车方向。
      next_node->set(next_pose, move_cost, total_cost, steering_index, is_back);
      // 记录从最近一次方向切换以来已经走了多少距离。
      next_node->dir_distance =
        std::abs(distance) + (is_direction_switch ? 0.0 : current_node.dir_distance);
      // 记录该节点到目标的几何距离，后面 getExpansionDistance() 会用到。
      next_node->dist_to_goal = calc_distance2d(next_pose, goal_pose_);
      // 记录该节点到最近障碍物的 EDT 距离，后面也会影响扩展步长和代价。
      next_node->dist_to_obs = obs_edt.distance;
      // 父节点指针用于最终回溯整条路径。
      next_node->parent = &current_node;
      // 放入优先队列，等待 future search 轮次里被取出继续扩展。
      openlist_.push(next_node);
      continue;
    }
  }
}

double AstarSearch::getExpansionDistance(const AstarNode & current_node) const
{
  if (!astar_param_.adapt_expansion_distance || max_expansion_dist_ <= min_expansion_dist_) {
    return min_expansion_dist_;
  }
  double exp_dist = std::min(
    current_node.dist_to_goal * dist_to_goal_expansion_factor_,
    current_node.dist_to_obs * dist_to_obs_expansion_factor_);
  return std::clamp(exp_dist, min_expansion_dist_, max_expansion_dist_);
}

double AstarSearch::getSteeringCost(const int steering_index) const
{
  return planner_common_param_.curve_weight *
         (static_cast<double>(abs(steering_index)) / planner_common_param_.turning_steps);
}

double AstarSearch::getSteeringChangeCost(
  const int steering_index, const int prev_steering_index) const
{
  double steering_index_diff = abs(steering_index - prev_steering_index);
  return astar_param_.smoothness_weight * steering_index_diff /
         (2.0 * planner_common_param_.turning_steps);
}

double AstarSearch::getDirectionChangeCost(const double dir_distance) const
{
  return planner_common_param_.direction_change_weight * (1.0 + (1.0 / (1.0 + dir_distance)));
}

double AstarSearch::getObsDistanceCost(const IndexXYT & index, const EDTData & obs_edt) const
{
  if (obs_edt.distance > collision_vehicle_shape_.max_dimension + cost_free_obs_dist) {
    return 0.0;
  }
  const double yaw = index.theta * (2.0 * M_PI / planner_common_param_.theta_size);
  const double base_to_frame_dist = getVehicleBaseToFrameDistance(yaw - obs_edt.angle);
  const double vehicle_to_obs_dist = std::max(obs_edt.distance - base_to_frame_dist, 0.0);
  return astar_param_.obstacle_distance_weight *
         std::max(1.0 - (vehicle_to_obs_dist / cost_free_obs_dist), 0.0);
}

double AstarSearch::getLatDistanceCost(const Pose & pose) const
{
  if (is_multiple_goals_) return 0.0;
  const auto ref_pose = is_backward_search_ ? start_pose_ : goal_pose_;
  const double distance_to_goal = calc_distance2d(pose, ref_pose);
  if (distance_to_goal > near_goal_dist_) return 0.0;
  const double lat_distance = std::abs(calcRelativePose(ref_pose, pose).position.y);
  return astar_param_.goal_lat_distance_weight * lat_distance;
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  std_msgs::msg::Header header;
  header.stamp = clock_->now();
  header.frame_id = costmap_.header.frame_id;

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  std::vector<PlannerWaypoint> waypoints;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  if (shifted_goal_pose_) {
    pose.pose = local2global(costmap_, shifted_goal_pose_.get());
    waypoints.push_back({pose, goal_node.is_back});
  }

  const auto interpolate = [this, &waypoints, &pose](const AstarNode & node) {
    if (node.parent == nullptr || !astar_param_.adapt_expansion_distance) return;
    const auto parent_pose = node2pose(*node.parent);
    const double distance_2d = calc_distance2d(node2pose(node), parent_pose);
    const int n = static_cast<int>(distance_2d / min_expansion_dist_);
    for (int i = 1; i < n; ++i) {
      const double dist =
        ((distance_2d * i) / n) * (node.is_back == is_backward_search_ ? 1.0 : -1.0);
      const double steering = node.steering_index * steering_resolution_;
      const auto local_pose = kinematic_bicycle_model::getPose(
        parent_pose, collision_vehicle_shape_.base_length, steering, dist);
      pose.pose = local2global(costmap_, local_pose);
      waypoints.push_back({pose, node.is_back});
    }
  };

  // push astar nodes poses
  while (node != nullptr) {
    pose.pose = local2global(costmap_, node2pose(*node));
    waypoints.push_back({pose, node->is_back});
    interpolate(*node);
    // To the next node
    node = node->parent;
  }

  if (waypoints.empty()) return;

  if (waypoints.size() > 1) waypoints.back().is_back = waypoints.rbegin()[1].is_back;

  if (!is_backward_search_) {
    // Reverse the vector to be start to goal order
    std::reverse(waypoints.begin(), waypoints.end());
  }

  waypoints_.header = header;
  waypoints_.waypoints = waypoints;

  if (!is_backward_search_) return;

  for (size_t i = 0; i < waypoints_.waypoints.size() - 1; ++i) {
    const auto & current = waypoints_.waypoints[i];
    auto & next = waypoints_.waypoints[i + 1];

    if (current.is_back != next.is_back) {
      next.is_back = current.is_back;
      ++i;  // skip next waypoint
    }
  }
}

bool AstarSearch::isGoal(const AstarNode & node) const
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle = autoware_utils::deg2rad(planner_common_param_.angle_goal_range / 2.0);

  const auto node_pose = node2pose(node);

  auto checkGoal = [this, &node_pose, &lateral_goal_range, &longitudinal_goal_range, &goal_angle,
                    &is_back = node.is_back](const Pose & pose) {
    const auto node_index = pose2index(costmap_, node_pose, planner_common_param_.theta_size);
    const auto goal_index = pose2index(costmap_, pose, planner_common_param_.theta_size);

    if (node_index == goal_index) return true;

    const auto relative_pose = calcRelativePose(pose, node_pose);

    bool is_behind_goal = relative_pose.position.x <= 0.0;

    if (astar_param_.only_behind_solutions && !is_behind_goal) {
      return false;
    }

    if (
      std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
      std::fabs(relative_pose.position.y) > lateral_goal_range) {
      return false;
    }

    const auto angle_diff =
      autoware_utils::normalize_radian(tf2::getYaw(relative_pose.orientation));
    if (std::abs(angle_diff) > goal_angle) {
      return false;
    }

    const bool is_set_shifted_goal_pose =
      is_backward_search_ ? is_behind_goal == is_back : is_behind_goal != is_back;
    if (is_set_shifted_goal_pose) {
      setShiftedGoalPose(pose, relative_pose.position.y);
    }

    return true;
  };

  if (checkGoal(goal_pose_)) return true;

  return std::any_of(
    alternate_goals_.begin(), alternate_goals_.end(),
    [&checkGoal](const Pose & pose) { return checkGoal(pose); });
}

void AstarSearch::setShiftedGoalPose(const Pose & goal_pose, const double lat_offset) const
{
  tf2::Transform tf;
  tf2::convert(goal_pose, tf);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf);

  Pose lat_pose;
  lat_pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(lat_offset).z(0.0);
  lat_pose.orientation =
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0);

  shifted_goal_pose_ = transformPose(lat_pose, transform);
}

Pose AstarSearch::node2pose(const AstarNode & node) const
{
  Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  pose_local.orientation = autoware_utils::create_quaternion_from_yaw(node.theta);

  return pose_local;
}

}  // namespace autoware::freespace_planning_algorithms
