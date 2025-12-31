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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "planner_manager.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/approval.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_internal_planning_msgs::msg::Scenario;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LateralOffset;
using tier4_planning_msgs::msg::RerouteAvailability;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using DebugPublisher = autoware_utils::DebugPublisher;

class BehaviorPathPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options);

  // Getter method for waiting approval modules
  std::vector<std::string> getWaitingApprovalModules();

  // Getter method for running modules
  std::vector<std::string> getRunningModules();

private:
// 1. route_subscriber_
// 消息类型: LaneletRoute
// 话题: ~/input/route
// QoS: transient_local() (1)
// 作用: 获取从路径规划器传来的路线信息，包括起点、终点和途经点的车道网格式路线，用于行为路径规划器知道车辆应该行驶到哪里
// 2. vector_map_subscriber_
// 消息类型: LaneletMapBin
// 话题: ~/input/vector_map
// QoS: transient_local() (1)
// 作用: 获取车道网格式地图数据，用于路径规划时了解道路结构、车道连接关系、交通规则等信息
// 3. velocity_subscriber_
// 消息类型: Odometry
// 话题: ~/input/odometry
// 作用: 获取车辆当前的位姿和速度信息，用于确定车辆在地图中的位置和当前运动状态
// 4. acceleration_subscriber_
// 消息类型: AccelWithCovarianceStamped
// 话题: ~/input/accel
// 作用: 获取车辆的加速度信息，用于更精确的运动规划和控制
// 5. scenario_subscriber_
// 消息类型: Scenario
// 话题: ~/input/scenario
// 作用: 获取当前驾驶场景信息（如车道驾驶、停车等），行为路径规划器只在 LANEDRIVING 场景下工作
// 6. perception_subscriber_
// 消息类型: PredictedObjects
// 话题: ~/input/perception
// 作用: 获取感知系统检测到的动态物体（车辆、行人等）及其预测轨迹，用于避障和安全路径规划
// 7. occupancy_grid_subscriber_
// 消息类型: OccupancyGrid
// 话题: ~/input/occupancy_grid_map
// 作用: 获取占用栅格地图，用于了解环境中的静态和动态障碍物分布
// 8. costmap_subscriber_
// 消息类型: OccupancyGrid
// 话题: ~/input/costmap
// 作用: 获取成本地图，用于评估路径的代价，帮助规划器选择最优路径
// 9. traffic_signals_subscriber_
// 消息类型: TrafficLightGroupArray
// 话题: ~/input/traffic_signals
// 作用: 获取交通信号灯信息，用于在交叉口等场景下做出正确的行为决策
// 10. lateral_offset_subscriber_
// 消息类型: LateralOffset
// 话题: ~/input/lateral_offset
// 作用: 获取横向偏移指令，用于调整车辆在车道中的横向位置
// 11. operation_mode_subscriber_
// 消息类型: OperationModeState
// 话题: /system/operation_mode/state
// QoS: transient_local() (1)
// 作用: 获取操作模式状态，确定车辆是否处于自动驾驶模式，是否启用Autoware控制
// 12. external_limit_max_velocity_subscriber_
// 消息类型: VelocityLimit
// 话题: /planning/scenario_planning/max_velocity
// 作用: 获取外部速度限制，用于确保路径规划不超过特定区域的速度限制
  // subscriber
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<Odometry> velocity_subscriber_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "~/input/accel"};
  autoware_utils::InterProcessPollingSubscriber<Scenario> scenario_subscriber_{
    this, "~/input/scenario"};
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> perception_subscriber_{
    this, "~/input/perception"};
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_subscriber_{
    this, "~/input/occupancy_grid_map"};
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> costmap_subscriber_{
    this, "~/input/costmap"};
  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> traffic_signals_subscriber_{
    this, "~/input/traffic_signals"};
  autoware_utils::InterProcessPollingSubscriber<LateralOffset> lateral_offset_subscriber_{
    this, "~/input/lateral_offset"};
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> operation_mode_subscriber_{
    this, "/system/operation_mode/state", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<autoware_internal_planning_msgs::msg::VelocityLimit>
    external_limit_max_velocity_subscriber_{this, "/planning/scenario_planning/max_velocity"};

  // publisher
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_signal_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr bound_publisher_;
  rclcpp::Publisher<PoseWithUuidStamped>::SharedPtr modified_goal_publisher_;
  rclcpp::Publisher<RerouteAvailability>::SharedPtr reroute_availability_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_candidate_publishers_;
  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_reference_publishers_;

  std::shared_ptr<PlannerData> planner_data_;
  Scenario::ConstSharedPtr current_scenario_{nullptr};
  LaneletMapBin::ConstSharedPtr map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  bool has_received_map_{false};
  bool has_received_route_{false};

  std::shared_ptr<PlannerManager> planner_manager_;

  std::unique_ptr<PlanningFactorInterface> planning_factor_interface_;

  std::mutex mutex_pd_;       // mutex for planner_data_
  std::mutex mutex_manager_;  // mutex for bt_manager_ or planner_manager_

  // setup
  void takeData();
  bool isDataReady();

  // callback
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onCostMap(const OccupancyGrid::ConstSharedPtr msg);
  void onTrafficSignals(const TrafficLightGroupArray::ConstSharedPtr msg);
  void onMap(const LaneletMapBin::ConstSharedPtr map_msg);
  void onRoute(const LaneletRoute::ConstSharedPtr route_msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onLateralOffset(const LateralOffset::ConstSharedPtr msg);
  void on_external_velocity_limiter(
    const autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);

  SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
  static PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data);

  /**
   * @brief skip smooth goal connection
   */
  void computeTurnSignal(
    const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
    const BehaviorModuleOutput & output);

  // debug
  rclcpp::Publisher<AvoidanceDebugMsgArray>::SharedPtr debug_avoidance_msg_array_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_turn_signal_info_publisher_;
  std::unique_ptr<DebugPublisher> debug_start_planner_evaluation_table_publisher_ptr_;

  /**
   * @brief publish reroute availability
   */
  void publish_reroute_availability() const;

  /**
   * @brief publish steering factor from intersection
   */
  void publish_steering_factor(
    const std::shared_ptr<PlannerData> & planner_data, const TurnIndicatorsCommand & turn_signal);

  /**
   * @brief publish turn signal debug info
   */
  void publish_turn_signal_debug_data(const TurnSignalDebugData & debug_data);

  /**
   * @brief publish left and right bound
   */
  void publish_bounds(const PathWithLaneId & path);

  /**
   * @brief publish debug messages
   */
  void publishSceneModuleDebugMsg(
    const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr);

  /**
   * @brief publish path candidate
   */
  void publishPathCandidate(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  void publishPathReference(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  /**
   * @brief convert path with lane id to path for publish path candidate
   */
  Path convertToPath(
    const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready,
    const std::shared_ptr<PlannerData> & planner_data);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
