// Copyright 2019 Autoware Foundation
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/srv/load_plugin.hpp>
#include <autoware_internal_planning_msgs/srv/unload_plugin.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::srv::LoadPlugin;
using autoware_internal_planning_msgs::srv::UnloadPlugin;
using autoware_map_msgs::msg::LaneletMapBin;

class BehaviorVelocityPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscriber
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;

  // polling subscribers
// 1. sub_predicted_objects_
// 消息类型：autoware_perception_msgs::msg::PredictedObjects
// 主题：~/input/dynamic_objects
// 数据内容：预测的动态物体信息，包括：
// 检测到的车辆、行人、自行车等
// 每个物体的位置、速度、加速度
// 预测的未来轨迹
// 来源：感知模块（如目标检测和跟踪模块）
// 2. sub_no_ground_pointcloud_
// 消息类型：sensor_msgs::msg::PointCloud2
// 主题：~/input/no_ground_pointcloud
// 数据内容：去除地面后的点云数据，包含：
// 环境中障碍物的3D点云
// 没有地面点的干净点云数据
// QoS：使用 single_depth_sensor_qos() 配置，适合传感器数据
// 来源：激光雷达（LiDAR）传感器，经过地面分割算法处理
// 3. sub_vehicle_odometry_
// 消息类型：nav_msgs::msg::Odometry
// 主题：~/input/vehicle_odometry
// 数据内容：车辆里程计信息，包括：
// 车辆的位姿（位置和方向）
// 车辆的线速度和角速度
// 来源：车辆的里程计系统或定位模块（如IMU+轮速计融合）
// 4. sub_acceleration_
// 消息类型：geometry_msgs::msg::AccelWithCovarianceStamped
// 主题：~/input/accel
// 数据内容：车辆加速度信息，包括：
// 线加速度和角加速度
// 协方差矩阵（表示测量不确定性）
// 来源：IMU（惯性测量单元）或车辆CAN总线
// 5. sub_traffic_signals_
// 消息类型：autoware_perception_msgs::msg::TrafficLightGroupArray
// 主题：~/input/traffic_signals
// 数据内容：交通信号灯组信息，包括：
// 各个交通灯组的ID
// 灯的颜色状态（红、黄、绿）
// 信号变化的时间信息
// 来源：交通信号识别模块或V2X通信系统
// 6. sub_occupancy_grid_
// 消息类型：nav_msgs::msg::OccupancyGrid
// 主题：~/input/occupancy_grid
// 数据内容：占用网格地图，包括：
// 2D网格表示的环境
// 每个网格的占用概率（0-100，-1表示未知）
// 来源：占用网格生成模块（通常基于传感器数据生成）
// 7. sub_lanelet_map_
// 消息类型：LaneletMapBin (即 autoware_map_msgs::msg::LaneletMapBin)
// 主题：~/input/vector_map
// 数据内容：车道网络地图，包括：
// 车道、道路边界、交通规则
// 路点、车道连接关系
// 交通标志和信号灯位置
// QoS：使用 transient_local，适合静态地图数据
// 策略：polling_policy::Newest，总是获取最新数据
// 来源：地图加载模块或地图服务器
// 8. sub_external_velocity_limit_
// 消息类型：VelocityLimit
// 主题：~/input/external_velocity_limit_mps
// 数据内容：外部速度限制，包括：
// 当前区域的速度上限
// 可能来自交通标志或特殊区域的限制
// QoS：使用 transient_local，适合配置数据
// 来源：速度限制管理模块或交通标志识别模块
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::PredictedObjects>
    sub_predicted_objects_{this, "~/input/dynamic_objects"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<sensor_msgs::msg::PointCloud2>
    sub_no_ground_pointcloud_{
      this, "~/input/no_ground_pointcloud", autoware_utils_rclcpp::single_depth_sensor_qos()};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    sub_vehicle_odometry_{this, "~/input/vehicle_odometry"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/accel"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_signals_{this, "~/input/traffic_signals"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    sub_occupancy_grid_{this, "~/input/occupancy_grid"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest>
    sub_lanelet_map_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit> sub_external_velocity_limit_{
    this, "~/input/external_velocity_limit_mps", rclcpp::QoS{1}.transient_local()};

  void onTrigger(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);

  void onParam();

  void processNoGroundPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void processTrafficSignals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg);
  bool processData(rclcpp::Clock clock);

  // publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;

  void publishDebugMarker(const autoware_planning_msgs::msg::Path & path);

  //  parameter
  double forward_path_length_;
  double backward_path_length_;
  double behavior_output_path_interval_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  bool is_driving_forward_{true};

  rclcpp::Service<LoadPlugin>::SharedPtr srv_load_plugin_;
  rclcpp::Service<UnloadPlugin>::SharedPtr srv_unload_plugin_;
  void onUnloadPlugin(
    const UnloadPlugin::Request::SharedPtr request,
    const UnloadPlugin::Response::SharedPtr response);
  void onLoadPlugin(
    const LoadPlugin::Request::SharedPtr request, const LoadPlugin::Response::SharedPtr response);

  // mutex for planner_data_
  std::mutex mutex_;

  // function
  bool isDataReady(rclcpp::Clock clock);
  autoware_planning_msgs::msg::Path generatePath(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
    const PlannerData & planner_data);

  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  static constexpr int logger_throttle_interval = 3000;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
