是的，`mission_planner.cpp` 是路线设置的最后接收者之一，但中间还有几层桥接。可以分成两条链路看：

**2D Pose Estimate**
RViz 的 `SetInitialPose` 发布 `/initialpose`，类型是 `geometry_msgs/PoseWithCovarianceStamped`，见 [autoware.rviz](/home/erge/work/autoware_docker/autoware_devel/src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz:4011)。

传递链路大概是：

```text
RViz 2D Pose Estimate
  -> /initialpose
  -> initial_pose_adaptor_node
  -> /api/localization/initialize
  -> default_adapi localization
  -> /localization/initialize
  -> pose_initializer
  -> /initialpose3d
  -> simple_planning_simulator
  -> /localization/kinematic_state
  -> mission_planner ~/input/odometry
```

所以初始位姿不是直接给 `mission_planner` 的。它先初始化仿真车辆位置，之后 simulator 发布 `/localization/kinematic_state`，`mission_planner` 用这个 odometry 作为路径起点。对应订阅在 [mission_planner.cpp](/home/erge/work/autoware_docker/autoware_devel/src/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp:86)。

**2D Goal Pose**
这里有两个入口：

```text
RViz 标准 2D Goal Pose
  -> /planning/mission_planning/goal
  -> routing_adaptor on_fixed_goal()
  -> /api/routing/set_route_points
  -> default_adapi RoutingNode
  -> /planning/set_waypoint_route
  -> RouteSelector
  -> /planning/mission_planning/mission_planner/set_waypoint_route
  -> MissionPlanner::on_set_waypoint_route()
```

对应 RViz topic 在 [autoware.rviz](/home/erge/work/autoware_docker/autoware_devel/src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz:4021)。`routing_adaptor` 的 remap 在 [rviz_adaptors.launch.xml](/home/erge/work/autoware_docker/autoware_devel/src/core/autoware_core/api/autoware_adapi_adaptors/launch/rviz_adaptors.launch.xml:15)，`on_fixed_goal()` 会把它包装成 `SetRoutePoints` 请求，且 `allow_goal_modification=false`，见 [routing_adaptor.cpp](/home/erge/work/autoware_docker/autoware_devel/src/core/autoware_core/api/autoware_adapi_adaptors/src/routing_adaptor.cpp:88)。

Autoware 自定义的 `2D Rough Goal Pose` 走：

```text
RViz 2D Rough Goal Pose
  -> /rviz/routing/rough_goal
  -> routing_adaptor on_rough_goal()
  -> 后面同样进入 /api/routing/set_route_points
```

这个工具在 [route_tool.cpp](/home/erge/work/autoware_docker/autoware_devel/src/universe/autoware_universe/visualization/tier4_adapi_rviz_plugin/src/route_tool.cpp:25)，它继承 RViz 的 `GoalTool`，真正发布 `PoseStamped` 的逻辑来自父类；topic 配在 RViz 文件里。

最后在 `mission_planner` 里，核心就是 [MissionPlanner::on_set_waypoint_route()](/home/erge/work/autoware_docker/autoware_devel/src/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp:346)。它拿到：

```text
req->goal_pose     // RViz 点击的目标点
req->waypoints     // checkpoint，可选
odometry_->pose    // 当前车位置，来自 /localization/kinematic_state
```

然后在 [create_route()](/home/erge/work/autoware_docker/autoware_devel/src/universe/autoware_universe/planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp:451) 里组装成：

```text
start_pose = 当前 odometry
waypoints  = 中间点
goal_pose  = 目标点
```

再调用 `planner_->plan(points)` 生成 Lanelet route。

一个实用判断：必须先设置 `2D Pose Estimate`，因为 `mission_planner` 会等地图和 odometry 都准备好才进入可用状态；否则点目标时可能报 “mission planner is not ready”。