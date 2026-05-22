# Autoware 规划控制层端到端实战复盘：视频脚本与 PPT 大纲

> 面向对象：刚学 Autoware 规划控制的新手  
> 推荐时长：60 到 90 分钟  
> 主线目标：用一个从“设置目标点”到“底盘控制命令”的完整案例，把 Route、Path、Trajectory、Velocity、Control 串起来  
> 本文用途：录课脚本、RViz 演示清单、topic 排查表、PPT 大纲

---

## 0. 使用说明

这份文档不是新的算法讲义，而是一集实战复盘课。

前 8 集已经分别讲过：

```text
Lanelet / Mission Planner
Hybrid A* Pull Out
Elastic Band
Velocity Smoother
Pure Pursuit
横向 MPC
MPT
整套链路复盘
```

这一集要做的事情是：

```text
把所有模块放到同一个真实运行链路里看。
```

录课时不要急着讲源码。建议先在 RViz 中让观众看到：

```text
点击初始位姿
  -> 点击目标点
  -> Route 出现
  -> Path 出现
  -> Trajectory 出现
  -> 速度曲线出现
  -> 控制器输出转向和加速度
  -> 车辆运动
```

然后再逐层解释每一步由哪个模块完成、该观察哪些 topic、出问题时该先查哪里。

> 说明：本文中的 topic 名称根据当前项目本地 Autoware 配置和 RViz 配置整理。不同 Autoware 版本或 launch 参数可能会有命名差异，实操时建议用 `ros2 topic list | grep <关键词>` 复核。

---

## 1. 本集案例设定

### 1.1 推荐场景

建议选择一个最简单但完整的 lane driving 场景：

```text
车辆已经在 Lanelet 地图中的某条车道附近
目标点在前方一段道路上
中间可以有一个轻微弯道
不强制加入复杂红绿灯或动态障碍
```

这样做的好处：

- Route 能稳定生成。
- Behavior Path Planner 不会被太多场景模块干扰。
- MPT、Velocity Smoother 和 Controller 的输入输出更容易观察。
- 新手可以先理解主链路，再扩展到避障、红灯、换道等复杂场景。

### 1.2 可选增强场景

如果你想让课程更有层次，可以准备三个小片段：

| 片段 | 场景 | 目的 |
|---|---|---|
| A | 普通车道行驶 | 讲主链路 |
| B | 起点偏离车道，需要 Pull Out | 讲 Start Planner / Hybrid A* |
| C | 前方有停止点或障碍 | 讲速度规划、virtual wall、stop reason |

建议第一遍录制只用片段 A。

片段 B 和 C 可以作为后续补充，避免一集塞太多。

---

## 2. 录课前准备

### 2.1 RViz 必开图层

建议提前保存一个 RViz 配置，打开这些层：

| 类别 | RViz 图层或 Marker | 主要作用 |
|---|---|---|
| 地图 | `/map/vector_map_marker` | 看 Lanelet 车道、边界、停止线 |
| 点云地图 | `/map/pointcloud_map` | 看静态环境背景 |
| 定位 | 当前车辆模型、pose、odometry | 确认车辆在地图上的位置 |
| Route | `/planning/mission_planning/route_marker` | 看 Mission Planner 生成的路线 |
| Goal | `/planning/mission_planning/echo_back_goal_pose` | 看目标点是否被接收 |
| Behavior Path | `/planning/scenario_planning/lane_driving/behavior_planning/path` | 看行为规划输出路径 |
| Path Candidate | `/planning/path_candidate/*` | 看 Start Planner、Lane Change 等候选路径 |
| Path Reference | `/planning/path_reference/*` | 看不同模块的参考路径 |
| Motion Trajectory | `/planning/scenario_planning/lane_driving/trajectory` | 看运动规划后的轨迹 |
| Final Trajectory | `/planning/trajectory` | 看最终给控制器的轨迹 |
| Virtual Wall | `/planning/**/virtual_wall*` | 看停止墙、减速墙、安全墙 |
| Control Debug | `/control/trajectory_follower/**` | 看控制器预测轨迹、参考轨迹、诊断 |
| Vehicle Status | 速度、转角、档位、控制模式 | 看底盘状态是否响应 |

录课建议：

```text
先只显示地图、车辆、route、final trajectory。
讲到某一层时，再打开对应 debug 图层。
```

这样画面不会乱。

### 2.2 建议记录的 rosbag topic

如果想复盘，建议至少记录：

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /map/vector_map \
  /localization/kinematic_state \
  /localization/acceleration \
  /vehicle/status/velocity_status \
  /vehicle/status/steering_status \
  /vehicle/status/gear_status \
  /planning/mission_planning/route \
  /planning/mission_planning/route_marker \
  /planning/scenario_planning/scenario \
  /planning/scenario_planning/lane_driving/behavior_planning/path \
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory \
  /planning/scenario_planning/lane_driving/trajectory \
  /planning/scenario_planning/velocity_smoother/trajectory \
  /planning/trajectory \
  /control/trajectory_follower/control_cmd \
  /control/command/control_cmd
```

如果磁盘压力大，可以先只录：

```text
定位
车辆状态
route
behavior path
lane driving trajectory
velocity smoother trajectory
final planning trajectory
control command
```

### 2.3 推荐终端观察命令

```bash
ros2 topic list | grep planning
ros2 topic list | grep control
ros2 topic hz /planning/trajectory
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic echo /planning/mission_planning/route_state
ros2 topic echo /vehicle/status/velocity_status
ros2 topic echo /vehicle/status/steering_status
```

如果你想看整条图：

```bash
rqt_graph
```

如果你想临时找某个模块参数：

```bash
ros2 param list | grep velocity_smoother
ros2 param list | grep controller
ros2 param list | grep path_optimizer
```

---

## 3. 全链路总图

录课时建议先展示这张总图：

```text
RViz Initial Pose
  -> /initialpose
  -> Localization / Simulator
  -> /localization/kinematic_state

RViz Goal Pose
  -> /planning/mission_planning/goal
  -> Routing API
  -> Mission Planner
  -> /planning/mission_planning/route

Route
  -> Behavior Path Planner
  -> /planning/scenario_planning/lane_driving/behavior_planning/path

Behavior Path
  -> Elastic Band / Path Smoother
  -> MPT / Path Optimizer
  -> /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory

Optimized Trajectory
  -> Motion Velocity Planner
  -> /planning/scenario_planning/lane_driving/trajectory

Lane Driving Trajectory
  -> Scenario Selector
  -> Velocity Smoother
  -> /planning/scenario_planning/velocity_smoother/trajectory

Velocity Smoothed Trajectory
  -> Planning Validator
  -> /planning/trajectory

Final Trajectory
  -> Trajectory Follower
  -> /control/trajectory_follower/control_cmd
  -> Vehicle Cmd Gate
  -> /control/command/control_cmd
  -> Vehicle Interface
```

一句话总结：

```text
Route 决定经过哪些车道；
Path 决定几何上走哪条线；
Trajectory 决定每个点的速度和连续性；
Control Command 决定车辆当前怎么执行。
```

---

## 4. 视频脚本总结构

建议分成 10 个章节。

| 时间 | 内容 | 目标 |
|---|---|---|
| 0-5 分钟 | 开场和案例说明 | 告诉观众本集不是讲新公式，而是串链路 |
| 5-12 分钟 | 准备 RViz、初始位姿、目标点 | 让观众看到起点和目标点如何进入系统 |
| 12-20 分钟 | Mission Planner 和 Route | 讲 route 不是轨迹 |
| 20-30 分钟 | Behavior Path 和 Pull Out | 讲 path 是局部几何线 |
| 30-40 分钟 | Elastic Band 与 MPT | 讲路径如何变成更可跟踪的局部轨迹 |
| 40-50 分钟 | Velocity Planner 和 Velocity Smoother | 讲速度曲线如何附加到轨迹上 |
| 50-60 分钟 | Planning Validator 和最终 `/planning/trajectory` | 讲最终轨迹进入控制器前还会被检查 |
| 60-70 分钟 | Pure Pursuit / MPC 控制器 | 讲轨迹如何变成转向和加速度 |
| 70-80 分钟 | 常见故障排查 | 按症状从上游到下游排查 |
| 80-90 分钟 | 总结和学习路线 | 给观众一个后续源码阅读方法 |

如果你想控制在 45 分钟内，可以把 Pull Out、MPT 细节和故障排查压缩。

---

## 5. 第 1 段脚本：开场

### 页面目标

告诉观众这一集解决什么问题。

### 讲解脚本

```text
前面我们分别讲了 Autoware 规划控制链路里的很多模块：
Mission Planner、Hybrid A* Pull Out、Elastic Band、Velocity Smoother、Pure Pursuit、MPC 和 MPT。

如果单独看每个模块，它们都有自己的公式和代码。
但真正跑车时，它们不是孤立运行的，而是一条连续的数据链。

这一集我们不再讲新的公式，而是用一个完整案例复盘：
从 RViz 里点击一个目标点开始，
Autoware 是怎样一步步生成 route、path、trajectory，
最后变成控制命令发给车辆的。
```

### PPT 内容

标题：

```text
端到端实战复盘：目标点如何变成控制命令
```

展示：

```text
Goal -> Route -> Path -> Trajectory -> Control Command -> Vehicle
```

### RViz 操作

先不点击目标点，只展示地图和车辆当前位置。

### 观察点

- 地图是否显示。
- 车辆是否在地图上。
- TF 是否正常。
- 车辆状态是否在刷新。

---

## 6. 第 2 段脚本：初始位姿

### 6.1 讲解重点

初始位姿不是直接给 Mission Planner。

它通常先进入定位或仿真链路，最后形成：

```text
/localization/kinematic_state
```

Mission Planner 用当前 odometry 作为 route 起点。

### 6.2 数据链路

根据当前项目整理的链路：

```text
RViz 2D Pose Estimate
  -> /initialpose
  -> initial_pose_adaptor_node
  -> /api/localization/initialize
  -> /localization/initialize
  -> pose_initializer
  -> /initialpose3d
  -> simulator or localization
  -> /localization/kinematic_state
```

### 6.3 重点 topic

| topic | 看什么 |
|---|---|
| `/initialpose` | RViz 点击的初始位姿是否发出 |
| `/localization/kinematic_state` | 当前车辆位姿和速度是否稳定 |
| `/localization/pose_with_covariance` | 定位协方差是否合理 |
| `/vehicle/status/velocity_status` | 当前车速是否正常 |
| `/vehicle/status/steering_status` | 当前转角是否正常 |

### 6.4 RViz 图层

- Vehicle model
- TF
- Localization pose
- Vector map
- Pointcloud map

### 6.5 关键参数

这一段不重点调算法参数，重点检查：

- 初始位姿是否落在地图附近。
- 初始朝向是否和车道方向一致。
- 是否有 `/localization/kinematic_state`。
- TF 是否有 `map -> base_link` 链路。

### 6.6 讲解脚本

```text
第一步不是点目标，而是先让系统知道车在哪里。

在 RViz 里点击 2D Pose Estimate 后，我们不是直接把起点给 Mission Planner。
Autoware 会先通过定位或仿真链路更新当前车辆状态。

Mission Planner 真正使用的起点，通常来自 /localization/kinematic_state。
所以如果你没有先设置初始位姿，后面点目标时经常会看到 mission planner is not ready。
```

### 6.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 点目标后没有 route | 是否先设置了初始位姿 |
| mission planner not ready | `/map/vector_map` 和 `/localization/kinematic_state` 是否都有 |
| 车辆在 RViz 中偏离道路很远 | 初始位姿点错、地图坐标系不对、TF 不对 |
| 车辆朝向反了 | 2D Pose Estimate 箭头方向错误 |

---

## 7. 第 3 段脚本：目标点和 Mission Planner

### 7.1 讲解重点

RViz 里点击目标点后，目标点会进入 routing API，然后 Mission Planner 生成 Lanelet route。

目标点不是最终轨迹点。

它只是告诉系统：

```text
我要去哪里。
```

Mission Planner 回答的是：

```text
应该经过哪些 lanelet。
```

### 7.2 数据链路

当前项目整理的目标点链路：

```text
RViz 2D Goal Pose
  -> /planning/mission_planning/goal
  -> routing_adaptor
  -> /api/routing/set_route_points
  -> default_adapi RoutingNode
  -> /planning/set_waypoint_route
  -> RouteSelector
  -> /planning/mission_planning/mission_planner/set_waypoint_route
  -> MissionPlanner::on_set_waypoint_route()
```

最终输出：

```text
/planning/mission_planning/route
/planning/mission_planning/route_state
/planning/mission_planning/route_marker
```

### 7.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/mission_planning/goal` | RViz 目标点是否发出 |
| `/api/routing/set_route_points` | 是否进入 routing API |
| `/planning/mission_planning/route` | route 是否生成 |
| `/planning/mission_planning/route_state` | route 状态是否 ready / set |
| `/planning/mission_planning/route_marker` | RViz 是否显示路线 |
| `/planning/mission_remaining_distance_time` | 剩余距离和时间，若启用 |

### 7.4 RViz 图层

- Vector map
- Route marker
- Echo back goal pose
- Lanelet boundaries
- Vehicle pose

### 7.5 关键参数

Mission Planner 相关参数建议讲直觉，不必现场逐个改：

| 参数类型 | 作用 |
|---|---|
| 起点和目标点 lanelet 搜索范围 | 决定点能否投影到合适车道 |
| 朝向惩罚 | 避免把车辆投影到方向相反的车道 |
| reroute time threshold | 自动驾驶中重规划需要保留的安全时间 |
| minimum reroute length | 重规划时共同路段的最低长度 |
| allow reroute in autonomous mode | 自动驾驶中是否允许重规划 |

### 7.6 讲解脚本

```text
现在我们点击目标点。
注意看 RViz 中 route marker 出现了，但这还不是车辆要逐点跟踪的轨迹。

Route 是车道级路线。
它告诉下游：从当前 lanelet 到目标 lanelet，中间应该经过哪些 lanelet。

你可以把它理解成导航软件告诉你走哪几条路。
但它还没有告诉你在车道内压哪条线，也没有告诉你每个点用多少速度。
```

### 7.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 点击目标点没有反应 | RViz goal topic 是否是 `/planning/mission_planning/goal` |
| route 没生成 | `/localization/kinematic_state`、`/map/vector_map` 是否正常 |
| route 走错车道 | 目标点投影是否落到错误 lanelet |
| route 方向反了 | 初始朝向或目标朝向是否和车道方向冲突 |
| route 断开 | Lanelet 拓扑是否连通 |

---

## 8. 第 4 段脚本：Behavior Path Planner

### 8.1 讲解重点

Behavior Path Planner 从 route 生成可行驶走廊内的 path。

它回答：

```text
在这些车道里，几何上应该走哪条线？
```

它不是最终速度轨迹，也不是控制命令。

### 8.2 数据链路

```text
/planning/mission_planning/route
  -> Behavior Path Planner
  -> /planning/scenario_planning/lane_driving/behavior_planning/path
```

内部还会有模块候选路径：

```text
/planning/path_candidate/start_planner
/planning/path_candidate/lane_change_left
/planning/path_candidate/lane_change_right
/planning/path_candidate/static_obstacle_avoidance
```

参考路径：

```text
/planning/path_reference/start_planner
/planning/path_reference/lane_change_left
/planning/path_reference/lane_change_right
/planning/path_reference/static_obstacle_avoidance
```

### 8.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/scenario` | 当前是 lane driving 还是 parking |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | 行为规划最终 path |
| `/planning/path_candidate/start_planner` | 起步候选路径 |
| `/planning/path_reference/start_planner` | 起步参考路径 |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound` | 可行驶边界 |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/start_planner` | start planner 状态信息 |

### 8.4 RViz 图层

- Behavior path
- Path candidate
- Path reference
- Behavior path planner debug bound
- Objects of interest
- Virtual wall for start planner / lane change / avoidance

### 8.5 关键参数

| 参数类型 | 作用 |
|---|---|
| `launch_start_planner_module` | 是否启用起步规划 |
| `launch_lane_change_left/right_module` | 是否启用换道模块 |
| `launch_static_obstacle_avoidance` | 是否启用静态障碍物避让 |
| `enable_all_modules_auto_mode` | 是否自动批准模块 |
| `enable_rtc` | 是否需要外部确认 |
| drivable area expansion 参数 | 可行驶区域边界扩张或收缩 |

### 8.6 讲解脚本

```text
有了 route 以后，下游还不能直接控制车辆。
因为 route 只是车道序列，没有给出车道内具体走哪条线。

Behavior Path Planner 会根据当前 route、车辆位置、障碍物、交通场景模块，生成一条 path。

如果车辆不在正常车道中心附近，Start Planner 会尝试生成 pull out 路径；
如果需要换道，会出现 lane change candidate；
如果前方有静态障碍，会出现 avoidance 相关路径。
```

### 8.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 有 route 但没有 path | 当前 scenario 是否是 lane driving |
| path 不在车道内 | drivable area / route section / lanelet 边界是否正确 |
| 起步失败 | 车辆初始位置是否过于偏离 lanelet，Start Planner 是否启用 |
| 候选路径有但没有采用 | RTC 是否需要批准，模块状态是否 waiting |
| path 抖动 | 输入 route、定位、障碍物、模块切换是否抖动 |

---

## 9. 第 5 段脚本：Pull Out 可选片段

### 9.1 讲解重点

如果车辆起点不在正常车道行驶线附近，Start Planner / Pull Out 模块会生成回归车道的路径。

它回答：

```text
车辆如何从自由空间回到车道？
```

### 9.2 可观察 topic

| topic | 看什么 |
|---|---|
| `/planning/path_candidate/start_planner` | Start Planner 候选路径 |
| `/planning/path_reference/start_planner` | Start Planner 参考路径 |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/start_planner` | 起步模块 debug |
| `/planning/debug/objects_of_interest/start_planner` | 起步相关障碍物 |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/virtual_wall/start_planner` | 起步相关 virtual wall |

### 9.3 关键参数

| 参数类型 | 作用 |
|---|---|
| 最大后退距离 | 控制是否允许倒车找空间 |
| lateral / longitudinal margin | 控制起步路径与边界或障碍的安全距离 |
| pull out search method | 决定使用 shift、geometric、Hybrid A* 等方法 |
| collision check margin | 决定候选路径碰撞检查保守程度 |

### 9.4 讲解脚本

```text
这一步不是每个普通车道行驶案例都会触发。
但如果车辆从路边、停车位、自由空间起步，普通车道中心线不一定能直接接上当前车辆姿态。

这时 Start Planner 会尝试生成 pull out path。
Hybrid A* 更像在自由空间中搜索一条满足车辆运动学的路径；
Shift Pull Out 更像在已知车道参考线附近做横向偏移。
```

### 9.5 常见故障

| 现象 | 优先排查 |
|---|---|
| 无法 pull out | 起点是否太靠近障碍或边界 |
| 候选路径碰撞 | collision margin、车辆尺寸、障碍物输入 |
| 起步路径很绕 | Hybrid A* 代价、最小转弯半径、目标姿态 |
| 倒车不符合预期 | 最大倒车距离、是否允许 reverse |

---

## 10. 第 6 段脚本：Elastic Band 和路径平滑

### 10.1 讲解重点

Behavior Path 输出的路径点可能有轻微锯齿、方向跳变或曲率抖动。

Elastic Band 负责轻量几何平滑。

它回答：

```text
这串离散路径点能不能更光滑？
```

### 10.2 数据链路

在当前 launch 中，motion planning 内部链路大致为：

```text
/planning/scenario_planning/lane_driving/behavior_planning/path
  -> elastic_band_smoother
  -> /planning/scenario_planning/lane_driving/motion_planning/path_smoother/path
```

如果不启用 path smoother，则可能直接 relay 到后续模块。

### 10.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | 平滑前路径 |
| `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path` | 平滑后路径 |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound` | 平滑允许范围 |

### 10.4 RViz 图层

- 平滑前 path
- 平滑后 path
- drivable area bound
- 曲率或点列方向可视化，若有

### 10.5 关键参数

| 参数类型 | 作用 |
|---|---|
| 重采样间隔 | 决定路径点密度 |
| 二阶差分平滑权重 | 控制路径光顺程度 |
| 横向误差权重 | 控制不要偏离原始路径太多 |
| fix / joint / smooth 区域 | 控制哪些点固定、哪些点可移动 |
| clearance | 控制相对边界的安全余量 |

### 10.6 讲解脚本

```text
Elastic Band 不负责决定走哪条车道，也不直接做车辆控制。
它更像把一串离散几何点揉顺，让后面的 Frenet 坐标、曲率估计、MPT 约束更稳定。

如果参考路径有锯齿，下游控制器会看到抖动的航向和曲率。
所以路径平滑不是锦上添花，而是让后续优化和控制有一个稳定输入。
```

### 10.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 平滑后路径偏离太多 | 横向误差权重是否太小，可移动范围是否太大 |
| 平滑效果不明显 | 平滑权重是否太小，fix 区域是否过长 |
| 路径仍然抖 | 输入 path 是否已经严重抖动，重采样是否不稳定 |
| 路径越界 | Elastic Band 本身不做完整 footprint 碰撞，需看后续 MPT / validator |

---

## 11. 第 7 段脚本：MPT / Path Optimizer

### 11.1 讲解重点

MPT 负责把平滑后的参考路径进一步优化成更可跟踪、更满足车辆模型和边界约束的局部轨迹。

它回答：

```text
这条参考路径能不能变成车辆更容易跟踪、且不越界的轨迹？
```

### 11.2 数据链路

当前 launch 中：

```text
/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path
  -> path_optimizer
  -> /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory
```

如果选择 `path_sampler` 或 `none`，输出路径可能仍映射到同一个后续 topic。

### 11.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path` | MPT 输入参考路径 |
| `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | MPT 输出轨迹 |
| `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/processing_time_ms` | 求解耗时，若启用 |
| `/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/marker` | 相关优化 debug marker，具体取决于配置 |

### 11.4 RViz 图层

- MPT 输入参考线
- MPT 输出轨迹
- drivable area boundary
- 车辆 footprint
- debug marker，若开启

### 11.5 关键参数

来自当前 path optimizer 参数的重点：

| 参数 | 作用 |
|---|---|
| `option.enable_skip_optimization` | 跳过 Elastic Band 和 MPT，用于对比 |
| `option.enable_reset_prev_optimization` | 是否重置上一帧优化结果 |
| `option.enable_outside_drivable_area_stop` | 越出可行驶区域时是否停车 |
| `mpt.common.num_points` | 优化点数 |
| `mpt.common.delta_arc_length` | 优化点间距 |
| `common.output_delta_arc_length` | 输出轨迹重采样间隔 |
| `replan.max_path_shape_forward_lat_dist` | 前方路径变化触发重规划阈值 |
| `mpt.weight.lat_error_weight` | 横向误差权重 |
| `mpt.weight.yaw_error_weight` | 航向误差权重 |
| `mpt.weight.steer_input_weight` | 转向输入权重 |
| `mpt.weight.steer_rate_weight` | 转向变化权重 |
| `mpt.clearance.soft_clearance_from_road` | 软约束道路 clearance |
| `mpt.collision_free_constraints.option.soft_constraint` | 是否使用软约束 |
| `mpt.collision_free_constraints.vehicle_circles.*` | 车辆圆近似方式 |

### 11.6 讲解脚本

```text
Elastic Band 主要让几何线更平滑。
MPT 则进一步把车辆模型、道路边界、车辆轮廓和连续性放进同一个优化问题。

它不是简单把点平滑一下，而是会考虑：
车辆能不能按这个转角变化走出来，
车身圆是否在边界内，
上一帧轨迹能否平滑继承，
以及必要时能否用 soft constraint 避免 QP 无解。
```

### 11.7 常见故障

| 现象 | 优先排查 |
|---|---|
| MPT 输出为空 | 输入 path 是否存在，优化是否被 skip |
| MPT 求解失败 | 边界是否过窄，车辆尺寸是否正确，soft constraint 是否启用 |
| 轨迹贴边 | clearance、车辆圆半径、边界输入是否合理 |
| 轨迹每帧跳动 | warm start、fixed point、reset prev optimization、replan 阈值 |
| 轨迹过于保守 | lat/yaw/steer/steer_rate 权重是否过大 |
| 求解耗时高 | num_points、circle 数量、soft constraint、debug marker 是否过重 |

---

## 12. 第 8 段脚本：Motion Velocity Planner

### 12.1 讲解重点

MPT 输出的是几何和运动学更合理的轨迹，但还需要处理障碍物、停止线、动态物体等速度决策。

Motion Velocity Planner 回答：

```text
这条轨迹上哪里需要停车、减速、限速？
```

### 12.2 数据链路

```text
/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory
  -> motion_velocity_planner
  -> /planning/scenario_planning/lane_driving/trajectory
```

### 12.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | 速度规划输入 |
| `/planning/scenario_planning/lane_driving/trajectory` | lane driving 输出轨迹 |
| `/planning/scenario_planning/status/stop_reasons` | 停止原因 |
| `/planning/velocity_factors/motion_velocity_planner` | 速度因素 |
| `/planning/scenario_planning/max_velocity_candidates` | 候选速度限制 |
| `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop/virtual_walls` | 障碍物停止墙 |
| `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down/virtual_walls` | 障碍物减速墙 |
| `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/out_of_lane/virtual_walls` | 越界相关墙 |

### 12.4 RViz 图层

- Lane driving trajectory
- Virtual wall
- Stop reasons
- Velocity factors
- Perception objects
- Occupancy grid，若使用

### 12.5 关键参数

| 参数类型 | 作用 |
|---|---|
| obstacle stop 模块参数 | 控制障碍物停车距离和判定范围 |
| obstacle slow down 模块参数 | 控制减速区域和目标速度 |
| out of lane 模块参数 | 控制越界风险处理 |
| run out / road user stop 参数 | 控制行人或路边风险减速 |
| velocity smoother 类型参数 | 一些速度规划模块内部也会复用速度平滑参数 |

### 12.6 讲解脚本

```text
现在我们有了一条几何上更好的轨迹。
但真实车辆不能只知道走哪条线，还要知道哪里该慢、哪里该停。

Motion Velocity Planner 会根据障碍物、地图停止线、交通场景和越界风险，在轨迹上插入速度限制或停止点。

RViz 里的 virtual wall 很有用。
如果车突然停住，先看是不是某个速度规划模块生成了停止墙。
```

### 12.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 车莫名停车 | stop_reasons、virtual wall、velocity_factors |
| 速度突然很低 | max_velocity_candidates、slow_down 模块 |
| 障碍物不触发停车 | perception objects、occupancy grid、模块是否启用 |
| 停止点位置不对 | 地图停止线、车辆 footprint、停止 margin |

---

## 13. 第 9 段脚本：Scenario Selector 和 Velocity Smoother

### 13.1 讲解重点

Scenario Selector 在 lane driving 和 parking 等场景之间选择最终轨迹。

Velocity Smoother 则把速度曲线变得满足加速度和 jerk 约束。

它回答：

```text
最终给控制器的轨迹速度是否连续、舒适、可执行？
```

### 13.2 数据链路

```text
/planning/scenario_planning/lane_driving/trajectory
  -> scenario_selector
  -> /planning/scenario_planning/scenario_selector/trajectory
  -> velocity_smoother
  -> /planning/scenario_planning/velocity_smoother/trajectory
```

然后：

```text
/planning/scenario_planning/velocity_smoother/trajectory
  -> planning_validator
  -> /planning/trajectory
```

### 13.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/scenario` | 当前场景选择 |
| `/planning/scenario_planning/scenario_selector/trajectory` | 场景选择后的轨迹 |
| `/planning/scenario_planning/velocity_smoother/trajectory` | 速度平滑后的轨迹 |
| `/planning/scenario_planning/current_max_velocity` | 当前速度上限 |
| `/planning/scenario_planning/velocity_smoother/closest_velocity` | 最近点速度 |
| `/planning/scenario_planning/velocity_smoother/closest_acceleration` | 最近点加速度 |
| `/planning/scenario_planning/velocity_smoother/debug/processing_time_ms` | 平滑器耗时 |

### 13.4 RViz 图层

- Scenario selector trajectory
- Velocity smoother trajectory
- Current max velocity
- Stop point
- Velocity colorized trajectory

### 13.5 关键参数

| 参数类型 | 作用 |
|---|---|
| velocity smoother algorithm type | Analytical、JerkFiltered 等 |
| resampling interval | 速度曲线离散间隔 |
| max acceleration | 最大加速度 |
| min deceleration | 最大减速度，通常是负值 |
| max jerk / min jerk | jerk 舒适性限制 |
| lateral acceleration limit | 弯道速度限制 |
| stop distance / mild stop 参数 | 停车舒适性 |

### 13.6 讲解脚本

```text
Velocity Smoother 的关键任务不是简单把速度上限裁掉。
如果速度从 10m/s 突然变成 0，车辆不可能瞬间完成。

所以它要考虑加速度和 jerk：
速度怎么降、什么时候开始降、停车点前能不能舒服地停住。

这一层处理不好，观众会看到的现象通常是急刹、速度抖、停车点附近反复调整。
```

### 13.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 急刹 | stop point、min deceleration、jerk、前方速度上限突变 |
| 速度曲线抖 | 输入轨迹点、速度限制源、resampling |
| 弯道速度太高 | lateral acceleration limit、曲率估计 |
| 停不住 | 停止点距离、当前速度、最大减速度、控制器跟踪 |
| 最终 `/planning/trajectory` 没有 | planning validator 是否拒绝 |

---

## 14. 第 10 段脚本：Planning Validator

### 14.1 讲解重点

规划输出给控制器前，会经过检查。

Planning Validator 的作用是：

```text
确认轨迹在物理、延迟和安全检查上没有明显问题。
```

### 14.2 数据链路

```text
/planning/scenario_planning/velocity_smoother/trajectory
  -> planning_validator
  -> /planning/trajectory
```

### 14.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/scenario_planning/velocity_smoother/trajectory` | validator 输入 |
| `/planning/trajectory` | validator 输出，也是控制器输入 |
| `/planning/planning_validator/debug/marker` | validator debug |
| `/planning/planning_validator/output/markers` | validator 输出 marker |
| `/planning/planning_validator/virtual_wall` | validator 生成的安全墙 |

### 14.4 RViz 图层

- Final trajectory
- Planning validator markers
- Planning validator virtual wall

### 14.5 关键参数

| 参数类型 | 作用 |
|---|---|
| latency checker | 轨迹延迟检查 |
| trajectory checker | 曲率、速度、加速度等轨迹质量检查 |
| collision checker | 轨迹碰撞或交叉风险检查 |

### 14.6 讲解脚本

```text
很多人以为 Velocity Smoother 输出后就直接进控制器。
实际上，最终进入控制器的是 /planning/trajectory。

在当前链路里，Planning Validator 会把 velocity smoother 的输出作为输入，
检查通过后再发布最终轨迹。

所以如果你看到速度平滑后有轨迹，但控制器没有收到 /planning/trajectory，
就要看 validator 是否拒绝了它。
```

### 14.7 常见故障

| 现象 | 优先排查 |
|---|---|
| velocity smoother 有输出但 `/planning/trajectory` 没有 | validator 是否报错 |
| RViz 出现 validator virtual wall | 检查具体 marker 文本 |
| 控制器无输出 | 先确认 `/planning/trajectory` 是否正常发布 |

---

## 15. 第 11 段脚本：Trajectory Follower

### 15.1 讲解重点

控制器把最终轨迹变成当前控制命令。

横向控制回答：

```text
当前方向盘或前轮转角该是多少？
```

纵向控制回答：

```text
当前加速度、油门或刹车该是多少？
```

### 15.2 数据链路

```text
/planning/trajectory
  -> trajectory_follower
  -> /control/trajectory_follower/control_cmd
  -> vehicle_cmd_gate
  -> /control/command/control_cmd
  -> vehicle_interface
```

### 15.3 重点 topic

| topic | 看什么 |
|---|---|
| `/planning/trajectory` | 控制器参考轨迹 |
| `/localization/kinematic_state` | 当前车辆状态 |
| `/vehicle/status/steering_status` | 当前实际转角 |
| `/localization/acceleration` | 当前加速度 |
| `/control/trajectory_follower/control_cmd` | 控制器原始输出 |
| `/control/command/control_cmd` | 经过 gate 后发给车辆的命令 |
| `/control/trajectory_follower/lateral/predicted_trajectory` | 横向控制预测轨迹 |
| `/control/trajectory_follower/lateral/diagnostic` | 横向控制诊断 |
| `/control/trajectory_follower/longitudinal/diagnostic` | 纵向控制诊断 |
| `/control/trajectory_follower/longitudinal/stop_reason` | 纵向停车原因 |

RViz 里还能看到：

```text
/control/trajectory_follower/controller_node_exe/debug/resampled_reference_trajectory
/control/trajectory_follower/controller_node_exe/debug/predicted_trajectory_in_frenet_coordinate
/control/trajectory_follower/mpc_follower/debug/markers
```

### 15.4 RViz 图层

- Final trajectory
- Controller resampled reference trajectory
- Predicted trajectory
- MPC debug markers
- Stop reason
- Vehicle status overlay

### 15.5 关键参数

横向 MPC 重点参数：

| 参数 | 作用 |
|---|---|
| `lateral_controller_mode` | 横向控制模式，默认可能是 `mpc` |
| `mpc_prediction_horizon` | 预测步数 |
| `mpc_prediction_dt` | 预测时间间隔 |
| `mpc_weight_lat_error` | 横向误差权重 |
| `mpc_weight_heading_error*` | 航向误差权重 |
| `mpc_weight_steering_input` | 转向输入权重 |
| `mpc_weight_steer_rate` | 转向速率权重 |
| `mpc_weight_steer_acc` | 转向加速度权重 |
| `vehicle_model_type` | 车辆模型类型 |
| `mpc_converged_threshold_rps` | 收敛判断阈值 |

Pure Pursuit 重点参数：

| 参数类型 | 作用 |
|---|---|
| lookahead distance | 前瞻距离 |
| speed proportional gain | 速度越快前瞻越远 |
| curvature / lateral error 相关项 | 弯道和误差下调整前瞻 |
| steering limit | 转角限幅 |

纵向控制重点参数：

| 参数类型 | 作用 |
|---|---|
| PID 增益 | 速度跟踪响应 |
| delay compensation | 延迟补偿 |
| stop state 参数 | 低速停车行为 |
| acceleration limit | 控制输出限幅 |

### 15.6 讲解脚本

```text
到这里，规划层已经给出了 /planning/trajectory。
控制器会拿当前车辆状态和这条轨迹做比较，输出当前这一帧应该发出的控制命令。

如果使用 Pure Pursuit，它会在前方找一个 lookahead point，用几何圆弧计算转角。
如果使用 MPC，它会用车辆模型预测未来误差，并解一个优化问题得到未来控制序列，然后只执行第一个控制。

注意：控制器不是重新规划路径。
它只是在当前状态下尽量跟踪规划层给出的轨迹。
```

### 15.7 常见故障

| 现象 | 优先排查 |
|---|---|
| 控制器没有输出 | `/planning/trajectory`、odometry、steering status 是否有 |
| 有 follower cmd 但车辆不动 | gate、engage、operation mode、vehicle interface |
| 转向抖动 | final trajectory 曲率是否抖，MPC/Pure Pursuit 参数 |
| 弯道外切 | 速度过高、转角限制、预测时域、轨迹曲率 |
| 停车不稳 | 速度曲线、纵向 PID、stop reason、当前速度 |

---

## 16. 第 12 段脚本：Vehicle Cmd Gate 和底盘状态

### 16.1 讲解重点

控制器输出不一定直接进入车辆。

通常还要经过：

```text
vehicle_cmd_gate
operation mode
engage
emergency
vehicle interface
```

所以：

```text
有 control_cmd 不代表车辆一定会动。
```

### 16.2 重点 topic

| topic | 看什么 |
|---|---|
| `/control/trajectory_follower/control_cmd` | 控制器输出 |
| `/control/command/control_cmd` | gate 后的最终控制命令 |
| `/control/current_gate_mode` | 当前 gate 模式 |
| `/system/operation_mode/state` | 系统操作模式 |
| `/api/autoware/get/engage` | engage 状态，具体版本可能不同 |
| `/vehicle/status/control_mode` | 车辆控制模式 |
| `/vehicle/status/velocity_status` | 实际速度 |
| `/vehicle/status/steering_status` | 实际转角 |
| `/vehicle/status/gear_status` | 档位 |

### 16.3 RViz 图层

- Vehicle status panel
- Autoware state panel
- Operation mode
- Control command overlay，若配置中有

### 16.4 常见故障

| 现象 | 优先排查 |
|---|---|
| follower 有输出但 `/control/command/control_cmd` 没变化 | vehicle_cmd_gate、operation mode、emergency |
| `/control/command/control_cmd` 有但车辆不动 | vehicle interface、档位、控制模式 |
| 自动驾驶不可用 | `/control/is_autonomous_available`、operation mode transition |
| 紧急停车 | emergency cmd、AEB、collision checker、planning validator |

### 16.5 讲解脚本

```text
最后一步很容易被忽略。
控制器算出命令以后，系统还要判断现在是否允许自动驾驶命令通过。

如果还没有 engage，或者 operation mode 不是 autonomous，
或者系统进入 emergency，命令可能会被 gate 拦住。

所以实战排查时，不能只看 /control/trajectory_follower/control_cmd，
还要看 /control/command/control_cmd 和车辆状态有没有真实变化。
```

---

## 17. 端到端观察清单

### 17.1 最小闭环 topic 清单

如果只想确认主链路通不通，按顺序看这些：

| 顺序 | topic | 期望现象 |
|---|---|---|
| 1 | `/localization/kinematic_state` | 当前位姿持续发布 |
| 2 | `/planning/mission_planning/route` | 点击目标后出现 route |
| 3 | `/planning/scenario_planning/lane_driving/behavior_planning/path` | route 后出现 path |
| 4 | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | 出现优化轨迹 |
| 5 | `/planning/scenario_planning/lane_driving/trajectory` | 出现 lane driving trajectory |
| 6 | `/planning/scenario_planning/velocity_smoother/trajectory` | 出现速度平滑轨迹 |
| 7 | `/planning/trajectory` | 最终轨迹发布 |
| 8 | `/control/trajectory_follower/control_cmd` | 控制器输出 |
| 9 | `/control/command/control_cmd` | gate 后命令输出 |
| 10 | `/vehicle/status/velocity_status` | 车辆速度变化 |

### 17.2 一句话排查法

```text
从下游发现问题，就往上游找第一个不正常的 topic。
```

例如车辆不动：

```text
车辆状态不动
  -> /control/command/control_cmd 是否有
  -> /control/trajectory_follower/control_cmd 是否有
  -> /planning/trajectory 是否有
  -> /planning/scenario_planning/velocity_smoother/trajectory 是否有
  -> route/path 是否有
```

第一个断掉的位置，就是优先排查的层。

---

## 18. 常见故障总表

### 18.1 Mission Planner 相关

| 现象 | 可能原因 | 排查顺序 |
|---|---|---|
| mission planner is not ready | 没有地图或定位 | `/map/vector_map` -> `/localization/kinematic_state` -> 初始位姿 |
| 点击目标没有 route | 目标 topic 不对或 API 未接通 | `/planning/mission_planning/goal` -> routing API -> route_state |
| route 走错方向 | 起点/目标点投影到反向 lanelet | RViz 看 lanelet 方向、目标朝向、起点朝向 |
| route 断开 | 地图拓扑不连通 | Lanelet 连接关系、目标点是否落在可达车道 |

### 18.2 Behavior Path 相关

| 现象 | 可能原因 | 排查顺序 |
|---|---|---|
| 有 route 无 path | scenario 不对或 behavior planner 未输出 | scenario -> route -> behavior path topic |
| path 突然切换 | 场景模块状态变化 | path_candidate、info、debug marker |
| lane change 不执行 | RTC 未批准或安全检查失败 | module info、objects_of_interest、RTC 状态 |
| start planner 不输出 | 起点位置、障碍、边界不满足 | start_planner candidate、bound、objects |

### 18.3 Path Smoothing / MPT 相关

| 现象 | 可能原因 | 排查顺序 |
|---|---|---|
| 平滑后仍锯齿 | 输入 path 抖或权重太小 | behavior path -> path_smoother output -> 参数 |
| MPT 无输出 | 输入为空或优化失败 | path_smoother/path -> path_optimizer/trajectory -> 日志 |
| MPT 贴边 | clearance、车辆尺寸、边界错误 | bound -> vehicle circles -> clearance 参数 |
| 轨迹每帧跳 | warm start / fixed point / replan 阈值 | reset prev optimization、replan 参数、定位噪声 |

### 18.4 Velocity 相关

| 现象 | 可能原因 | 排查顺序 |
|---|---|---|
| 突然停车 | virtual wall 或 stop reason | stop_reasons -> virtual_wall -> perception/map |
| 急刹 | 停止点太近或 jerk/accel 参数 | velocity_smoother trajectory -> closest_acceleration |
| 弯道过快 | 曲率或横向加速度限制 | final trajectory curvature -> lateral acceleration limit |
| 速度忽快忽慢 | 上游速度限制跳变 | max_velocity_candidates -> current_max_velocity |

### 18.5 Control 相关

| 现象 | 可能原因 | 排查顺序 |
|---|---|---|
| 控制器无输出 | 缺少轨迹或车辆状态 | `/planning/trajectory` -> odometry -> steering |
| 转向抖动 | 轨迹曲率抖或控制参数激进 | final trajectory -> resampled reference -> MPC/Pure Pursuit 参数 |
| 弯道外切 | 速度高、转角受限、预测时域短 | velocity -> steering limit -> predicted trajectory |
| 有 follower cmd 但车辆不动 | gate 或车辆接口问题 | follower cmd -> command cmd -> operation mode -> vehicle status |

---

## 19. 录屏演示步骤

### Step 1：展示系统空状态

画面：

- 地图
- 车辆
- 没有 route

讲：

```text
现在系统还不知道我们要去哪。
```

观察：

```bash
ros2 topic hz /localization/kinematic_state
```

### Step 2：设置初始位姿

画面：

- RViz 2D Pose Estimate
- 车辆位置更新

讲：

```text
这一步让系统知道车辆当前在哪里。
```

观察：

```bash
ros2 topic echo /localization/kinematic_state
```

### Step 3：点击目标点

画面：

- RViz 2D Goal Pose
- route marker 出现

讲：

```text
Mission Planner 现在生成的是车道级 route，不是轨迹。
```

观察：

```bash
ros2 topic echo /planning/mission_planning/route_state
```

### Step 4：打开 Behavior Path

画面：

- behavior path
- path candidate/reference

讲：

```text
行为规划把车道级路线变成车道内的几何 path。
```

### Step 5：打开 MPT 输出

画面：

- path optimizer trajectory
- path smoother path

讲：

```text
MPT 让局部轨迹更可跟踪、更满足边界和车辆模型。
```

### Step 6：打开速度轨迹

画面：

- lane driving trajectory
- velocity smoother trajectory
- virtual wall

讲：

```text
速度规划和速度平滑把几何轨迹变成带速度含义的 trajectory。
```

### Step 7：打开最终轨迹和控制输出

画面：

- `/planning/trajectory`
- predicted trajectory
- control command
- vehicle status

讲：

```text
控制器只跟踪最终 /planning/trajectory。
它根据当前状态输出这一帧的转向和加速度。
```

### Step 8：让车辆运动

画面：

- 车辆沿轨迹移动
- steering status 变化
- velocity status 变化

讲：

```text
车辆运动后，定位状态更新，下一帧规划控制再次计算。
这就是闭环。
```

---

## 20. PPT 大纲

### 第 1 页：标题页

标题：

```text
Autoware 规划控制层端到端实战复盘
```

副标题：

```text
从目标点到控制命令
```

配图：

一张完整链路图。

### 第 2 页：本集要解决的问题

内容：

- 点击目标点后发生了什么？
- Route、Path、Trajectory、Control Command 如何连接？
- 出问题时应该先看哪个 topic？

### 第 3 页：案例设定

内容：

```text
车辆在 lanelet 地图中的某条车道附近；
目标点在前方；
主线使用普通 lane driving；
可选加入 pull out 或 stop point。
```

### 第 4 页：总链路图

内容：

```text
Initial Pose -> Goal -> Route -> Path -> MPT -> Velocity -> Validator -> Control -> Vehicle
```

### 第 5 页：RViz 观察图层

内容表格：

- Map
- Route marker
- Behavior path
- Path optimizer trajectory
- Velocity smoother trajectory
- Final trajectory
- Control predicted trajectory
- Vehicle status

### 第 6 页：初始位姿链路

内容：

```text
2D Pose Estimate -> /initialpose -> localization -> /localization/kinematic_state
```

备注：

强调 Mission Planner 用 odometry 作为 route 起点。

### 第 7 页：目标点链路

内容：

```text
2D Goal Pose -> /planning/mission_planning/goal -> routing API -> mission planner
```

### 第 8 页：Mission Planner 输出

内容：

- `/planning/mission_planning/route`
- `/planning/mission_planning/route_state`
- `/planning/mission_planning/route_marker`

结论：

```text
Route 是 lanelet 序列，不是轨迹。
```

### 第 9 页：Route 常见问题

内容：

- mission planner not ready
- route 方向错误
- 目标点投影错误
- lanelet 拓扑不连通

### 第 10 页：Behavior Path Planner

内容：

```text
Route -> Behavior Path -> Path
```

展示 topic：

```text
/planning/scenario_planning/lane_driving/behavior_planning/path
```

### 第 11 页：场景模块

内容：

- start_planner
- lane_change
- static_obstacle_avoidance
- goal_planner
- dynamic_obstacle_avoidance

### 第 12 页：Pull Out 可选链路

内容：

- `/planning/path_candidate/start_planner`
- `/planning/path_reference/start_planner`

结论：

```text
车辆不在正常车道时，需要起步规划接回车道。
```

### 第 13 页：Elastic Band

内容：

```text
Behavior path -> path smoother -> smoother path
```

结论：

```text
它主要改善几何光顺性。
```

### 第 14 页：MPT 定位

内容：

```text
MPT 是规划和控制之间的局部轨迹优化器。
```

### 第 15 页：MPT 输入输出

输入：

- reference path
- boundary
- odometry
- vehicle params

输出：

- path optimizer trajectory

### 第 16 页：MPT 关键参数

内容：

- num_points
- delta_arc_length
- lat/yaw/steer weights
- clearance
- soft constraint
- vehicle circles
- replan thresholds

### 第 17 页：MPT 常见问题

内容：

- 无输出
- 贴边
- 跳动
- 求解慢
- 越界

### 第 18 页：Motion Velocity Planner

内容：

```text
Path optimizer trajectory -> lane driving trajectory
```

观察：

- stop_reasons
- virtual walls
- velocity_factors

### 第 19 页：Velocity Smoother

内容：

```text
Scenario trajectory -> velocity smoother trajectory
```

公式直觉：

```text
速度不能硬裁剪；
要满足 acceleration 和 jerk。
```

### 第 20 页：Planning Validator

内容：

```text
velocity smoother trajectory -> planning validator -> /planning/trajectory
```

结论：

```text
控制器只看最终 /planning/trajectory。
```

### 第 21 页：Trajectory Follower

内容：

```text
/planning/trajectory -> /control/trajectory_follower/control_cmd
```

区分：

- Pure Pursuit
- MPC
- longitudinal PID

### 第 22 页：Control Gate

内容：

```text
follower cmd -> vehicle cmd gate -> /control/command/control_cmd
```

强调：

```text
有 follower cmd 不等于车辆一定会动。
```

### 第 23 页：最小 topic 检查链

内容表格：

```text
/localization/kinematic_state
/planning/mission_planning/route
/planning/.../behavior_planning/path
/planning/.../path_optimizer/trajectory
/planning/.../velocity_smoother/trajectory
/planning/trajectory
/control/trajectory_follower/control_cmd
/control/command/control_cmd
/vehicle/status/velocity_status
```

### 第 24 页：故障排查原则

内容：

```text
从现象所在层往上游找第一个不正常 topic。
```

### 第 25 页：路线问题排查

内容：

- map
- odometry
- goal
- route_state
- route_marker

### 第 26 页：路径问题排查

内容：

- route
- scenario
- behavior path
- path candidate
- debug bound

### 第 27 页：轨迹优化问题排查

内容：

- path smoother
- path optimizer
- MPT params
- boundary
- vehicle dimensions

### 第 28 页：速度问题排查

内容：

- virtual wall
- stop reason
- max velocity
- acceleration
- jerk

### 第 29 页：控制问题排查

内容：

- final trajectory
- odometry
- steering status
- control cmd
- predicted trajectory
- gate mode

### 第 30 页：全链路复盘大图

内容：

```text
Map + Goal
  -> Route
  -> Path
  -> Optimized Trajectory
  -> Velocity Trajectory
  -> Validated Trajectory
  -> Control Command
  -> Vehicle
```

### 第 31 页：学习源码的方法

内容：

每个模块问：

1. 输入是什么？
2. 输出是什么？
3. 上游是谁？
4. 下游是谁？
5. 主干模型是什么？
6. 出错时怎么观测？

### 第 32 页：结尾

内容：

```text
Autoware 的规划控制不是一个单独算法，
而是一条不断闭环更新的数据链。
```

---

## 21. 录课口播总结

可以用这段作为结尾：

```text
今天这集我们没有再推新的公式，而是把前面学过的模块放进同一个真实链路里。

从 RViz 里点击目标点开始，Mission Planner 先生成车道级 route；
Behavior Path Planner 把 route 变成车道内的 path；
Elastic Band 和 MPT 让 path 更平滑、更符合车辆运动学和边界约束；
Velocity Planner 和 Velocity Smoother 给轨迹加上速度、停止点和舒适性约束；
Planning Validator 检查后发布最终 /planning/trajectory；
控制器再根据当前车辆状态，把这条轨迹变成转向和加速度命令。

以后调试 Autoware 时，不要一上来就改控制器参数。
先沿着 topic 链路往上游找：
route 对不对，path 稳不稳，trajectory 是否可执行，速度是否合理，最后再看控制器是否跟得上。

只要你能按这条链路定位问题，你就已经从“看不懂 Autoware”进入到“能系统调试 Autoware”的阶段了。
```

---

## 22. 后续可以继续制作的实战内容

如果这集录完后还想继续扩展，可以做下面几集：

1. **RViz 调试专题**：每个 planning/control 图层到底怎么看。
2. **rosbag 复盘专题**：用 bag 离线找 route/path/trajectory/control 的断点。
3. **转向抖动案例**：从轨迹曲率到 MPC 参数的完整排查。
4. **急刹案例**：从 virtual wall 到 jerk 参数的完整排查。
5. **Pull Out 失败案例**：从起点位置、边界、障碍到 Hybrid A* 参数。
6. **MPT 求解失败案例**：边界太窄、车辆圆、soft constraint 和 warm start。
7. **源码导读专题**：按 topic 反查 launch、node、callback 和核心函数。

