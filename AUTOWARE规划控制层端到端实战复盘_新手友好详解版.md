# Autoware 规划控制层端到端实战复盘：新手友好详解版

> 面向对象：刚学 Autoware 规划控制的新手  
> 推荐时长：60 到 90 分钟  
> 主线目标：用一个从“设置目标点”到“底盘控制命令”的完整案例，把 Route、Path、Trajectory、Velocity、Control 串起来  
> 本文用途：录课脚本、RViz 演示清单、topic 排查表、PPT 大纲
> 版本说明：本文件在原“视频脚本与 PPT 大纲”基础上增强，不删除原有主线，重点补充每一层的输入、输出、正常现象、异常现象、优先排查顺序、录制画面建议和讲解备注。

这份详解版的目标不是把课程讲短，而是把学习门槛降下来。录制时可以放心讲细一点：先让观众看到画面和 topic，再解释模块职责，最后再回到“如果这里坏了该怎么排查”。

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

### 0.1 新手应该怎样使用这份文档

建议把本文件当成“三合一材料”：

1. **录制脚本**：每一段都有建议口播，可以按顺序录。
2. **调试手册**：每一层都列出 topic、RViz 图层和优先排查顺序。
3. **课程总复盘**：把第 1 集到第 8 集的知识重新放回同一条运行链路里。

第一次录制时，不建议同时追求“源码很深”和“演示很快”。更好的节奏是：

```text
先看现象
再看数据
再讲模块
再讲参数
最后讲排查
```

这样新手不会一开始就被 topic、坐标系、算法名和公式压住。

### 0.2 本集和前 8 集的关系

| 前置课程 | 在本集里的作用 | 新手要记住的一句话 |
|---|---|---|
| 第 1 集：规划控制总览 | 提供 route/path/trajectory/control 的总框架 | 先知道数据链路，再看单个算法 |
| 第 2 集：Lanelet 与 Mission Planner | 解释目标点如何变成车道级路线 | Route 是“经过哪些车道”，不是可跟踪轨迹 |
| 第 3 集：Hybrid A* Pull Out | 解释车辆不在车道中心时如何回到车道 | Pull Out 是从自由空间回到结构化车道 |
| 第 4 集：Elastic Band | 解释离散 path 如何变得更平滑 | 平滑是让点列更像车能走的线 |
| 第 5 集：Velocity Smoother | 解释速度不能硬裁剪 | 速度曲线要同时考虑加速度和 jerk |
| 第 6 集：Pure Pursuit | 解释几何控制直觉 | 控制器要把轨迹变成转向 |
| 第 7 集：横向 MPC | 解释车辆模型和 QP 控制 | MPC 是用模型预测未来误差并优化控制 |
| 第 8 集：MPT 与链路复盘 | 解释轨迹优化怎样连接规划和控制 | 优化层把“能走”变成“更好跟踪” |

录这集时不要把前 8 集重新完整讲一遍，而是在每个阶段只补一句“它来自哪一集、解决什么问题”。这样既能复盘，又不会把主线讲散。

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

## 4A. 端到端案例的分层讲解总表

本集建议用同一个案例贯穿到底：

```text
车辆在地图中的一条普通车道附近
  -> RViz 设置初始位姿
  -> RViz 点击前方目标点
  -> Mission Planner 生成 route
  -> Behavior Path Planner 生成 path
  -> 必要时 Start Planner / Pull Out 生成回车道路径
  -> Elastic Band / MPT 优化几何轨迹
  -> Motion Velocity Planner 添加限速、停止、慢行逻辑
  -> Velocity Smoother 让速度曲线满足加速度和 jerk
  -> Planning Validator 检查最终轨迹
  -> Trajectory Follower 输出控制命令
  -> Vehicle Cmd Gate 决定命令是否真正下发
```

新手最容易混淆的是 route、path、trajectory 和 control command。录制时建议反复强调：

| 名称 | 它回答的问题 | 它不是 |
|---|---|---|
| Route | 车辆应该经过哪些车道？ | 不是逐点跟踪轨迹 |
| Path | 车辆在局部几何上走哪条线？ | 不一定有完整速度曲线 |
| Trajectory | 每个点的位置、姿态、速度如何安排？ | 不是底盘命令 |
| Control Command | 此刻方向盘和加减速怎么打？ | 不是全局规划结果 |

### 4A.1 阶段 0：初始位姿与定位

**这一层解决什么问题**

Autoware 需要先知道“车现在在哪里”。如果当前位姿不对，后面 route、path、trajectory 全都会错，因为所有模块都默认当前位姿是可信输入。

**输入是什么**

- RViz 的 `2D Pose Estimate`。
- 地图坐标系下的初始位姿。
- 定位模块、TF、车辆状态。

**输出是什么**

- 当前车辆位姿。
- `map -> base_link` 或等价 TF。
- `/localization/kinematic_state` 中的 pose 和 twist。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/initialpose` | 点击初始位姿后是否有消息 |
| `/localization/kinematic_state` | 位姿是否持续更新，位置是否落在地图道路附近 |
| `/tf`、`/tf_static` | `map`、`base_link`、传感器坐标系是否连通 |
| `/vehicle/status/velocity_status` | 车辆速度状态是否正常 |

**RViz 图层**

- Vector Map / Lanelet2 Map。
- TF。
- Vehicle Model。
- Localization Pose。
- Pose With Covariance。

**关键参数或配置**

- 地图坐标系是否一致。
- 初始位姿朝向是否接近车道方向。
- 定位模块是否 ready。
- 车辆模型尺寸是否和配置一致。

**正常现象**

- 车辆模型出现在车道附近。
- 朝向和车道方向基本一致。
- `/localization/kinematic_state` 持续发布。
- TF 树没有断。

**异常现象**

- 车辆漂到地图外。
- 车辆朝向反了。
- RViz 中车辆抖动很大。
- `/localization/kinematic_state` 没有数据。
- TF 显示缺失或红色报错。

**优先排查顺序**

1. 先看 RViz 里初始位姿是否点在正确道路上。
2. 再看 `/initialpose` 是否发出。
3. 再看 `/localization/kinematic_state` 是否持续更新。
4. 再看 TF 的 `map -> base_link` 是否存在。
5. 最后看地图坐标系、车辆模型、定位模块日志。

**录制画面建议**

把 RViz 画面放大到地图和车辆，点击 `2D Pose Estimate` 后停 3 秒，让观众看到车辆模型从未知状态变成明确位置。

**讲解备注**

这里不要急着讲规划。先告诉观众：规划控制链路的第一块积木是当前位姿。如果车不知道自己在哪，后面所有“规划到目标点”的动作都没有意义。

### 4A.2 阶段 1：目标点与 Mission Planner

**这一层解决什么问题**

Mission Planner 把 RViz 中点击的目标点，转换成 Lanelet 地图上的 route。它回答的是“从当前车道到目标车道应该经过哪些车道段”。

**输入是什么**

- 当前车辆位姿。
- RViz 点击的目标点。
- Lanelet2 地图拓扑。
- Routing API 或 mission planning 内部 routing 结果。

**输出是什么**

- Route。
- Route marker。
- Goal echo。
- Route state。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/mission_planning/goal` | RViz 目标点是否进入系统 |
| `/planning/mission_planning/echo_back_goal_pose` | 系统是否确认目标点 |
| `/planning/mission_planning/route` | 是否生成 route |
| `/planning/mission_planning/route_marker` | RViz 中路线是否可视化 |
| `/planning/mission_planning/route_state` | route 状态是否成功 |

**RViz 图层**

- Mission Route。
- Goal Pose。
- Lanelet Map。
- Route Marker。

**关键参数或配置**

- Goal topic 是否和 RViz 工具配置一致。
- 起点和终点是否能投影到 lanelet。
- lanelet 方向是否和车辆朝向一致。
- 地图 lanelet 拓扑是否连通。

**正常现象**

- 点击目标点后出现 route marker。
- route 沿着车道中心或车道拓扑连接到目标附近。
- route state 不报错。
- echo back goal 和点击位置接近。

**异常现象**

- 点击目标点没有任何反应。
- route 绕远路。
- route 朝反方向。
- route state 显示不可达或 not ready。
- 目标点在 RViz 里看起来正确，但系统没有接受。

**优先排查顺序**

1. 看 `/planning/mission_planning/goal` 是否收到消息。
2. 看目标点是否落在可达车道附近。
3. 看当前位姿是否在地图车道附近。
4. 看 `/planning/mission_planning/route_state` 的状态。
5. 看 Lanelet 地图拓扑、方向和连通关系。

**录制画面建议**

先关闭大部分 debug 图层，只保留地图、车辆、目标点和 route marker。点击目标点后让 route marker 明显出现在画面中央。

**讲解备注**

可以把 route 类比成导航软件里的“走哪些路段”。导航路线告诉你经过哪几条路，但不会告诉你方向盘每一瞬间转多少。

### 4A.3 阶段 2：Behavior Path Planner

**这一层解决什么问题**

Behavior Path Planner 把车道级 route 转换成局部 path。它开始考虑车辆附近的可行驶区域、车道保持、变道、避障、pull out 等行为模块。

**输入是什么**

- Route。
- 当前车辆位姿和速度。
- Lanelet 地图。
- 感知目标、障碍物、交通规则信息。
- 模块状态和 RTC 审批状态。

**输出是什么**

- Behavior path。
- Path candidate。
- Path reference。
- Drivable area。
- 模块调试 marker。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | 主输出 path 是否存在 |
| `/planning/path_candidate/*` | 候选 path 是否合理 |
| `/planning/path_reference/*` | 参考 path 是否沿车道 |
| behavior planning debug marker | 模块状态、边界、可行驶区域 |

**RViz 图层**

- Behavior Path。
- Path Candidate。
- Path Reference。
- Drivable Area。
- Module Debug Marker。

**关键参数或配置**

- 车道保持、避障、变道、start planner 等模块是否启用。
- drivable area 扩展距离。
- path 采样间隔。
- 安全检查和 RTC 配置。

**正常现象**

- route 生成后，局部 path 稳定出现在车道内。
- path 不频繁跳变。
- drivable area 包住车辆可行驶区域。
- path candidate 和 path reference 大方向一致。

**异常现象**

- 有 route 但没有 path。
- path 突然跳到旁边车道。
- path 和车道边界交叉。
- path candidate 很多但最终 path 不更新。
- RViz 中模块 marker 显示 blocked、unsafe 或 waiting approval。

**优先排查顺序**

1. 确认 route 是否正常。
2. 确认 scenario 是否进入 lane driving。
3. 查看 behavior path 主 topic 是否发布。
4. 查看 candidate/reference/debug marker。
5. 如果有 RTC 或安全检查，先看是否被审批或安全条件卡住。
6. 最后再看单个行为模块参数。

**录制画面建议**

把 route marker 和 behavior path 同时打开，用不同颜色强调：route 是车道序列，path 是局部几何线。

**讲解备注**

这一段要提醒新手：Behavior Path Planner 不是简单沿着 route 中心线走。它会根据当前场景选择局部行为，所以 path 比 route 更接近车辆真正要走的线。

### 4A.4 阶段 3：Pull Out 可选链路

**这一层解决什么问题**

Pull Out 处理车辆从非车道中心、停车区域或自由空间回到正常车道的过程。它是 Start Planner 的典型场景。

**输入是什么**

- 当前车辆位姿。
- 目标车道或目标 path。
- 可行驶区域边界。
- 障碍物。
- 车辆运动学约束。

**输出是什么**

- Pull out path candidate。
- Shift pull out path。
- Geometric / Hybrid A* pull out path。
- 最终被采用的 start path。

**重点 topic**

| topic | 观察重点 |
|---|---|
| start planner candidate | 是否生成多条候选起步路径 |
| pull out debug marker | 哪种 pull out 方法被选择 |
| drivable area / bound | 起步区域是否足够 |
| behavior path output | 最终 path 是否接上车道 |

**RViz 图层**

- Start Planner Debug。
- Pull Out Candidate。
- Drivable Area。
- Obstacle / Predicted Objects。
- Vehicle Footprint。

**关键参数或配置**

- 是否启用 shift pull out、geometric pull out、freespace pull out。
- 横向偏移距离和最大横向 jerk。
- 搜索分辨率。
- Hybrid A* 的栅格分辨率、转角离散、代价权重。
- 起步安全检查距离。

**正常现象**

- 车辆起点偏离车道时，系统能生成从当前位置接回车道的路径。
- 路径满足车辆转弯半径，不会横着平移。
- 候选路径中有一条被选为最终 path。
- 起步路径和后续 lane driving path 平顺连接。

**异常现象**

- start planner 一直没有 path。
- 候选 path 穿过障碍物或边界。
- Hybrid A* 搜索失败。
- 起步 path 转角过大或方向反复切换。
- 车辆明明在路边，却被系统认为已经在 lane driving path 上。

**优先排查顺序**

1. 确认当前位姿是否真的偏离目标车道。
2. 看 start planner 是否启用。
3. 看 drivable area 和边界是否把起步空间包住。
4. 看障碍物是否挡住所有候选路径。
5. 看 shift pull out 是否已经足够，如果不够再看 Hybrid A*。
6. 最后看搜索分辨率、代价权重和车辆最小转弯半径。

**录制画面建议**

准备一个“车停在车道旁边”的短片段。先展示普通 lane driving 没有 pull out 的情况，再打开 pull out candidate，让观众看到系统如何从自由空间接回车道。

**讲解备注**

不要把 Pull Out 讲成“倒车入库算法”。它在本课程里的重点是：结构化道路规划并不总是从车道中心开始，系统需要一个过渡机制把自由空间状态接回车道级规划。

### 4A.5 阶段 4：Elastic Band 路径平滑

**这一层解决什么问题**

Behavior path 往往是离散点列，可能存在折线、采样噪声或曲率不连续。Elastic Band 的作用是让路径像一根被约束在走廊里的弹性带：既贴近原始 path，又尽量平滑，还不能越界。

**输入是什么**

- Behavior path。
- 路径边界或可行驶区域。
- 车辆宽度和安全距离。
- 平滑参数。

**输出是什么**

- 平滑后的 path。
- smoother debug marker。
- 边界约束可视化。

**重点 topic**

| topic | 观察重点 |
|---|---|
| behavior path | 平滑前路径是否正常 |
| path smoother output | 平滑后路径是否更顺 |
| bound / drivable area marker | 约束范围是否合理 |
| debug marker | 是否有失败或越界提示 |

**RViz 图层**

- Before Smoothing Path。
- Smoothed Path。
- Bound / Drivable Area。
- Vehicle Footprint。

**核心公式回忆**

可以用下面的抽象形式讲给新手：

```text
目标函数 = 贴近原路径的代价 + 平滑代价 + 曲率/形状代价
约束 = 不能越过左右边界，不能违反车辆几何限制
```

如果写成 QP 形式，可以表达为：

```text
minimize    1/2 x^T H x + f^T x
subject to  l <= A x <= u
```

- `x`：待优化的路径点坐标或偏移量。
- `H`：二次代价矩阵，决定平滑、贴近原路径等权重。
- `f`：一次项，常用于引导路径靠近参考。
- `A, l, u`：边界、连续性等约束。

**正常现象**

- 平滑后路径比原路径更顺。
- 路径仍然留在可行驶区域内。
- 曲率变化更连续。
- 没有突然贴边或穿出边界。

**异常现象**

- 平滑后仍然锯齿。
- 平滑后离原 path 太远。
- 平滑路径贴边。
- 优化失败或输出为空。

**优先排查顺序**

1. 先看输入 behavior path 是否已经抖动。
2. 再看边界是否过窄或不合理。
3. 再看平滑权重是否过小或过大。
4. 再看车辆尺寸和安全距离是否配置错误。
5. 最后看 QP 求解器日志。

**录制画面建议**

在 RViz 中同时显示平滑前后两条线。可以暂停视频，用鼠标指着折线和被拉顺后的曲线讲“弹性带”的直觉。

**讲解备注**

这里要把公式讲慢：优化不是为了让路径看起来漂亮，而是为了让后面的控制器更容易跟踪。如果 path 的曲率跳变很大，控制器会被迫输出跳变的转向。

### 4A.6 阶段 5：MPT / Path Optimizer

**这一层解决什么问题**

MPT / Path Optimizer 在局部边界内进一步优化轨迹，使它更满足车辆运动学、边界约束和控制可跟踪性。它把“几何上能走”推进到“车辆更容易稳定跟踪”。

**输入是什么**

- 平滑 path 或行为规划 path。
- 车辆当前状态。
- 道路边界。
- 车辆模型参数。
- 优化权重和约束。

**输出是什么**

- 优化后的 trajectory。
- debug marker。
- 优化状态。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path` | MPT 前的输入是否正常 |
| `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | 优化输出是否存在 |
| optimizer debug marker | 边界、车辆圆、优化状态 |
| log | 是否出现 infeasible、timeout、empty input |

**RViz 图层**

- Path Optimizer Trajectory。
- Reference Path。
- Optimized Trajectory。
- Bound。
- Vehicle Circles / Footprint。

**关键参数或配置**

- `option.enable_skip_optimization`。
- `common.output_delta_arc_length`。
- `mpt.weight.steer_input_weight`。
- 边界软约束和硬约束。
- 车辆圆半径和安全距离。
- warm start / fixed point / replan 阈值。

**核心公式回忆**

MPT 的直觉可以讲成：

```text
既不要偏离参考 path 太多，
又要让方向、曲率、转向变化更平顺，
还要保持在道路边界和车辆几何约束内。
```

常见优化目标可以抽象成：

```text
J = w_y * 横向误差^2
  + w_theta * 航向误差^2
  + w_delta * 转向角^2
  + w_d_delta * 转向变化^2
```

这些权重不是为了“追求数学好看”，而是在工程上平衡舒适性、贴近参考线、可跟踪性和边界安全。

**正常现象**

- path optimizer 有稳定输出。
- 输出轨迹比输入 path 更平顺。
- 轨迹不越界。
- 车辆圆没有明显压出可行驶区域。
- 轨迹不会每帧大幅跳变。

**异常现象**

- MPT 无输出。
- 输出轨迹贴边。
- 优化结果每帧跳。
- 轨迹突然变短。
- 日志出现 infeasible 或 timeout。

**优先排查顺序**

1. 看输入 path_smoother/path 是否为空。
2. 看边界是否过窄。
3. 看车辆尺寸和车辆圆是否正确。
4. 看是否因为硬约束导致无解。
5. 看权重是否导致路径过分贴参考或过分避边。
6. 看 warm start / replan 机制是否频繁重置。

**录制画面建议**

打开 reference path、optimized trajectory、bound 和 vehicle footprint，让观众看到“优化不是凭空画线，而是在边界走廊里调整点列”。

**讲解备注**

这里可以连接第 4 集和第 8 集：Elastic Band 更像路径平滑，MPT 更像把平滑路径放进车辆模型和边界约束里再优化一次。二者共同让控制层更好工作。

### 4A.7 阶段 6：Motion Velocity Planner

**这一层解决什么问题**

Motion Velocity Planner 给几何轨迹附加速度约束。它会根据交通规则、障碍物、停止线、曲率限制等，决定哪里该慢、哪里该停、哪里可以按限速走。

**输入是什么**

- 优化后的几何 trajectory。
- 交通规则信息。
- 感知障碍物。
- 地图停止线、红绿灯、横穿区域等。
- 当前车辆速度。

**输出是什么**

- 带速度约束的 lane driving trajectory。
- stop reason。
- slow down reason。
- virtual wall marker。
- velocity factor。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | 输入轨迹是否存在 |
| `/planning/scenario_planning/lane_driving/trajectory` | 速度规划后的轨迹是否存在 |
| stop reason / velocity factor | 为什么停车或减速 |
| virtual wall marker | 停止点在哪里 |

**RViz 图层**

- Lane Driving Trajectory。
- Virtual Wall。
- Stop Reason。
- Velocity Factor。
- Traffic Light / Objects。

**关键参数或配置**

- 速度上限。
- 停止距离。
- 曲率限速。
- obstacle stop / slow down 参数。
- 红绿灯、停止线、crosswalk 等模块开关。

**正常现象**

- 轨迹上有合理速度。
- 接近停止线或障碍物时出现 virtual wall。
- stop reason 和实际场景一致。
- 弯道速度不会明显过快。

**异常现象**

- 车辆突然被规划为 0 速度。
- virtual wall 出现在奇怪位置。
- 没有障碍却一直 slow down。
- 弯道速度过快。
- 速度限制每帧跳变。

**优先排查顺序**

1. 看输入 trajectory 是否正常。
2. 看 virtual wall 和 stop reason。
3. 看感知物体是否误检。
4. 看地图交通规则是否触发。
5. 看速度限制和曲率限速参数。
6. 看上游轨迹曲率是否异常。

**录制画面建议**

把轨迹颜色按速度显示，再打开 virtual wall。让观众看到速度规划不是“给所有点一个固定速度”，而是在不同路段修改速度上限。

**讲解备注**

这一段要强调：速度规划不是控制器做的。控制器只执行最终轨迹，真正决定前方停止点和慢行点的是规划层。

### 4A.8 阶段 7：Velocity Smoother

**这一层解决什么问题**

Velocity Smoother 把上游给出的速度上限和停止要求，整理成车辆能够舒适执行的速度曲线。它重点控制加速度和 jerk，避免速度硬裁剪造成急刹或突变。

**输入是什么**

- 上游 lane driving trajectory。
- 当前车辆速度和加速度估计。
- 最大速度、最大加速度、最大减速度、最大 jerk。
- 停止点和速度限制。

**输出是什么**

- 平滑后的 velocity trajectory。
- 当前最大速度候选。
- 加速度和 jerk 调试信息。
- 最终进入 validator 的轨迹。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/scenario_planning/lane_driving/trajectory` | 输入速度轨迹 |
| `/planning/scenario_planning/velocity_smoother/trajectory` | 平滑后的速度轨迹 |
| current max velocity / candidates | 当前速度上限来源 |
| closest acceleration | 最近点加速度是否合理 |

**RViz 图层**

- Velocity Smoother Trajectory。
- Velocity Colorized Trajectory。
- Speed Limit Marker。
- Stop Point / Slow Down Point。

**关键参数或配置**

- 最大加速度。
- 最大减速度。
- 最大 jerk。
- resample interval。
- engage 相关速度。
- 停止点前缓停距离。

**核心公式回忆**

可以用最简单的离散运动关系解释：

```text
v_{k+1} = v_k + a_k * dt
a_{k+1} = a_k + j_k * dt
```

- `v` 是速度。
- `a` 是加速度。
- `j` 是 jerk，也就是加速度变化率。
- `dt` 是相邻采样点的时间间隔。

工程含义是：速度不能从 10 m/s 瞬间变成 0 m/s。即使前方要停车，也要通过合理的加速度和 jerk 逐步降下来。

**正常现象**

- 速度曲线连续。
- 减速段不是垂直掉到 0。
- 加速度和 jerk 没有明显尖峰。
- 停止点前能提前减速。

**异常现象**

- 速度忽高忽低。
- 接近停止点急刹。
- 速度上限来回跳。
- 起步时速度迟迟上不来。
- 平滑输出比上游轨迹短或断。

**优先排查顺序**

1. 看上游 lane driving trajectory 的速度是否已经突变。
2. 看 velocity smoother 输出是否更平滑。
3. 看当前速度状态是否正确。
4. 看 max accel / max decel / jerk 是否过严或过松。
5. 看停止点是否太近。
6. 看 resample 和时间参数是否导致离散点过稀。

**录制画面建议**

建议同时展示一条“硬裁剪速度”的示意图和一条“平滑减速”的 RViz 轨迹颜色图。先讲直觉，再讲 topic。

**讲解备注**

这里非常适合用乘车体验做类比：速度硬裁剪就像突然踩死刹车，jerk 受限则像司机提前、均匀地减速。

### 4A.9 阶段 8：Planning Validator

**这一层解决什么问题**

Planning Validator 是最终轨迹进入控制器前的一道检查。它会验证轨迹是否存在明显异常，例如轨迹为空、跳变过大、曲率异常、速度异常等。

**输入是什么**

- Velocity Smoother 输出轨迹。
- 当前车辆状态。
- 检查阈值。

**输出是什么**

- 最终 `/planning/trajectory`。
- validator marker。
- 诊断信息。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/scenario_planning/velocity_smoother/trajectory` | validator 输入是否正常 |
| `/planning/trajectory` | 最终轨迹是否发布 |
| `/planning/planning_validator/output/markers` | 是否有异常提示 |
| diagnostics | 是否出现 planning invalid |

**RViz 图层**

- Final Planning Trajectory。
- Planning Validator Marker。
- Diagnostics。

**关键参数或配置**

- 最大轨迹跳变阈值。
- 最大速度、加速度、曲率检查阈值。
- 轨迹长度下限。
- 是否允许紧急停止。

**正常现象**

- `/planning/trajectory` 持续发布。
- final trajectory 和 smoother trajectory 基本一致。
- validator marker 没有错误提示。
- 轨迹长度足够控制器使用。

**异常现象**

- smoother 有轨迹，但 `/planning/trajectory` 没有。
- RViz 出现 validator virtual wall 或 error marker。
- 轨迹突然被截断。
- 控制器因为没有最终轨迹而不输出。

**优先排查顺序**

1. 看 velocity smoother 输出是否正常。
2. 看 validator marker 具体提示。
3. 看轨迹长度、速度、曲率是否超过阈值。
4. 看当前车辆状态是否和轨迹起点偏差过大。
5. 看日志中的 planning invalid 原因。

**录制画面建议**

把 smoother trajectory 和 final `/planning/trajectory` 用不同颜色展示。如果两者一致，说明 validator 只是放行；如果不一致，就说明这里发生了检查或截断。

**讲解备注**

这一层容易被新手忽略。要强调：看到 smoother 有输出，不代表控制器一定能收到最终轨迹。控制器通常看的是 `/planning/trajectory`。

### 4A.10 阶段 9：Trajectory Follower

**这一层解决什么问题**

Trajectory Follower 把最终轨迹和当前车辆状态进行比较，输出当前这一帧的转向、加速度或速度控制命令。

**输入是什么**

- `/planning/trajectory`。
- 当前车辆位姿。
- 当前速度。
- 当前转向角。
- 控制器参数。

**输出是什么**

- `/control/trajectory_follower/control_cmd`。
- predicted trajectory。
- resampled reference trajectory。
- lateral / longitudinal debug 信息。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/planning/trajectory` | 控制器输入轨迹 |
| `/localization/kinematic_state` | 当前位姿和速度 |
| `/vehicle/status/steering_status` | 当前实际转向 |
| `/control/trajectory_follower/control_cmd` | follower 输出命令 |
| control debug topics | 横向误差、航向误差、预测轨迹 |

**RViz 图层**

- Final Trajectory。
- Controller Resampled Reference。
- Predicted Trajectory。
- Lateral Error / Debug Marker。
- Vehicle Footprint。

**关键参数或配置**

- Pure Pursuit lookahead distance。
- MPC 预测时域和采样时间。
- 横向误差权重。
- 航向误差权重。
- 转向输入权重。
- 转向变化惩罚。
- 纵向控制的加速度限制和停止状态参数。

**核心公式回忆**

Pure Pursuit 的几何直觉可以用：

```text
kappa = 2 * y_L / L_d^2
delta = atan(L * kappa)
```

- `kappa` 是目标曲率。
- `y_L` 是前视点在车辆坐标系下的横向偏差。
- `L_d` 是前视距离。
- `L` 是轴距。
- `delta` 是前轮转角。

MPC 的直觉可以用：

```text
在未来 N 步里，
让横向误差、航向误差、转向角、转向变化的加权和尽量小。
```

**正常现象**

- follower control_cmd 持续发布。
- predicted trajectory 和 final trajectory 大方向一致。
- 横向误差逐渐变小或保持较小。
- 转向命令连续，不频繁大幅跳变。

**异常现象**

- `/planning/trajectory` 有数据，但 follower 没有输出。
- 转向抖动。
- 弯道外切。
- 车辆沿着轨迹左右摆动。
- predicted trajectory 明显偏离参考轨迹。

**优先排查顺序**

1. 看 `/planning/trajectory` 是否存在且长度足够。
2. 看当前车辆位姿和轨迹起点是否距离过大。
3. 看车辆速度和 steering status 是否正常。
4. 看 resampled reference 是否平滑。
5. 看 predicted trajectory 是否偏离。
6. 最后调控制器参数，不要一开始就改参数。

**录制画面建议**

把 final trajectory、predicted trajectory、车辆模型同时打开。车辆运动时暂停一帧，讲“控制器在这一刻看到的参考线”和“它预测自己会怎么走”。

**讲解备注**

控制器不是重新规划路线。它只是在当前时刻根据最终轨迹和车辆状态，计算短时间内最合理的执行命令。

### 4A.11 阶段 10：Vehicle Cmd Gate 与底盘命令

**这一层解决什么问题**

Vehicle Cmd Gate 决定控制器输出能不能真正下发到底盘。它会考虑 operation mode、engage、emergency、外部控制源、安全状态等。

**输入是什么**

- Trajectory Follower 的控制命令。
- Operation mode。
- Engage 状态。
- Emergency 状态。
- 车辆接口状态。

**输出是什么**

- gate 后的最终控制命令。
- 底盘接口命令。
- 车辆实际速度和转向状态变化。

**重点 topic**

| topic | 观察重点 |
|---|---|
| `/control/trajectory_follower/control_cmd` | 控制器是否已经输出 |
| `/control/command/control_cmd` | gate 后是否还有命令 |
| operation mode 相关 topic | 是否处于 autonomous |
| engage 相关 topic | 是否已经允许自动驾驶 |
| `/vehicle/status/velocity_status` | 车辆速度是否跟随变化 |
| `/vehicle/status/steering_status` | 实际转向是否跟随变化 |

**RViz 图层**

- Operation Mode / Engage 状态。
- Vehicle Status。
- Control Command Marker。
- Diagnostics。

**关键参数或配置**

- 是否 engage。
- operation mode 是否 autonomous。
- emergency 是否触发。
- command gate 输入源选择。
- 车辆接口是否 ready。

**正常现象**

- follower cmd 和 gate 后 cmd 都有输出。
- engage 后车辆速度或转向状态开始变化。
- diagnostics 没有 emergency。
- operation mode 显示自动驾驶模式。

**异常现象**

- follower 有命令，但 gate 后没有。
- gate 后有命令，但车辆状态不变。
- 系统一直不 engage。
- operation mode 不允许自动驾驶。
- emergency 触发导致命令被拦截。

**优先排查顺序**

1. 看 follower cmd 是否存在。
2. 看 gate 后 `/control/command/control_cmd` 是否存在。
3. 看 operation mode 是否 autonomous。
4. 看 engage 是否成功。
5. 看 emergency 和 diagnostics。
6. 看车辆接口和底盘状态。

**录制画面建议**

终端同时显示 follower cmd 和 gate 后 cmd。然后切到 RViz 或车辆状态界面，让观众看到“控制器算出命令”和“命令真正下发”不是同一件事。

**讲解备注**

这里是实车和仿真调试里非常常见的断点。很多时候不是规划控制算法没算出来，而是系统安全门没有放行。

---

## 4B. 录制时的统一讲法模板

每一段都建议按下面顺序讲，不要直接跳到源码：

```text
1. 现在画面上发生了什么？
2. 这一层的输入是什么？
3. 这一层的输出是什么？
4. 它解决了哪个具体问题？
5. 我们应该看哪些 topic？
6. RViz 哪些图层能验证结果？
7. 正常时应该看到什么？
8. 异常时最常见的现象是什么？
9. 排查时先看上游还是下游？
10. 这一层和前面哪一集课程对应？
```

可以反复使用下面这句过渡话术：

```text
我们先不看源码，先看它在系统里吃什么、吐什么。
只要输入输出关系搞清楚，源码只是把这件事实现出来。
```

### 4B.1 新手友好的三种画面组合

| 画面组合 | 适合讲什么 | 录制建议 |
|---|---|---|
| RViz 全屏 | route、path、trajectory、virtual wall、vehicle footprint | 用鼠标或高亮框指出当前层输出 |
| RViz + 终端 | topic 是否发布、频率是否稳定 | 左边画面看结果，右边终端看数据 |
| PPT + RViz 回放 | 解释抽象概念和链路 | PPT 讲概念，RViz 验证现象 |

### 4B.2 每一层都要提醒观众的学习方法

```text
不要背 topic 名称；
要记住 topic 在链路中的位置。

不要一开始就调参数；
要先找到第一个异常输出。

不要把 route、path、trajectory 混成一个东西；
要问它到底回答哪个问题。
```

### 4B.3 实战课里可以轻带的核心公式

本集不需要重新推导所有公式，但可以在关键处用“工程意义”复习：

| 位置 | 公式或模型 | 工程意义 |
|---|---|---|
| Elastic Band / MPT | `min 1/2 x^T H x + f^T x` | 把平滑、贴近参考、边界约束统一成优化问题 |
| Velocity Smoother | `v_{k+1}=v_k+a_k dt`，`a_{k+1}=a_k+j_k dt` | 速度变化要受加速度和 jerk 限制 |
| Pure Pursuit | `kappa=2y_L/L_d^2`，`delta=atan(L kappa)` | 用前视点几何关系算转向 |
| 横向 MPC | 未来误差加权最小 | 用车辆模型预测未来，并选择更好的控制输入 |

讲公式时建议用下面顺序：

```text
先说变量是什么
再说公式在惩罚什么
再说它对车辆表现有什么影响
最后说在 RViz 或 topic 里如何观察
```

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

## 18A. 常见故障排查决策树

这一节可以作为录课时的“实战排查主菜单”。建议先把决策树放在 PPT 上，再切到 RViz 和终端演示。

### 18A.1 总原则：从现象所在层往上游找第一个异常点

```text
现象发生在车辆执行端
  -> 先看 gate 后命令
  -> 再看 follower 命令
  -> 再看最终轨迹
  -> 再看 velocity smoother
  -> 再看 velocity planner
  -> 再看 path optimizer / smoother
  -> 再看 behavior path
  -> 再看 route
  -> 再看 localization 和 map
```

这样排查的好处是：你不会一看到车辆不动就去改 MPC 参数，也不会一看到路线没有出来就去查控制器。

### 18A.2 症状一：点击目标点后没有 route

```text
没有 route
  -> /planning/mission_planning/goal 是否收到目标点？
      否 -> 检查 RViz Goal 工具的 topic 配置
      是 -> 当前车辆定位是否有效？
          否 -> 回到 /localization/kinematic_state 和 TF
          是 -> 目标点是否落在可达 lanelet 附近？
              否 -> 重新点击目标点，靠近车道中心
              是 -> route_state 是否报错？
                  是 -> 看地图拓扑、lanelet 方向、起终点可达性
                  否 -> 看 route_marker 是否只是 RViz 图层没开
```

**录制讲解备注**

这里要提醒观众：Mission Planner 不是在空白地图上画直线，它依赖 Lanelet 拓扑。如果目标点落不到可达车道上，route 就可能生成不了。

### 18A.3 症状二：有 route，但没有 behavior path

```text
有 route，无 path
  -> scenario 是否进入 lane_driving？
      否 -> 检查场景选择、operation mode、route state
      是 -> behavior path topic 是否发布？
          否 -> 看 behavior planner 日志和模块状态
          是 -> RViz 图层是否打开？
              否 -> 打开 behavior path / candidate / reference
              是 -> path 是否被某个模块卡住？
                  是 -> 看 RTC、safety check、objects_of_interest
                  否 -> 看 drivable area 和 map 边界
```

**录制讲解备注**

这一段适合强调“route 是任务路线，path 是局部行为结果”。有 route 只能说明导航路线成立，不代表局部行为一定能输出。

### 18A.4 症状三：有 path，但优化轨迹没有出来

```text
有 behavior path，无优化 trajectory
  -> path_smoother 输入是否为空？
      是 -> 回查 behavior path 输出
      否 -> path_smoother 输出是否正常？
          否 -> 看边界、平滑权重、输入点是否异常
          是 -> path_optimizer 输入是否正常？
              否 -> 看 topic remap 和模块连接
              是 -> 优化器是否失败？
                  是 -> 看 infeasible / timeout / bound too narrow
                  否 -> 看 RViz 图层是否打开或输出 topic 名称是否不同
```

**录制讲解备注**

这里要把“优化失败”讲成普通工程问题：输入、边界、约束、求解器状态，按顺序查，不要直接把它想成数学太难。

### 18A.5 症状四：有轨迹，但速度不合理

```text
轨迹存在，速度异常
  -> 异常是突然 0 速度吗？
      是 -> 看 virtual wall / stop reason / velocity factor
      否 -> 是弯道过快吗？
          是 -> 看曲率限速和 lateral acceleration limit
          否 -> 是速度忽快忽慢吗？
              是 -> 看上游速度限制是否跳变
              否 -> 看 velocity smoother 输出和当前车辆速度
```

继续细查：

```text
Velocity Smoother 输出异常
  -> 上游 lane_driving trajectory 是否已经异常？
      是 -> 回到 Motion Velocity Planner
      否 -> 当前速度状态是否正确？
          否 -> 查 vehicle velocity status
          是 -> jerk / acceleration 参数是否过严？
              是 -> 解释参数影响，谨慎调整
              否 -> 查停止点距离和重采样间隔
```

**录制讲解备注**

速度问题不要只盯着 smoother。很多“急刹”其实是上游突然给了一个很近的停止点，smoother 只能在约束内尽力减速。

### 18A.6 症状五：有 velocity smoother 输出，但没有最终 `/planning/trajectory`

```text
smoother 有输出，final trajectory 没有
  -> Planning Validator marker 是否报错？
      是 -> 读取 marker 文本或 diagnostics
      否 -> /planning/trajectory topic 是否存在？
          否 -> 检查 launch、remap、validator 是否启动
          是 -> 是否频率很低或间断？
              是 -> 看输入轨迹是否间断、CPU 是否超时
              否 -> 检查 RViz 图层配置
```

可能的 validator 拦截原因：

- 轨迹为空或长度太短。
- 轨迹点跳变过大。
- 速度、加速度或曲率超过阈值。
- 当前车辆位置离轨迹太远。
- 上游轨迹时间戳或坐标系异常。

**录制讲解备注**

要强调：最终给控制器的一般是 `/planning/trajectory`，不是中间某个 smoother topic。中间轨迹看起来正常，最终轨迹没过检查，控制器仍然不会正常工作。

### 18A.7 症状六：有最终轨迹，但控制器没有输出

```text
/planning/trajectory 有，follower cmd 没有
  -> 控制器节点是否运行？
      否 -> 检查 launch 和节点状态
      是 -> 当前定位和车辆状态是否有效？
          否 -> 查 /localization/kinematic_state、velocity_status、steering_status
          是 -> 轨迹起点离车辆是否太远？
              是 -> 回查 planning output 与定位
              否 -> 轨迹长度和时间戳是否有效？
                  否 -> 回查 validator 和 trajectory message
                  是 -> 看控制器日志和 debug topic
```

**录制讲解备注**

这里很适合讲“控制器不是魔法”。控制器必须同时拿到参考轨迹、当前位姿、当前速度、当前转向，缺一个都可能不输出。

### 18A.8 症状七：控制器有输出，但车不动

```text
follower cmd 有，车辆不动
  -> /control/command/control_cmd 是否有？
      否 -> command gate 拦住了
          -> 看 engage、operation mode、emergency、gate 输入源
      是 -> 车辆接口是否收到命令？
          否 -> 查 vehicle interface / bridge / simulator
          是 -> 车辆状态是否变化？
              否 -> 查底盘使能、控制模式、急停状态
              是 -> 可能只是速度太小或被限速
```

**录制讲解备注**

这一段要给新手建立一个很重要的边界：控制器输出和车辆执行之间还有安全门。自动驾驶系统为了安全，不会因为控制器算出命令就无条件下发。

### 18A.9 症状八：车辆能动，但表现不好

```text
车辆能动，但表现不好
  -> 是转向抖动？
      -> 看 final trajectory 曲率
      -> 看 resampled reference
      -> 看 predicted trajectory
      -> 再看 MPC/Pure Pursuit 参数

  -> 是弯道外切？
      -> 看速度是否过高
      -> 看轨迹曲率是否过大
      -> 看转向限制
      -> 看 MPC 预测时域和权重

  -> 是急刹？
      -> 看 virtual wall / stop reason
      -> 看停止点距离
      -> 看 velocity smoother jerk / decel
      -> 看当前速度估计

  -> 是路线绕远或方向错？
      -> 回到 Mission Planner
      -> 看 lanelet 方向、起终点投影、目标朝向
```

**录制讲解备注**

这部分可以作为全课总结：车辆表现不好不一定是控制器问题，可能是路线、路径、速度、轨迹或车辆状态任意一层传下来的问题。

### 18A.10 排查时最容易犯的错误

| 错误做法 | 为什么容易误导 | 更好的做法 |
|---|---|---|
| 车辆不动就改 MPC 参数 | 可能根本没有最终命令下发 | 先看 follower cmd 和 gate cmd |
| 没有 route 就查 controller | controller 还没有输入 | 先看 goal、route_state、lanelet |
| 急刹就改 jerk | 停止点可能太近 | 先看 virtual wall 和 stop reason |
| path 抖就改控制器 | 控制器只是跟踪输入 | 先看 behavior path 和 smoother |
| RViz 没显示就认为没输出 | 可能只是图层没开或 topic 名不同 | 用 `ros2 topic list/hz/echo` 复核 |

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

## 19A. 录制操作步骤、画面建议和讲解备注

这一节是给正式录课用的“导演表”。你可以把它放在旁边当提词器，按顺序录，不容易漏掉关键观察点。

| 步骤 | 操作 | 画面建议 | 讲解备注 | 停顿检查 |
|---|---|---|---|---|
| 1 | 启动仿真或实车回放环境 | RViz 显示地图和车辆，终端显示关键节点已启动 | 先告诉观众本集不是讲单个算法，而是看完整链路 | 确认地图、TF、车辆模型正常 |
| 2 | 打开规划控制相关 RViz 图层 | 左侧图层面板保持可见 5 秒 | 让观众知道后面每一种颜色代表哪类数据 | route/path/trajectory/control 图层不要混色太接近 |
| 3 | 设置初始位姿 | 车辆模型从未定位状态变到车道附近 | 当前位姿是全链路起点，定位错后面都会错 | `/localization/kinematic_state` 持续发布 |
| 4 | 点击目标点 | 目标点和 route marker 出现 | route 只回答经过哪些车道，不是控制器跟踪线 | `/planning/mission_planning/route_state` 正常 |
| 5 | 打开 behavior path | route 和 path 同屏显示 | 用两条线对比 route 和 path 的区别 | behavior path 不为空、不乱跳 |
| 6 | 如果有起步偏移，展示 Pull Out | 显示候选路径、边界、车辆 footprint | Pull Out 是把自由空间状态接回车道结构 | candidate 是否生成、是否被安全检查卡住 |
| 7 | 打开 Elastic Band / MPT 输出 | 平滑前后路径或优化前后轨迹同屏 | 优化不是换路线，而是在约束内让轨迹更可跟踪 | path optimizer trajectory 是否稳定 |
| 8 | 打开速度规划图层 | 轨迹颜色按速度变化，virtual wall 可见 | 速度是规划层加上去的，不是控制器临时决定的 | stop reason 和场景是否一致 |
| 9 | 打开 velocity smoother 输出 | 对比 smoother 前后速度变化 | 速度不能硬裁剪，要满足加速度和 jerk | 平滑输出是否连续 |
| 10 | 打开 final trajectory 和 validator | final trajectory 叠加在 smoother 输出上 | 控制器通常吃最终 `/planning/trajectory` | validator 是否报错 |
| 11 | 打开 controller debug | final、reference、predicted trajectory 同屏 | 控制器根据当前状态跟踪最终轨迹 | follower cmd 是否持续发布 |
| 12 | 打开 command gate 和车辆状态 | 终端显示 follower cmd、gate cmd、vehicle status | 控制器输出不等于底盘一定执行，中间还有安全门 | gate 后命令和车辆状态是否变化 |

### 19A.1 推荐录制窗口布局

**布局 A：讲主链路**

```text
左侧 70%：RViz
右侧 30%：终端
终端轮流显示：
  ros2 topic hz /planning/trajectory
  ros2 topic hz /control/trajectory_follower/control_cmd
  ros2 topic echo /planning/mission_planning/route_state
```

适合录制 route、path、trajectory、control command 连续出现的过程。

**布局 B：讲故障排查**

```text
左侧 50%：RViz
右上 25%：topic hz / echo
右下 25%：日志或 diagnostics
```

适合演示“看到现象 -> 找 topic -> 看 marker -> 判断上游或下游”的排查过程。

**布局 C：讲公式和概念**

```text
左侧：PPT 公式或链路图
右侧：RViz 对应现象
```

适合讲 Velocity Smoother、MPT、MPC 这类容易抽象的内容。公式不要单独停留太久，要尽快回到 RViz 现象。

### 19A.2 每一步的推荐口播节奏

可以用下面的节奏模板：

```text
第一句：现在我们在看哪一层？
第二句：这一层的输入是什么？
第三句：这一层输出到哪个 topic？
第四句：RViz 里正常应该看到什么？
第五句：如果这里不正常，先查哪三个地方？
```

例如讲 Mission Planner：

```text
现在我们在看 Mission Planner。
它的输入是当前定位、目标点和 Lanelet 地图。
它的输出是 route，也就是 /planning/mission_planning/route。
RViz 里正常会看到 route marker 沿着车道出现。
如果没有 route，我们先查 goal topic，再查定位，最后查目标点是否落在可达 lanelet 上。
```

### 19A.3 录制时建议保留的慢镜头

| 慢镜头位置 | 为什么要慢 | 建议停留 |
|---|---|---|
| 初始位姿点击后 | 新手要看到定位是链路起点 | 3 到 5 秒 |
| 目标点点击后 route 出现 | 强化 route 的概念 | 5 秒 |
| route 和 behavior path 同屏 | 避免把 route 和 path 混淆 | 8 秒 |
| 平滑前后路径对比 | 让优化目标变得可见 | 8 到 10 秒 |
| virtual wall 出现 | 说明停止点来自规划层 | 5 秒 |
| smoother 前后速度对比 | 说明速度曲线不能硬裁剪 | 8 秒 |
| predicted trajectory 和 reference 同屏 | 说明控制器在预测未来 | 8 秒 |
| follower cmd 与 gate cmd 对比 | 说明安全门的存在 | 8 秒 |

### 19A.4 正式录制前检查清单

| 检查项 | 通过标准 |
|---|---|
| 地图 | Lanelet 地图正常显示，车辆在道路附近 |
| TF | `map` 到 `base_link` 连通，RViz 不报 frame 错误 |
| 定位 | `/localization/kinematic_state` 有稳定频率 |
| Route | 点击目标后 route marker 出现 |
| Behavior Path | path/candidate/reference 至少能看到主输出 |
| MPT | path optimizer trajectory 有输出 |
| Velocity | virtual wall、velocity trajectory 可视化正常 |
| Final Trajectory | `/planning/trajectory` 持续发布 |
| Control | follower cmd 和 gate cmd 都可观察 |
| 车辆状态 | velocity_status、steering_status 有变化 |
| 终端 | 常用命令提前准备好，避免录制时临时输入出错 |
| PPT | 链路图、topic 表、故障树页准备好 |

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

---

## 23. 下一次给 Codex 的指令

完成本详解版之后，建议下一步生成一份正式录课前的总检查清单。它不再扩展算法内容，而是帮助你把所有课程材料、RViz 演示、topic 命令、故障案例和发布准备统一检查一遍。

你可以复制下面这段给 Codex：

```text
请执行优化任务 11：
在不覆盖原文件的前提下，
根据 AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md、
AUTOWARE规划控制层课程总索引.md、
以及当前根目录第1集到第8集的新手友好详解版 PPT，
生成 AUTOWARE规划控制层课程录制总检查清单.md。

要求：
1. 面向正式录课前检查；
2. 按“素材检查、PPT检查、公式检查、RViz演示检查、topic检查、参数检查、故障案例检查、发布检查”组织；
3. 每一项给出检查方法和通过标准；
4. 保持新手友好，不精简关键内容；
5. 输出到项目根目录，不覆盖原文件；
6. 任务完成后，给出下一次任务的指令。
```
