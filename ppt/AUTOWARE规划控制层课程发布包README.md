# Autoware 规划控制层课程发布包 README

> 面向对象：准备发布课程的作者、拿到课程资料后自学的观众、希望按资料复现 Autoware 规划控制链路的学习者。  
> 课程范围：第 1 集到第 8 集 + 端到端实战复盘 + 录制总检查清单。  
> 使用原则：这套资料不追求“压缩到最短”，而是尽量把新手学习 Autoware 规划控制时最容易卡住的概念、公式、topic、RViz 图层和排查路径讲清楚。

---

## 0. 这份 README 怎么用

如果你是课程作者：

1. 先看 **推荐目录结构**，把视频、PPT、concept 文档、实战复盘和检查清单整理好。
2. 再看 **文件用途表**，确认每个文件在发布包里应该怎么介绍。
3. 录视频前看 **录制总检查清单**。
4. 发布视频前套用 **视频简介模板** 和 **章节时间戳模板**。
5. 发布后根据 **后续补充专题规划** 决定下一批内容。

如果你是观众：

1. 先看 `AUTOWARE规划控制层课程总索引.md`。
2. 再按第 1 集到第 8 集学习 PPT。
3. 学完主线后看 `AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md`。
4. 真正动手调试时看 `AUTOWARE规划控制层课程录制总检查清单.md` 中的 topic、RViz、参数、故障部分。

---

## 1. 推荐发布包目录结构

建议发布时不要把所有文件平铺在一个目录里。观众拿到资料后，最怕的是“不知道先打开哪个”。推荐结构如下：

```text
autoware_planning_control_course/
  README.md
  00_课程总览/
    AUTOWARE规划控制层课程总索引.md
    AUTOWARE规划控制层课程发布包README.md
    AUTOWARE规划控制层课程录制总检查清单.md

  01_PPT_新手友好详解版/
    AUTOWARE规划控制层第1集_规划控制总览PPT_优化版.md
    AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT_新手友好详解版.md
    AUTOWARE规划控制层第3集_HybridAStarPullOutPPT_新手友好详解版.md
    AUTOWARE规划控制层第4集_ElasticBand路径平滑PPT_新手友好详解版.md
    AUTOWARE规划控制层第5集_VelocitySmoother速度平滑PPT_新手友好详解版.md
    AUTOWARE规划控制层第6集_PurePursuit横向控制PPT_新手友好详解版.md
    AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT_新手友好详解版.md
    AUTOWARE规划控制层第8集_MPT与整套链路复盘PPT_新手友好详解版.md

  02_端到端实战/
    AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md
    AUTOWARE规划控制层端到端实战复盘_视频脚本与PPT大纲.md

  03_concept_原理素材/
    lanelet.md
    mission_planner_route_planning_blog.md
    hybrid_astar_freespace_pull_out_algorithm_blog.md
    start_planner_module_algorithm_blog.md
    shift_pull_out_algorithm_blog.md
    autoware_elastic_band_smoothing_blog.md
    QP求解.md
    velocity_smoother_blog.md
    从速度上限到 Jerk 解析减速：Autoware Velocity Smoother 的完整数学链条.md
    jerk.md
    pure_pursuit_blog.md
    pure pursuit 曲率计算.md
    KinematicsBicycleModel.md
    阿克曼转向模型.md
    从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md
    autoware_mpt_blog.md

  04_视频发布材料/
    视频简介模板.md
    章节时间戳模板.md
    资料下载说明模板.md
    课程封面文案模板.md

  05_后续补充专题/
    RViz调试专题_规划中/
    rosbag复盘专题_规划中/
    常见故障案例专题_规划中/
    源码导读专题_规划中/
```

### 1.1 当前项目中对应的实际文件

当前项目根目录已经包含核心发布材料：

| 类别 | 当前文件 | 发布包建议位置 |
|---|---|---|
| 课程入口 | `AUTOWARE规划控制层课程总索引.md` | `00_课程总览/` |
| 发布 README | `AUTOWARE规划控制层课程发布包README.md` | `00_课程总览/` 或发布包根目录 `README.md` |
| 录制检查 | `AUTOWARE规划控制层课程录制总检查清单.md` | `00_课程总览/` |
| 第 1 集 PPT | `AUTOWARE规划控制层第1集_规划控制总览PPT_优化版.md` | `01_PPT_新手友好详解版/` |
| 第 2 集 PPT | `AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 3 集 PPT | `AUTOWARE规划控制层第3集_HybridAStarPullOutPPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 4 集 PPT | `AUTOWARE规划控制层第4集_ElasticBand路径平滑PPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 5 集 PPT | `AUTOWARE规划控制层第5集_VelocitySmoother速度平滑PPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 6 集 PPT | `AUTOWARE规划控制层第6集_PurePursuit横向控制PPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 7 集 PPT | `AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 第 8 集 PPT | `AUTOWARE规划控制层第8集_MPT与整套链路复盘PPT_新手友好详解版.md` | `01_PPT_新手友好详解版/` |
| 实战复盘 | `AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md` | `02_端到端实战/` |
| 原理素材 | `concept/` 目录下规划控制文档 | `03_concept_原理素材/` |

---

## 2. 文件用途、适合人群和阅读顺序

### 2.1 总览类文件

| 文件 | 用途 | 适合谁看 | 建议阅读顺序 |
|---|---|---|---|
| `AUTOWARE规划控制层课程总索引.md` | 整套课程地图，说明第 1 到第 8 集和端到端实战的关系 | 所有人，尤其是第一次打开资料包的观众 | 第 1 个看 |
| `AUTOWARE规划控制层课程发布包README.md` | 发布包入口，说明目录结构、文件用途、视频发布模板 | 课程作者、助教、观众 | 第 2 个看 |
| `AUTOWARE规划控制层课程录制总检查清单.md` | 正式录制前检查，也可作为学习者自查表 | 课程作者、想复现实操的观众 | 录制前或实操前看 |

### 2.2 PPT 课程文件

| 集数 | 文件 | 主题 | 用途 | 适合谁看 | 建议阅读顺序 |
|---|---|---|---|---|---|
| 第 1 集 | `AUTOWARE规划控制层第1集_规划控制总览PPT_优化版.md` | 规划控制总览 | 建立 Route、Path、Trajectory、Control Command 的分层概念 | 新手、所有后续课程学习者 | 先看 |
| 第 2 集 | `AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT_新手友好详解版.md` | Lanelet 与 Mission Planner | 解释地图车道图如何生成车道级 route | 想理解地图、导航、路线规划的人 | 第 1 集后看 |
| 第 3 集 | `AUTOWARE规划控制层第3集_HybridAStarPullOutPPT_新手友好详解版.md` | Hybrid A* Pull Out | 解释车辆从自由空间回到车道的搜索和安全检查 | 想理解起步规划、自由空间搜索的人 | 第 2 集后看 |
| 第 4 集 | `AUTOWARE规划控制层第4集_ElasticBand路径平滑PPT_新手友好详解版.md` | Elastic Band 路径平滑 | 从离散点、横向偏移、二阶差分讲到 QP 平滑 | 想入门路径平滑和 QP 的人 | 第 3 集后看 |
| 第 5 集 | `AUTOWARE规划控制层第5集_VelocitySmoother速度平滑PPT_新手友好详解版.md` | Velocity Smoother | 解释速度为什么不能硬裁剪，以及加速度、jerk、曲率限速 | 想理解速度规划和舒适性的人 | 第 4 集后看 |
| 第 6 集 | `AUTOWARE规划控制层第6集_PurePursuit横向控制PPT_新手友好详解版.md` | Pure Pursuit | 用几何直觉解释前瞻点、曲率和转角 | 想先从简单控制器入门的人 | 可在第 5 集后看，也可提前看 |
| 第 7 集 | `AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT_新手友好详解版.md` | 横向 MPC | 从阿克曼、自行车模型、误差模型、线性化、离散化讲到 QP | 想深入控制和优化的人 | 建议看完第 6 集后看 |
| 第 8 集 | `AUTOWARE规划控制层第8集_MPT与整套链路复盘PPT_新手友好详解版.md` | MPT 与链路复盘 | 解释 MPT 如何连接规划和控制，并复盘 Elastic Band、MPT、MPC 的边界 | 已看过前 7 集的人 | 最后看 |

### 2.3 端到端实战文件

| 文件 | 用途 | 适合谁看 | 建议阅读顺序 |
|---|---|---|---|
| `AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md` | 从目标点到控制命令的完整案例，包含每阶段 topic、RViz、参数、正常异常和排查顺序 | 学完主线后想实操、调试、录制演示的人 | 第 8 集后看 |
| `AUTOWARE规划控制层端到端实战复盘_视频脚本与PPT大纲.md` | 原始视频脚本与 PPT 大纲 | 想对照旧版结构或二次整理的人 | 可选阅读 |

### 2.4 concept 原理素材

| 文件 | 对应课程 | 用途 | 适合谁看 |
|---|---|---|---|
| `concept/lanelet.md` | 第 2 集 | Lanelet 基础概念、边界、中心线、地图结构 | 想补地图基础的人 |
| `concept/mission_planner_route_planning_blog.md` | 第 2 集 | Mission Planner、routing graph、route section、目标点处理 | 想读 route 代码前的人 |
| `concept/hybrid_astar_freespace_pull_out_algorithm_blog.md` | 第 3 集 | Hybrid A* 自由空间 Pull Out 算法细节 | 想理解搜索算法的人 |
| `concept/start_planner_module_algorithm_blog.md` | 第 3 集 | Start Planner 模块职责和起步规划流程 | 想理解 Pull Out 上下文的人 |
| `concept/shift_pull_out_algorithm_blog.md` | 第 3 集 | Shift Pull Out 方法和适用边界 | 想区分不同 Pull Out 方法的人 |
| `concept/autoware_elastic_band_smoothing_blog.md` | 第 4 集、第 8 集 | Elastic Band 平滑原理和 QP 构造 | 想学路径平滑的人 |
| `concept/QP求解.md` | 第 4 集、第 7 集、第 8 集 | QP 标准形式、矩阵、约束、求解器输入 | 想补优化基础的人 |
| `concept/velocity_smoother_blog.md` | 第 5 集 | Velocity Smoother 模块流程和工程细节 | 想调速度曲线的人 |
| `concept/从速度上限到 Jerk 解析减速：Autoware Velocity Smoother 的完整数学链条.md` | 第 5 集 | 速度上限、jerk、解析减速推导 | 想深入公式的人 |
| `concept/jerk.md` | 第 5 集 | jerk 的物理含义和乘坐体验 | 新手补直觉 |
| `concept/pure_pursuit_blog.md` | 第 6 集 | Pure Pursuit 工程流程 | 想理解控制器实现的人 |
| `concept/pure pursuit 曲率计算.md` | 第 6 集 | Pure Pursuit 曲率公式推导 | 想补几何推导的人 |
| `concept/KinematicsBicycleModel.md` | 第 3、6、7、8 集 | 运动学自行车模型 | 想统一车辆模型符号的人 |
| `concept/阿克曼转向模型.md` | 第 7 集 | 阿克曼转向几何 | 想理解转角和曲率关系的人 |
| `concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md` | 第 7 集、第 8 集 | 横向 MPC 从几何到 QP 的完整数学链条 | 想深入 MPC 的人 |
| `concept/autoware_mpt_blog.md` | 第 8 集 | MPT 轨迹优化原理和链路位置 | 想理解规划和控制连接的人 |

---

## 3. 推荐阅读顺序

### 3.1 新手完整学习路线

适合第一次系统学习 Autoware 规划控制的人：

```text
课程总索引
  -> 第 1 集：规划控制总览
  -> 第 2 集：Lanelet 与 Mission Planner
  -> 第 3 集：Hybrid A* Pull Out
  -> 第 4 集：Elastic Band 路径平滑
  -> 第 5 集：Velocity Smoother
  -> 第 6 集：Pure Pursuit
  -> 第 7 集：横向 MPC
  -> 第 8 集：MPT 与整套链路复盘
  -> 端到端实战复盘
  -> 录制总检查清单中的 topic / RViz / 故障排查部分
```

这条路线最稳。它先建立全局地图，再从路线、路径、速度、控制逐步深入，最后用端到端实战把所有模块串起来。

### 3.2 只想先看规划

适合已经知道控制器大概是什么，但想先搞懂 Autoware 规划链路的人：

```text
第 1 集
  -> 第 2 集
  -> 第 3 集
  -> 第 4 集
  -> 第 5 集
  -> 第 8 集中的 MPT 和链路复盘部分
  -> 端到端实战复盘中的 Route / Path / Trajectory / Velocity 部分
```

学习重点：

- Lanelet 地图如何变成 route。
- route 如何变成 behavior path。
- path 为什么要平滑和优化。
- 速度为什么不能直接硬裁剪。
- MPT 为什么处在规划和控制之间。

### 3.3 只想先看控制

适合已经有轨迹规划基础，但想理解 Pure Pursuit 和 MPC 的人：

```text
第 1 集
  -> 第 6 集
  -> 第 7 集
  -> 第 8 集中的 MPT / MPC 对比
  -> 端到端实战复盘中的 Trajectory Follower / Cmd Gate 部分
```

学习重点：

- 轨迹进入控制器前是什么形态。
- Pure Pursuit 如何用几何直觉算转角。
- MPC 如何用模型预测未来误差。
- 控制器输出为什么还不一定等于底盘执行。

### 3.4 想直接做实操排查

适合已经跑过 Autoware，但 route/path/trajectory/control 经常看不懂的人：

```text
第 1 集
  -> 端到端实战复盘
  -> 课程录制总检查清单第 5、6、7、8 章
  -> 根据卡住的位置回看对应单集
```

如果现象是：

| 现象 | 建议回看 |
|---|---|
| 点击目标没有 route | 第 2 集 + 端到端 Mission Planner 段 |
| 车辆从路边起步失败 | 第 3 集 + 端到端 Pull Out 段 |
| path 锯齿或贴边 | 第 4 集、第 8 集 + 端到端 MPT 段 |
| 急刹或速度跳变 | 第 5 集 + 端到端 Velocity 段 |
| 转向抖动或弯道外切 | 第 6、7 集 + 端到端 Controller 段 |
| follower 有命令但车不动 | 端到端 Cmd Gate 段 |

### 3.5 想读源码

建议不要一开始就读源码。先按下面路线：

```text
课程总索引
  -> 对应集 PPT 的“模块卡片”
  -> 对应 concept 文档
  -> 端到端实战中的 topic 链路
  -> launch / node / callback / core function
```

读源码时每个模块都问六个问题：

1. 输入 topic 或输入数据是什么？
2. 输出 topic 或输出数据是什么？
3. 上游是谁？
4. 下游是谁？
5. 核心模型、公式或优化问题是什么？
6. 出错时能在 RViz 或 topic 中看到什么？

---

## 4. 推荐录制和发布顺序

### 4.1 录制顺序

推荐按下面顺序录：

```text
第 1 集：总览
第 2 集：Lanelet 与 Mission Planner
第 3 集：Hybrid A* Pull Out
第 4 集：Elastic Band
第 5 集：Velocity Smoother
第 6 集：Pure Pursuit
第 7 集：横向 MPC
第 8 集：MPT 与链路复盘
端到端实战复盘
```

原因：

- 第 1 集先建立大地图。
- 第 2 到第 5 集讲规划侧。
- 第 6 到第 7 集讲控制侧。
- 第 8 集把 MPT 和整套链路复盘。
- 端到端实战把理论放回真实运行链路。

### 4.2 发布顺序

发布顺序建议和录制顺序一致。不要先发第 7 集或第 8 集，因为 MPC 和 MPT 对新手来说概念密度最高，如果没有前面的 Route、Path、Trajectory 和车辆模型铺垫，容易劝退。

### 4.3 每集建议标题

| 集数 | 建议视频标题 |
|---|---|
| 第 1 集 | Autoware 规划控制总览：目标点如何一步步变成控制命令 |
| 第 2 集 | Autoware Lanelet 与 Mission Planner：地图车道图如何生成 Route |
| 第 3 集 | Autoware Hybrid A* Pull Out：车辆如何从自由空间回到车道 |
| 第 4 集 | Autoware Elastic Band 路径平滑：从离散点到 QP |
| 第 5 集 | Autoware Velocity Smoother：速度曲线为什么不能硬裁剪 |
| 第 6 集 | Autoware Pure Pursuit：用几何直觉理解横向控制 |
| 第 7 集 | Autoware 横向 MPC：从阿克曼几何到 QP 求解器输入 |
| 第 8 集 | Autoware MPT 与整套链路复盘：轨迹优化如何连接规划和控制 |
| 实战复盘 | Autoware 规划控制端到端实战：从目标点到控制命令的完整排查 |

---

## 5. 视频简介模板

### 5.1 单集视频简介模板

```text
本集是 Autoware 规划控制层课程的第【集数】集，主题是【主题】。

这集主要解决一个问题：
【用一句话写核心问题】

我们会先用新手能理解的方式建立直觉，再逐步进入公式、模块输入输出和 Autoware 工程链路。

本集你会学到：
1. 【知识点 1】
2. 【知识点 2】
3. 【知识点 3】
4. 【知识点 4】
5. 【常见误区或排查点】

建议前置知识：
- 【前置课程或概念】

配套资料：
- PPT：【PPT 文件名】
- 原理文档：【concept 文件名】
- 课程总索引：AUTOWARE规划控制层课程总索引.md

说明：
不同 Autoware 版本的 topic、参数和 launch 名称可能有差异，实操时建议用 ros2 topic list、RViz 图层和本地参数文件复核。
```

### 5.2 第 1 集简介示例

```text
本集是 Autoware 规划控制层课程的第 1 集，主题是规划控制总览。

这集主要解决一个问题：
为什么用户点击一个目标点后，车辆不能直接生成控制命令，而要经过 Route、Path、Trajectory 和 Control Command 多层转换？

我们会先建立整套规划控制的数据链路，再解释 Mission Planner、Behavior Planner、Path Smoother、Velocity Smoother、Controller 在链路中的位置。

本集你会学到：
1. Route、Path、Trajectory、Control Command 的区别。
2. Mission Planner 为什么只输出车道级 route。
3. Path 和 Trajectory 为什么不是一个概念。
4. 控制器为什么只跟踪最终轨迹，而不是重新规划路线。
5. 调试 Autoware 时为什么要先按链路找第一个异常 topic。
```

### 5.3 端到端实战复盘简介示例

```text
本集是 Autoware 规划控制层课程的端到端实战复盘。

这集不再单独讲一个新算法，而是用一个从 RViz 点击目标点到控制命令输出的完整案例，把 Route、Path、Trajectory、Velocity、Validation、Control 和 Vehicle Cmd Gate 串起来。

你会看到每个阶段应该观察哪些 topic、打开哪些 RViz 图层、关注哪些关键参数，以及出现常见故障时应该按什么顺序排查。

本集你会学到：
1. 初始位姿、目标点、route、path、trajectory、control command 如何一步步出现。
2. 每一层的输入、输出和正常现象。
3. 有 route 无 path、有 trajectory 无 control、有 follower cmd 但车不动时如何排查。
4. 为什么不要一上来就调 MPC 参数。
5. 如何沿着 topic 链路找到第一个异常点。
```

---

## 6. 章节时间戳模板

### 6.1 通用章节模板

适合第 1 到第 8 集：

```text
00:00 开场：本集要解决的问题
03:00 本集在 Autoware 规划控制链路中的位置
06:00 新手先记住的几个核心词
10:00 模块输入、输出和工程职责
15:00 核心直觉：先看图，不急着看公式
25:00 核心公式 1：变量、物理意义和工程作用
35:00 核心公式 2：从单步到完整形式
45:00 Autoware 中的模块流程
55:00 关键参数和 RViz / topic 观察点
65:00 常见误区和故障现象
75:00 本集总结图
80:00 下一集预告
```

### 6.2 公式密集集模板

适合第 4、5、7、8 集：

```text
00:00 本集问题：为什么需要这个优化或平滑模块
04:00 系统位置：上游输入和下游输出
08:00 直觉图：先看几何或速度曲线
15:00 最小例子：三个点 / 一段减速 / 单步模型
25:00 核心变量：状态、输入、偏移、速度或误差
35:00 公式慢读：每个符号是什么意思
50:00 从单步到多步：为什么会出现矩阵
65:00 标准 QP 或约束形式
75:00 参数直觉：调大调小会发生什么
85:00 RViz 和 topic 中如何观察
95:00 常见故障和排查顺序
```

### 6.3 端到端实战复盘模板

```text
00:00 本集目标：从目标点到控制命令
04:00 案例设定和 RViz 图层准备
08:00 初始位姿：Localization 和 TF
14:00 目标点：Mission Planner 生成 route
22:00 Behavior Path：route 如何变成局部 path
30:00 Pull Out：车辆偏离车道时如何接回车道
38:00 Elastic Band / MPT：路径如何变得更可跟踪
50:00 Motion Velocity Planner：停止点、慢行点和 virtual wall
60:00 Velocity Smoother：速度曲线为什么要平滑
70:00 Planning Validator：最终轨迹发布前的检查
78:00 Trajectory Follower：轨迹如何变成控制命令
86:00 Vehicle Cmd Gate：为什么控制器输出不等于车辆一定动
94:00 故障排查决策树：从下游往上游找第一个异常 topic
105:00 总结：以后调试 Autoware 应该先看什么
```

### 6.4 短视频切片模板

如果后续要把长课切成短视频，可以用下面模板：

```text
标题：【一个具体问题】
00:00 现象
00:30 它属于链路中的哪一层
01:00 正常输入是什么
01:30 正常输出是什么
02:00 应该看哪个 topic
02:30 RViz 中应该看哪一层
03:00 常见异常
03:30 优先排查顺序
04:30 一句话总结
```

适合切片的问题：

- 为什么目标点不能直接变成控制命令？
- Route、Path、Trajectory 到底有什么区别？
- 为什么有 route 但没有 path？
- 为什么速度不能硬裁剪？
- Pure Pursuit 为什么追前方点？
- MPC 为什么只执行第一个控制输入？
- follower cmd 有了，为什么车还不动？

---

## 7. 资料下载说明模板

### 7.1 视频简介中的资料说明

```text
配套资料说明：

本课程配套 Markdown 资料包含：
1. 每集 PPT 制作稿：适合跟着视频看，也适合复习。
2. concept 原理文档：适合想深入公式和源码前置知识的同学。
3. 端到端实战复盘：适合实际跑 Autoware 时查 topic、RViz 图层和排查顺序。
4. 录制总检查清单：也可以当作学习者自查表。

建议先打开：
AUTOWARE规划控制层课程总索引.md

如果你是新手，请按第 1 集到第 8 集顺序学习。
如果你正在排查问题，请直接看端到端实战复盘中的 topic 链路和故障决策树。
```

### 7.2 README 中的版本说明

```text
版本说明：

本资料基于当前项目中整理的 Autoware 规划控制层文档制作。
不同 Autoware 版本、不同 launch 配置、不同车辆平台下，topic 名称、参数路径、RViz 图层名称可能不同。

学习原理时，可以直接参考本文档。
实操复现时，请以自己环境中的 ros2 topic list、参数文件、launch 文件和 RViz 配置为准。
```

### 7.3 观众自学建议

```text
学习建议：

不要一开始就背 topic 名称。
先记住链路位置：
Goal -> Route -> Path -> Trajectory -> Velocity -> Validation -> Control -> Gate -> Vehicle

不要一开始就调参数。
先找第一个异常输出：
上游正常吗？当前模块有输出吗？下游收到吗？RViz marker 有提示吗？

不要把所有 QP 都混在一起。
Elastic Band、MPT、MPC 都可能用 QP，但它们优化的对象不同：
Elastic Band 偏路径平滑；
MPT 优化给控制器的轨迹；
MPC 优化当前要执行的控制输入。
```

---

## 8. 课程资料发布检查清单

发布前建议逐项确认：

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| README | 打开本文件 | 观众知道先看什么 |
| 总索引 | 打开课程总索引 | 第 1 到第 8 集和端到端复盘都列出 |
| PPT | 打开 8 个新手友好详解版 PPT | 每页都有讲解备注和配图建议 |
| 端到端实战 | 打开端到端详解版 | 每阶段有 topic、RViz、参数、排查顺序 |
| concept | 检查 concept 目录 | 每集核心公式有原理素材支撑 |
| 视频简介 | 使用模板生成 | 每集标题、核心问题、配套资料清楚 |
| 章节时间戳 | 使用模板生成 | 长视频可跳转 |
| 文件命名 | 检查文件名 | 集数、主题、版本清晰 |
| 资料说明 | 放在视频简介或发布页 | 观众知道 Markdown 文件用途 |
| 版本提醒 | 放在 README 或简介 | 说明 Autoware topic/参数可能随版本变化 |

---

## 9. 后续补充专题规划

这套主线课程已经覆盖规划控制层核心链路。后续可以做补充专题，帮助观众从“听懂原理”进入“能调试、能读源码、能复盘问题”。

### 9.1 RViz 调试专题

**推荐标题**

```text
Autoware RViz 调试专题：规划控制图层到底怎么看
```

**适合观众**

- 已经看完第 1 集，但打开 RViz 仍然不知道看哪条线的人。
- 想区分 route、path、trajectory、virtual wall、predicted trajectory 的人。

**建议内容**

1. 地图、TF、车辆模型。
2. Mission route 和 route marker。
3. Behavior path、candidate、reference。
4. Drivable area 和 bound。
5. MPT / path optimizer output。
6. Velocity trajectory 和 virtual wall。
7. Planning validator marker。
8. Controller reference 和 predicted trajectory。
9. Vehicle status。

**输出资料**

- `AUTOWARE规划控制层RViz调试专题_新手友好详解版.md`

### 9.2 rosbag 复盘专题

**推荐标题**

```text
Autoware rosbag 复盘专题：离线定位 Route / Path / Trajectory / Control 的断点
```

**适合观众**

- 跑车或仿真后想离线分析问题的人。
- 想把一次故障变成可重复复盘案例的人。

**建议内容**

1. rosbag 应该录哪些 topic。
2. 如何从 `/planning/trajectory` 往上游查。
3. 如何对齐 route、path、velocity、control 的时间。
4. 如何截取故障片段。
5. 如何写复盘报告。

**输出资料**

- `AUTOWARE规划控制层rosbag复盘专题_新手友好详解版.md`

### 9.3 常见故障案例专题

**推荐标题**

```text
Autoware 规划控制常见故障案例：从现象到第一个异常 topic
```

**适合观众**

- 已经能跑起来，但经常不知道为什么不动、急刹、抖动的人。

**建议案例**

| 案例 | 对应主线课程 |
|---|---|
| 点击目标没有 route | 第 2 集 |
| 有 route 无 path | 第 2、3 集 |
| Pull Out 失败 | 第 3 集 |
| MPT infeasible | 第 4、8 集 |
| 急刹或速度跳变 | 第 5 集 |
| Pure Pursuit 转向不稳定 | 第 6 集 |
| MPC 转向抖动或外切 | 第 7 集 |
| follower cmd 有但车不动 | 端到端实战复盘 |

**输出资料**

- `AUTOWARE规划控制层常见故障案例专题_新手友好详解版.md`

### 9.4 源码导读专题

**推荐标题**

```text
Autoware 规划控制源码导读：从 topic 反查 launch、node、callback 和核心函数
```

**适合观众**

- 已经理解原理，准备读 Autoware 源码的人。

**建议内容**

1. 从 topic 找 node。
2. 从 node 找 package。
3. 从 launch 找参数。
4. 从 callback 找输入输出。
5. 从核心函数找算法实现。
6. 如何把源码和 PPT 中的公式对应起来。

**输出资料**

- `AUTOWARE规划控制层源码导读专题_新手友好详解版.md`

### 9.5 全课程速查手册

**推荐标题**

```text
Autoware 规划控制层公式、topic、RViz、参数速查手册
```

**适合观众**

- 已经看完课程，想快速复习或调试的人。

**建议内容**

1. 每集核心公式速查。
2. 每层输入输出 topic 速查。
3. RViz 图层速查。
4. 关键参数速查。
5. 常见故障排查速查。

**输出资料**

- `AUTOWARE规划控制层公式Topic参数速查手册.md`

---

## 10. 发布后的维护建议

### 10.1 建议维护一个修订记录

```text
版本：v1.0
日期：YYYY-MM-DD
内容：发布第 1 到第 8 集 PPT、端到端实战复盘、录制检查清单。

版本：v1.1
日期：YYYY-MM-DD
内容：修正某集公式符号、补充某个 topic 名称差异说明。
```

### 10.2 建议维护一个观众 FAQ

可以收集这些问题：

- 为什么我的 topic 名称和视频里不一样？
- 为什么有 route 但没有 path？
- 为什么 `/planning/trajectory` 没有输出？
- 为什么 follower cmd 有但车不动？
- MPT 和 MPC 都是 QP，区别到底是什么？
- Velocity Smoother 和 Motion Velocity Planner 有什么区别？
- Pure Pursuit 和 MPC 应该先学哪个？

### 10.3 建议维护一个版本差异说明

Autoware 版本变化时，最容易变的是：

- topic 名称。
- launch 文件结构。
- 参数路径。
- RViz 图层名称。
- debug marker 名称。
- 模块开关。

建议在 README 或单独文件中写：

```text
本文档讲的是原理和排查方法。
如果你的版本 topic 不同，请先用 ros2 topic list 搜关键词：
route、path、trajectory、velocity、control、marker、debug。
```

---

## 11. 下一次给 Codex 的指令

完成课程发布包 README 后，建议下一步生成一份“全课程公式、topic、RViz、参数速查手册”。这份手册会非常适合观众看完课程后复习，也适合你录课时放在旁边快速查。

你可以复制下面这段给 Codex：

```text
请执行优化任务 13：
在不覆盖原文件的前提下，
根据 AUTOWARE规划控制层课程总索引.md、
AUTOWARE规划控制层课程发布包README.md、
AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md、
当前根目录第1集到第8集的新手友好详解版 PPT、
以及 concept 目录下的规划控制文档，
生成 AUTOWARE规划控制层公式Topic参数速查手册.md。

要求：
1. 面向观众复习和实操调试；
2. 按 Route、Path、Pull Out、Path Smoothing、MPT、Velocity、Pure Pursuit、MPC、End-to-End Control Gate 分类；
3. 每类包含：核心概念、核心公式、符号解释、物理意义、工程作用、相关 topic、RViz 图层、关键参数、常见故障和优先排查顺序；
4. 保持新手友好，不精简关键内容；
5. 输出到项目根目录，不覆盖原文件；
6. 任务完成后，给出下一次任务的指令。
```
