# Autoware 规划控制层课程总索引

本文档用于把当前项目根目录已经生成的第 1 集到第 8 集 PPT、端到端实战复盘文档、以及 `concept/` 目录下的规划控制素材串成一套可录制、可复习、可继续优化的课程索引。

使用建议：

- 录课时先看本文档，确认本集要回答的核心问题。
- 备课时再打开对应 PPT 和 concept 素材。
- 公式不要一次性全讲完，优先讲“这个公式在链路里解决什么问题”。
- 当前 PPT 普遍偏完整版，适合作为讲义和备课材料；正式录制时，部分集建议再生成录制精简版。

---

## 1. 当前课程主线

整套课的主线是一句话：

```text
目标点
-> Lanelet 车道级 Route
-> 局部 Path
-> 平滑 / 优化后的 Trajectory
-> 速度平滑
-> 横向控制
-> 当前控制命令
```

当前已经形成的课程结构：

| 集数 | 课程文件 | 主题定位 | 当前建议 |
|---|---|---|---|
| 第 1 集 | `AUTOWARE规划控制层第1集_规划控制总览PPT_优化版.md` | 建立全局链路地图 | 建议作为正式录制版基础 |
| 第 2 集 | `AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT.md` | Lanelet 与 Mission Planner | 保留完整版，建议再生成录制精简版 |
| 第 3 集 | `AUTOWARE规划控制层第3集_HybridAStarPullOutPPT.md` | Hybrid A* Pull Out | 保留完整版，建议再生成录制精简版 |
| 第 4 集 | `AUTOWARE规划控制层第4集_ElasticBand路径平滑PPT.md` | Elastic Band 路径平滑 | 保留完整版，建议再生成录制精简版 |
| 第 5 集 | `AUTOWARE规划控制层第5集_VelocitySmoother速度平滑PPT.md` | Velocity Smoother | 保留完整版，建议再生成录制精简版 |
| 第 6 集 | `AUTOWARE规划控制层第6集_PurePursuit横向控制PPT.md` | Pure Pursuit 横向控制 | 可直接录，也可轻度精简 |
| 第 7 集 | `AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT.md` | 横向 MPC | 保留完整版，强烈建议再生成录制精简版 |
| 第 8 集 | `AUTOWARE规划控制层第8集_MPT与整套链路复盘PPT.md` | MPT 与链路复盘 | 保留完整版，建议再生成录制精简版 |
| 实战复盘 | `AUTOWARE规划控制层端到端实战复盘_视频脚本与PPT大纲.md` | 从目标点到控制命令实操 | 建议保留长视频版，也可拆成短实战 |

---

## 2. 分集索引

### 第 1 集：Autoware 规划控制总览

| 项目 | 内容 |
|---|---|
| 主题 | 从目标点到方向盘控制命令，建立 Route、Path、Trajectory、Control Command 的层级概念 |
| 核心问题 | 为什么用户点击一个目标点后，车辆不能直接生成控制命令，而要经过多层规划和控制模块？ |
| 核心公式 | 本集不以公式为主，核心是数据抽象：`Route -> Path -> Trajectory -> Control Command`；可辅助讲 `TrajectoryPoint = pose + velocity + acceleration + time` 的概念 |
| 对应 concept 素材 | `concept/mission_planner_route_planning_blog.md`、`concept/lanelet.md`、`concept/start_planner_module_algorithm_blog.md`、`concept/autoware_elastic_band_smoothing_blog.md`、`concept/velocity_smoother_blog.md`、`concept/pure_pursuit_blog.md`、`concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md`、`concept/autoware_mpt_blog.md` |
| 建议录制时长 | 35 到 45 分钟 |
| 适合新手重点 | 先分清“路线、路径、轨迹、控制命令”四个词；再理解每个模块只是把上游结果翻译成更接近车辆执行的形式 |
| 容易讲难的地方 | 不要一开始进入源码和公式；这一集重点是建地图，不是证明算法 |
| 录制版本建议 | 使用优化版 PPT 录制；原版 PPT 保留为历史版本 |

建议讲法：

```text
先用一个目标点案例开场，再按数据越来越具体的顺序展开。
不要把所有算法细节塞进总览，只需要告诉观众后面每一集会填哪块拼图。
```

---

### 第 2 集：Lanelet 与 Mission Planner

| 项目 | 内容 |
|---|---|
| 主题 | 从 Lanelet 地图、车道拓扑、交通规则，到 Mission Planner 输出车道级 Route |
| 核心问题 | Autoware 如何把起点和终点映射到车道图上，并搜索出一条可行的车道级路线？ |
| 核心公式 | 车道图：$G=(V,E)$；起点朝向过滤：$|\operatorname{norm}(\psi_\ell(p_s)-\psi_s)| \le \pi/2$；最短路：$P^*=\arg\min_{P:\ell_s\leadsto \ell_g}\sum_{e\in P}w(e)$ |
| 对应 concept 素材 | `concept/lanelet.md`、`concept/mission_planner_route_planning_blog.md`、`concept/osm文件讲解.md`、`concept/车道的方向性.md`、`concept/height.md` |
| 建议录制时长 | 完整版 60 到 75 分钟；精简录制版 40 到 50 分钟 |
| 适合新手重点 | Lanelet 不是图片，而是带方向、边界、中心线、相邻关系和交通规则的车道对象；Mission Planner 输出的是 route sections，不是最终轨迹 |
| 容易讲难的地方 | OSM node、Lanelet 图节点、几何中心线、route section 很容易混在一起；建议每个概念都配一张小图 |
| 录制版本建议 | 保留完整版作为地图与路线讲义；正式录制建议生成精简版，把 OSM 细节压缩，把 Route Section 和 Pose-to-Lanelet 讲透 |

建议精简重点：

```text
Lanelet 结构 -> 拓扑图 -> 起终点匹配 -> 最短路 -> Route Section。
```

---

### 第 3 集：Hybrid A* Pull Out

| 项目 | 内容 |
|---|---|
| 主题 | 车辆不在车道中心时，如何通过 Start Planner / Pull Out 从自由空间回到车道 |
| 核心问题 | 当车辆停在路边、车位或自由空间中，普通车道跟随为什么不够，Hybrid A* 如何搜索可执行的驶出路径？ |
| 核心公式 | A* 评分：$f(n)=g(n)+h(n)$；自行车模型运动原语：$\dot{x}=v\cos\psi,\ \dot{y}=v\sin\psi,\ \dot{\psi}=v\tan\delta/L$；代价由路径长度、倒车、换挡、转角变化、障碍物距离等组成 |
| 对应 concept 素材 | `concept/hybrid_astar_freespace_pull_out_algorithm_blog.md`、`concept/start_planner_module_algorithm_blog.md`、`concept/shift_pull_out_algorithm_blog.md`、`concept/车辆碰撞检测建模.md`、`concept/KinematicsBicycleModel.md` |
| 建议录制时长 | 完整版 70 到 90 分钟；精简录制版 45 到 55 分钟 |
| 适合新手重点 | Hybrid A* 仍然是 A*，只是状态从二维格子变成了 `(x, y, yaw)`，动作从上下左右变成了车辆能执行的运动原语 |
| 容易讲难的地方 | 很容易把 Hybrid A* 讲成“神秘搜索算法”；要反复提醒：难点其实在状态、动作、碰撞检查和代价，而不是 `f=g+h` 本身 |
| 录制版本建议 | 保留完整版作为算法讲义；正式录制建议生成精简版，减少 Pull Out 方法家族的横向展开，主讲 Hybrid A* 主链路 |

建议精简重点：

```text
为什么需要 Pull Out -> costmap -> 状态离散 -> 运动原语 -> 碰撞检查 -> g/h 代价 -> 回接车道。
```

---

### 第 4 集：Elastic Band 路径平滑

| 项目 | 内容 |
|---|---|
| 主题 | 从离散路径点到横向偏移变量，再到 QP 路径平滑 |
| 核心问题 | 为什么离散点列不能直接给控制器，Autoware 如何用横向偏移和二阶差分把路径变平滑？ |
| 核心公式 | 平滑项：$\sum_{k=1}^{N-2}\|\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}\|^2$；横向偏移：$\mathbf{p}_i=\mathbf{r}_i+l_i\mathbf{n}_i$；QP：$\min_{\mathbf{l}}\frac12\mathbf{l}^TP\mathbf{l}+\mathbf{q}^T\mathbf{l}\ \text{s.t.}\ -c_i\le l_i\le c_i$ |
| 对应 concept 素材 | `concept/autoware_elastic_band_smoothing_blog.md`、`concept/QP求解.md`、`concept/中心线坐标系 和 后轮中心坐标系.md` |
| 建议录制时长 | 完整版 60 到 75 分钟；精简录制版 40 到 50 分钟 |
| 适合新手重点 | 先用“折线变柔和”建立直觉，再讲二阶差分；先让观众接受“只优化横向偏移”这个简化，再进入 QP |
| 容易讲难的地方 | 二阶差分矩阵和 QP 矩阵容易吓退新手；建议先画三个点，再画一排点，最后才写矩阵 |
| 录制版本建议 | 保留完整版作为 QP 入门材料；正式录制建议生成精简版，把矩阵推导控制在少量关键页 |

建议精简重点：

```text
离散点锯齿 -> 二阶差分衡量弯折 -> 横向偏移建模 -> 平滑项 + 贴近原路径项 -> box 约束 -> QP。
```

---

### 第 5 集：Velocity Smoother

| 项目 | 内容 |
|---|---|
| 主题 | 速度曲线为什么不能硬裁剪，如何同时满足速度、加速度、jerk、曲率和停车约束 |
| 核心问题 | 为什么看到限速或停车点时，不能把后方轨迹速度直接改成目标速度？ |
| 核心公式 | Jerk：$j=da/dt$；曲率限速：$v\le\sqrt{a_{\mathrm{lat,max}}/|\kappa|}$；常 jerk 积分：$v(t)=v_0+a_0t+\frac12jt^2$；空间变量：$b=v^2$ |
| 对应 concept 素材 | `concept/velocity_smoother_blog.md`、`concept/从速度上限到 Jerk 解析减速：Autoware Velocity Smoother 的完整数学链条.md`、`concept/jerk.md`、`concept/减速区间判断逻辑.md`、`concept/行为速度规划模块.md` |
| 建议录制时长 | 完整版 60 到 75 分钟；精简录制版 40 到 50 分钟 |
| 适合新手重点 | 把速度、加速度、jerk 解释成“走多快、速度变化多快、加速度变化多快”；用停车距离例子说明硬裁剪会导致不舒适甚至刹不住 |
| 容易讲难的地方 | 解析 jerk 三类减速曲线公式很多，正式录制不要逐行推完；先讲物理形状，再讲必要公式 |
| 录制版本建议 | 保留完整版；正式录制建议生成精简版，把重点放在“硬裁剪为什么错”和“前向/后向滤波如何保证可达” |

建议精简重点：

```text
硬裁剪的问题 -> v/a/j 三个量 -> 曲率限速 -> 前向 jerk filter -> 后向 decel filter -> 三类减速曲线。
```

---

### 第 6 集：Pure Pursuit

| 项目 | 内容 |
|---|---|
| 主题 | 用几何直觉理解横向控制，从前瞻目标点到前轮转角 |
| 核心问题 | Pure Pursuit 为什么不是追最近点，而是追前方一个目标点？ |
| 核心公式 | 曲率：$\kappa=\frac{2y}{x^2+y^2}$；自行车模型：$\kappa=\frac{\tan\delta}{L}$；转角：$\delta=\arctan(L\kappa)$ |
| 对应 concept 素材 | `concept/pure_pursuit_blog.md`、`concept/pure pursuit 曲率计算.md`、`concept/KinematicsBicycleModel.md`、`concept/阿克曼转向模型.md` |
| 建议录制时长 | 40 到 50 分钟 |
| 适合新手重点 | 让观众先接受“车沿圆弧追前方点”；再推导目标点横向偏移 `y` 如何决定曲率 |
| 容易讲难的地方 | 前瞻距离、目标点搜索、线段与圆求交容易变成实现细节；正式视频里要让“圆弧几何”始终是主线 |
| 录制版本建议 | 当前 PPT 已比较适合录制；可轻度精简动态前瞻距离和失败回退部分 |

建议讲法：

```text
追最近点为什么抖 -> 追前方点 -> 圆弧过目标点 -> 曲率公式 -> 自行车模型转角 -> 工程里的前瞻点搜索。
```

---

### 第 7 集：横向 MPC

| 项目 | 内容 |
|---|---|
| 主题 | 从阿克曼几何、自行车模型、Frenet 误差，到线性化、离散化和 QP 求解器输入 |
| 核心问题 | MPC 如何把“未来一段时间尽量贴着轨迹走，并且转向不要太猛”写成一个可求解的优化问题？ |
| 核心公式 | 几何关系：$\delta=\arctan(L\kappa)$；单步预测：$x_{k+1}=A_dx_k+B_du_k+W_d$；QP：$\min_U\frac12U^THU+g^TU$，约束：$lb\le U\le ub,\ lbA\le DU\le ubA$ |
| 对应 concept 素材 | `concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md`、`concept/KinematicsBicycleModel.md`、`concept/阿克曼转向模型.md`、`concept/QP求解.md` |
| 建议录制时长 | 完整版 80 到 100 分钟；精简录制版 50 到 60 分钟；也可以拆成上下两集 |
| 适合新手重点 | 把 MPC 拆成四步：车辆几何、误差状态、未来预测、优化求解；每一步只回答一个问题 |
| 容易讲难的地方 | 线性化、离散化、矩阵堆叠和 QP 推导密度很高；建议不要一口气讲完所有矩阵，要用“单步到多步”的节奏 |
| 录制版本建议 | 强烈建议保留完整版作为讲义，再生成录制精简版；如果追求质量，最好拆成“模型篇”和“QP篇” |

建议精简重点：

```text
阿克曼几何 -> Frenet 误差 -> 连续误差模型 -> 线性化/离散化 -> 预测时域 -> 目标函数和约束 -> 只执行第一个控制量。
```

---

### 第 8 集：MPT 与整套链路复盘

| 项目 | 内容 |
|---|---|
| 主题 | MPT 如何连接规划和控制，并把整套 Route、Path、Trajectory、Control 链路复盘一遍 |
| 核心问题 | 为什么路径平滑之后还需要 MPT，MPT 和 Elastic Band、MPC 分别有什么区别？ |
| 核心公式 | 空间误差模型：$dl/ds=\theta,\ d\theta/ds=\tan\delta/L-\kappa_{ref}$；单步仿射模型：$x_{i+1}=A_ix_i+B_iu_i+w_i$；QP 变量：$v=[X,U,S]^T$；标准形式：$\min_v\frac12v^THv+f^Tv,\ l_b\le Av\le u_b$ |
| 对应 concept 素材 | `concept/autoware_mpt_blog.md`、`concept/autoware_elastic_band_smoothing_blog.md`、`concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md`、`concept/车辆碰撞检测建模.md`、`concept/QP求解.md` |
| 建议录制时长 | 完整版 75 到 95 分钟；精简录制版 45 到 60 分钟 |
| 适合新手重点 | MPT 是局部轨迹优化，不是全局路线搜索，也不是控制器；它在道路边界、车身宽度、软约束和轨迹可控性之间做平衡 |
| 容易讲难的地方 | MPT 和 MPC 都有模型、预测、QP，容易混淆；建议用一句话固定边界：MPT 优化要给控制器的轨迹，MPC 优化当前要执行的控制输入 |
| 录制版本建议 | 保留完整版作为综合复盘讲义；正式录制建议生成精简版，把 MPT 深推导和整套链路复盘分清层次 |

建议精简重点：

```text
MPT 的位置 -> 输入输出 -> 局部坐标 -> 空间域模型 -> 道路边界和车辆圆 -> 软约束 -> QP -> 与 EB/MPC 的区别 -> 全链路复盘。
```

---

### 端到端实战复盘：从目标点到控制命令

| 项目 | 内容 |
|---|---|
| 主题 | 以一个从目标点到控制命令的完整案例，把理论模块和 Autoware topic / RViz / 参数观察串起来 |
| 核心问题 | 真实运行时，如何判断数据卡在哪一层：Route、Path、Trajectory、Velocity、Validation、Control 还是 Vehicle Gate？ |
| 核心公式 | 不新增公式，复用全链路抽象：`Route -> Path -> Trajectory -> Control Command`；重点是 topic 链路和排查顺序 |
| 对应 concept 素材 | 全部规划控制素材，尤其 `concept/data_link.md`、`concept/场景模块.md`、`concept/行为速度规划模块.md`、`concept/mission_planner_route_planning_blog.md`、`concept/velocity_smoother_blog.md`、`concept/pure_pursuit_blog.md`、`concept/autoware_mpt_blog.md` |
| 建议录制时长 | 长版 80 到 90 分钟；短版可拆成 3 个 25 分钟实战 |
| 适合新手重点 | 不要求新手一次看懂所有算法，而是学会“沿 topic 链路定位问题” |
| 容易讲难的地方 | topic、RViz 图层和参数很多，容易变成报菜单；必须用一个案例主线贯穿 |
| 录制版本建议 | 建议保留长视频版作为课程收尾；后续可以拆成“Route 排查”“Trajectory 排查”“Control 排查”三个短实战 |

建议主线：

```text
设置初始位姿 -> 点击目标点 -> 观察 route -> 观察 behavior path -> 观察优化轨迹 -> 观察速度平滑 -> 观察最终 trajectory -> 观察 control_cmd。
```

---

## 3. 推荐学习顺序

### 主线学习顺序

推荐按发布顺序学习：

1. 第 1 集：先建立全链路地图。
2. 第 2 集：理解地图和 route 从哪里来。
3. 第 3 集：理解车辆不在车道中心时如何回到车道。
4. 第 4 集：理解路径为什么要平滑，为什么会出现 QP。
5. 第 5 集：理解速度为什么需要平滑，为什么 jerk 很重要。
6. 第 6 集：理解几何横向控制，建立“轨迹如何变成转角”的直觉。
7. 第 7 集：理解优化控制，建立 MPC 的模型、预测、QP 链条。
8. 第 8 集：理解 MPT 和整套规划控制链路的关系。
9. 端到端实战复盘：把所有概念落到 topic、RViz 和排查流程。

### 如果只想先看控制

```text
第 1 集 -> 第 6 集 -> 第 7 集 -> 第 5 集 -> 第 8 集 -> 端到端实战
```

原因：

- 第 6 集最容易建立“控制器到底在干什么”的直觉。
- 第 7 集再升级到 MPC。
- 第 5 集补上速度曲线对控制体验的影响。
- 第 8 集解释轨迹优化如何给控制器更好的输入。

### 如果只想先看规划

```text
第 1 集 -> 第 2 集 -> 第 3 集 -> 第 4 集 -> 第 8 集 -> 端到端实战
```

原因：

- 第 2 集负责车道级路线。
- 第 3 集负责自由空间回接车道。
- 第 4 集负责路径几何质量。
- 第 8 集把局部轨迹优化和后续控制接起来。

---

## 4. 推荐录制顺序

发布顺序建议保持第 1 集到第 8 集不变，但实际录制时可以按“最容易录稳、最能建立信心”的顺序制作。

推荐录制顺序：

1. 第 1 集：规划控制总览  
   先录总览，后面每一集都可以引用这张总图。

2. 第 2 集：Lanelet 与 Mission Planner  
   它是规划链路入口，适合紧跟总览录。

3. 第 6 集：Pure Pursuit  
   几何直觉强，公式少，容易录出清晰效果，也能增强你讲控制的信心。

4. 第 5 集：Velocity Smoother  
   速度、加速度、jerk 很适合用曲线图讲，录制观感会比较好。

5. 第 4 集：Elastic Band  
   在观众已经理解路径和速度后，再讲 QP 路径平滑更自然。

6. 第 3 集：Hybrid A* Pull Out  
   图示和流程较多，建议在路线、路径概念稳定后录。

7. 第 7 集：横向 MPC  
   数学密度最高，建议等前面几集录完后再录。

8. 第 8 集：MPT 与整套链路复盘  
   必须放到接近最后录，因为它会引用前面所有内容。

9. 端到端实战复盘  
   最后录，把理论课变成可观察、可排查的完整案例。

---

## 5. 完整版与精简版建议

### 建议保留完整版的材料

以下文件建议都保留，不要删减原文：

| 文件 | 保留原因 |
|---|---|
| 第 1 集优化版 PPT | 已统一格式，可作为整套课入口 |
| 第 2 集 PPT | Lanelet 和 Mission Planner 概念密集，适合作为地图路线参考讲义 |
| 第 3 集 PPT | Pull Out / Hybrid A* 细节多，完整版适合复习算法边界 |
| 第 4 集 PPT | 二阶差分和 QP 推导完整，适合作为优化类算法入门资料 |
| 第 5 集 PPT | Velocity Smoother 内容很深，完整版适合作为速度规划手册 |
| 第 6 集 PPT | Pure Pursuit 推导完整，适合直接录制或轻度精简 |
| 第 7 集 PPT | MPC 推导链条完整，必须保留为讲义 |
| 第 8 集 PPT | MPT 与链路复盘内容完整，适合作为课程收束材料 |
| 端到端实战复盘文档 | topic、RViz、参数和排查顺序很适合长期查阅 |

### 建议再生成录制精简版的集数

| 集数 | 精简优先级 | 建议精简目标 |
|---|---:|---|
| 第 2 集 | 高 | 压缩 OSM 和细节状态机，突出 Lanelet 图、起终点匹配、Route Section |
| 第 3 集 | 高 | 压缩 Pull Out 方法家族，突出 Hybrid A* 状态、动作、碰撞、代价 |
| 第 4 集 | 中高 | 压缩矩阵推导页，突出二阶差分到 QP 的主线 |
| 第 5 集 | 中高 | 压缩三类 jerk 减速公式细节，突出硬裁剪问题和前后向滤波 |
| 第 7 集 | 最高 | 建议拆成上下两集，或生成 60 分钟以内精简版 |
| 第 8 集 | 高 | 建议把 MPT 深推导和整套链路复盘拆清楚，避免一集太满 |
| 端到端实战 | 中 | 保留长版，同时拆成 3 个短实战更利于传播 |

第 1 集和第 6 集可以先按当前版本录制。第 1 集作为入口不宜过短，第 6 集几何直觉强，完整版不会明显增加学习负担。

---

## 6. 每集录制时的统一模板

每一集建议都按下面结构录：

```text
1. 本集解决什么问题
2. 它在总链路中的位置
3. 一个直觉场景
4. 最小数学模型
5. 工程流程
6. 关键参数
7. 常见误区
8. 本集总结和下一集衔接
```

公式页统一讲法：

```text
先讲物理意义
再讲符号含义
最后讲它在代码或工程里会变成什么变量
```

不要直接念公式。比如讲 Pure Pursuit：

```text
\kappa = 2y / (x^2 + y^2)
```

要先说：

```text
目标点越偏左，车辆越应该左转；
目标点越远，同样横向偏差对应的弯就越缓。
```

---

## 7. 后续优化任务建议

为了把课程继续打磨好，建议接下来按下面任务推进。

### 优化任务 3：生成第 2 集录制精简版

```text
请执行优化任务 3：
在不覆盖原文件的前提下，
根据 AUTOWARE规划控制层课程总索引.md、
AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT.md、
concept/lanelet.md 和 concept/mission_planner_route_planning_blog.md，
生成 AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT_录制精简版.md。

要求：
1. 保留课程主线，不删除关键概念；
2. 面向新手，控制在 35 到 45 页；
3. 每页仍包含页面展示内容、讲解备注、配图建议；
4. 重点突出 Lanelet 图、Pose-to-Lanelet、最短路、Route Section；
5. 减少 OSM 低层细节和过多状态机展开。
```

### 优化任务 4：生成第 7 集拆分建议

```text
请执行优化任务 4：
根据 AUTOWARE规划控制层课程总索引.md 和
AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT.md，
设计第 7 集横向 MPC 的拆分方案。

要求：
1. 给出拆成上下两集的推荐标题；
2. 分别列出每集保留哪些页、压缩哪些页；
3. 给出每集核心公式和讲解主线；
4. 输出 Markdown 文件到项目根目录；
5. 不修改原 PPT。
```

### 优化任务 5：生成端到端实战短视频三段版

```text
请执行优化任务 5：
根据 AUTOWARE规划控制层端到端实战复盘_视频脚本与PPT大纲.md，
生成端到端实战复盘的三段短视频方案。

要求：
1. 第 1 段讲 Route 和 Path；
2. 第 2 段讲 Trajectory、Velocity Smoother、Planning Validator；
3. 第 3 段讲 Controller、Vehicle Cmd Gate 和故障排查；
4. 每段包含 topic、RViz 图层、演示步骤和口播提纲；
5. 输出 Markdown 文件到项目根目录。
```

---

## 8. 最重要的讲课原则

这套课要降低新手门槛，最重要的是不要让观众被模块名和公式淹没。

每一集都反复回答三个问题：

```text
这个模块接收什么？
这个模块输出什么？
它为什么不能省掉？
```

只要这三个问题讲清楚，观众即使暂时记不住全部公式，也能建立正确的源码阅读路线。

