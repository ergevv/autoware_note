# Autoware 规划控制层课程录制总检查清单

> 用途：正式录制前逐项检查课程材料、PPT、公式、RViz 演示、topic、参数、故障案例和发布准备。  
> 适用范围：第 1 集到第 8 集 + 端到端实战复盘。  
> 检查原则：不追求把内容讲短，而是确认每个知识点都能被新手看懂、听懂、复现、排查。

---

## 0. 使用方式

这份清单建议分三次使用：

1. **录制前一天**：检查资料、PPT、公式、配图和演示环境。
2. **录制当天开机后**：检查 RViz、topic、参数、rosbag、麦克风和屏幕布局。
3. **录制完成后发布前**：检查视频结构、字幕、章节、文件名、简介和配套资料。

每一项都按下面格式检查：

| 字段 | 含义 |
|---|---|
| 检查项 | 要确认的事情 |
| 检查方法 | 具体怎么检查 |
| 通过标准 | 什么情况算可以录 |
| 常见问题 | 如果不通过，通常哪里出错 |

建议不要一次性从头到尾机械打勾。更好的做法是：

```text
先检查课程主线
再检查每集 PPT
再检查公式讲法
再检查实操环境
最后检查发布材料
```

---

## 1. 总通过标准

整套课程可以正式录制，至少需要满足下面这些条件：

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 课程主线清晰 | 打开课程总索引，从第 1 集顺读到端到端实战复盘 | 能用一句话讲清楚：目标点如何变成 route、path、trajectory、control command | 每集都很详细，但观众不知道为什么要学这一集 |
| 每集定位清晰 | 查看每集标题页和“本集在总链路中的位置”页 | 每集开头都能说明它在 Autoware 规划控制链路中的位置 | 单集讲得很深，但和上下游断开 |
| 新手门槛足够低 | 随机抽一页公式页，检查是否有通俗解释 | 每个核心概念第一次出现时都有解释，公式有符号、物理意义、工程作用 | 公式直接出现，观众还不知道变量是什么 |
| 每页可录 | 随机抽 10 页 PPT | 每页都有页面展示内容、讲解备注、配图建议 | 页面内容能看，但不知道录的时候怎么讲 |
| 实操链路可演示 | 按端到端详解版的最小闭环 topic 清单检查 | route、path、trajectory、control command 至少能被观察到 | 只讲理论，没有现场可验证的运行链路 |
| 故障排查可讲 | 打开端到端详解版的故障决策树 | 能按“现象 -> topic -> RViz -> 参数 -> 上游模块”讲出排查顺序 | 只列现象，不知道先查哪里 |
| 发布材料完整 | 检查视频标题、简介、章节、资料链接 | 观众看完视频能找到对应 PPT 和 concept 文档 | 视频录完了，但资料散在多个文件里 |

---

## 2. 素材检查

素材检查的目标是确认：你录课时不会临时找不到文档、图、公式、topic 或演示场景。

### 2.1 文件素材总表

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 课程总索引 | 打开 `AUTOWARE规划控制层课程总索引.md` | 第 1 集到第 8 集和端到端实战复盘都在索引中 | 后续生成了新文件，但索引没有更新 |
| 第 1 集 PPT | 打开 `AUTOWARE规划控制层第1集_规划控制总览PPT_优化版.md` | 总览、Route、Path、Trajectory、Control Command 都有讲解备注和配图建议 | 第 1 集格式和后续详解版不一致 |
| 第 2 集 PPT | 打开 `AUTOWARE规划控制层第2集_Lanelet与MissionPlannerPPT_新手友好详解版.md` | Lanelet、车道图、起终点匹配、最短路、Route Section 都覆盖 | 只讲 Mission Planner，不讲 Lanelet 地图基础 |
| 第 3 集 PPT | 打开 `AUTOWARE规划控制层第3集_HybridAStarPullOutPPT_新手友好详解版.md` | Start Planner、Shift Pull Out、Hybrid A*、代价、碰撞、安全检查都覆盖 | 只讲 A* 公式，没有讲车辆运动学和边界 |
| 第 4 集 PPT | 打开 `AUTOWARE规划控制层第4集_ElasticBand路径平滑PPT_新手友好详解版.md` | 离散点、横向偏移、二阶差分、QP、边界约束都有慢读 | 直接写矩阵，缺少三点直觉 |
| 第 5 集 PPT | 打开 `AUTOWARE规划控制层第5集_VelocitySmoother速度平滑PPT_新手友好详解版.md` | 速度上限、加速度、jerk、曲率限速、解析减速都覆盖 | 只讲限速，没有讲速度不能硬裁剪 |
| 第 6 集 PPT | 打开 `AUTOWARE规划控制层第6集_PurePursuit横向控制PPT_新手友好详解版.md` | 前瞻点、车辆坐标系、曲率、转角、动态前瞻距离都覆盖 | 只给公式，不讲为什么追前方点 |
| 第 7 集 PPT | 打开 `AUTOWARE规划控制层第7集_横向MPC从阿克曼到QP_PPT_新手友好详解版.md` | 阿克曼几何、自行车模型、误差模型、线性化、离散化、QP 都覆盖 | 矩阵推导太密，没有单步到多步的过渡 |
| 第 8 集 PPT | 打开 `AUTOWARE规划控制层第8集_MPT与整套链路复盘PPT_新手友好详解版.md` | MPT、Elastic Band、MPC 区别、局部坐标、QP、链路复盘都覆盖 | MPT 和 MPC 边界讲不清 |
| 端到端实战复盘 | 打开 `AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md` | 每个阶段都有输入、输出、topic、RViz、参数、正常/异常、排查顺序 | 只列 topic，没有讲每层输入输出 |

### 2.2 concept 素材检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| Lanelet 素材 | 检查 `concept/lanelet.md` | 能支撑第 2 集的地图、边界、中心线、拓扑讲解 | Lanelet 概念讲成普通图片或普通车道线 |
| Mission Planner 素材 | 检查 `concept/mission_planner_route_planning_blog.md` | 能解释 goal、route、route section、routing graph | route 和 trajectory 混淆 |
| Hybrid A* 素材 | 检查 `concept/hybrid_astar_freespace_pull_out_algorithm_blog.md` | 能解释状态、运动原语、启发函数、代价、碰撞检查 | 只剩 `f=g+h`，缺少工程细节 |
| Start Planner 素材 | 检查 `concept/start_planner_module_algorithm_blog.md` | 能解释 pull out 为什么属于起步规划 | 观众不知道这个模块为什么在主链路里出现 |
| Shift Pull Out 素材 | 检查 `concept/shift_pull_out_algorithm_blog.md` | 能解释 shift pull out 和 Hybrid A* 的边界 | 误以为所有起步都必须用 Hybrid A* |
| Elastic Band 素材 | 检查 `concept/autoware_elastic_band_smoothing_blog.md` | 能支撑第 4 集的横向偏移、平滑和 QP | 公式有了，缺少物理直觉 |
| QP 素材 | 检查 `concept/QP求解.md` | 能解释标准 QP、目标函数、约束、求解器输入 | 新手不知道 H、f、A、lb、ub 分别是什么 |
| Velocity Smoother 素材 | 检查 `concept/velocity_smoother_blog.md` | 能解释速度约束、smoother 类型、重采样、初始状态 | 只看到速度点，不理解加速度和 jerk |
| Jerk 素材 | 检查 `concept/jerk.md` | 能用生活直觉解释 jerk | jerk 被讲成纯数学符号 |
| Pure Pursuit 素材 | 检查 `concept/pure_pursuit_blog.md` 和 `concept/pure pursuit 曲率计算.md` | 能支撑几何推导和工程搜索 | 曲率公式讲完后没有连接到转角 |
| 自行车模型素材 | 检查 `concept/KinematicsBicycleModel.md` | 能支撑第 3、6、7、8 集的车辆模型 | 每集重复讲模型但符号不一致 |
| 阿克曼模型素材 | 检查 `concept/阿克曼转向模型.md` | 能解释前轮转角、轴距、曲率关系 | 阿克曼和自行车模型边界讲混 |
| 横向 MPC 素材 | 检查 `concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md` | 能支撑第 7 集完整数学链条 | 线性化、离散化、QP 堆叠跳步 |
| MPT 素材 | 检查 `concept/autoware_mpt_blog.md` | 能支撑第 8 集 MPT 与全链路复盘 | MPT 被讲成另一个控制器 |

### 2.3 图示素材检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 全链路图 | 准备一张 `Goal -> Route -> Path -> Trajectory -> Control` 图 | 第 1 集、第 8 集、端到端实战都能复用 | 每集画法不同，观众难以建立统一心智模型 |
| Route / Path / Trajectory 对比图 | 画三条不同层级的线 | 能直观看出 route 是车道级，path 是几何线，trajectory 带速度 | 三者用同一种颜色或同一种形态展示 |
| Lanelet 基础图 | 画左右边界、中心线、前后连接、相邻连接 | 能解释 Lanelet 不是图片，而是可查询的地图结构 | 只截地图截图，没有标注边界和拓扑 |
| Hybrid A* 搜索图 | 画栅格、航向、运动原语、open/closed 集合 | 能解释搜索状态不是普通二维点 | 只画网格，不画车辆朝向 |
| Elastic Band 图 | 画离散折线和被平滑后的曲线 | 能解释弹性带直觉和边界约束 | 只显示 QP 公式，缺少可视化 |
| Velocity 曲线图 | 画速度、加速度、jerk 三条曲线 | 能说明速度硬裁剪为什么不舒服 | 只画速度，不画加速度变化 |
| Pure Pursuit 圆弧图 | 画车辆坐标系、前瞻点、圆弧、曲率 | 能解释 `kappa=2y/L_d^2` | 缺少车辆坐标系，公式无从理解 |
| MPC 预测图 | 画当前状态、参考轨迹、预测轨迹、控制序列 | 能解释“预测未来，只执行第一个输入” | 把 MPC 画成一次性规划整条路 |
| MPT / MPC 对比图 | 一边画轨迹优化，一边画控制优化 | 能解释 MPT 优化轨迹，MPC 优化控制输入 | 两个都写 QP，观众分不清 |
| 故障排查树图 | 使用端到端详解版 18A 的决策树 | 能从“车辆不动”一路查到 route/path/trajectory/control/gate | 只有故障列表，没有决策路径 |

### 2.4 演示素材检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 普通 lane driving 场景 | 准备一个起点和目标点都在可达车道上的场景 | route、path、trajectory、control 都能稳定出现 | 场景太复杂，第一遍演示就被红绿灯或障碍卡住 |
| Pull Out 场景 | 准备车辆偏离车道或路边起步的片段 | 能看到 start planner / pull out candidate | 起点太靠近障碍或边界，候选路径不稳定 |
| 停止点或慢行场景 | 准备停止线、virtual wall 或障碍触发片段 | 能看到 stop reason / virtual wall / velocity factor | 停止原因不清楚，观众以为是控制器刹车 |
| 转向跟踪场景 | 准备轻微弯道 | 能看到 predicted trajectory 和 reference 的关系 | 轨迹太直，控制器调试信息看不出意义 |
| 故障演示片段 | 至少准备 3 个：无 route、有轨迹无控制、有 follower cmd 但车不动 | 能按决策树排查 | 只讲理论故障，没有可视化例子 |
| rosbag 素材 | 录制关键 topic 的 bag | 回放时能复现 route/path/trajectory/control 的链路 | bag topic 不全，后期无法复盘 |

---

## 3. PPT 检查

PPT 检查的目标是确认：每一页都能被录制，不只是能被阅读。

### 3.1 全局格式检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| Marp 分页 | 搜索 `---` | 每页之间用 `---` 分隔 | 多页粘在一起，导出 PPT 不稳定 |
| 页面标题 | 搜索 `# ` | 每页有清晰标题，标题能回答“这一页讲什么” | 标题太抽象，如“公式推导 1” |
| 页面展示内容 | 随机抽页阅读标题下正文 | 屏幕上展示的内容不超过一页承载能力 | 一页塞太多公式和解释 |
| 讲解备注 | 搜索 `讲解备注：` | 每页都有口播方向 | 只有展示内容，没有录制话术 |
| 配图建议 | 搜索 `配图建议：` | 每页都有可执行配图建议 | 只写“配图”，没有说画什么 |
| 过渡页 | 搜索 `过渡` | 难点前后有过渡页 | 从概念突然跳公式 |
| 常见疑问页 | 搜索 `常见疑问` | 易混点有专门解释 | 误区埋在讲解备注里，观众看不到 |
| 模块卡片页 | 搜索 `模块卡片` | 重要模块都有输入、输出、职责 | 只讲算法，不讲系统接口 |
| 总结页 | 搜索 `总结` | 每集末尾有三句话或一张图总结 | 录完后观众不知道带走什么 |
| 下一集预告 | 搜索 `下一集预告` | 每集结尾能自然衔接下一集 | 课程之间像孤立文章 |

### 3.2 每集 PPT 主线检查

| 集数 | 检查方法 | 通过标准 |
|---|---|---|
| 第 1 集 | 检查是否反复出现 `Route -> Path -> Trajectory -> Control Command` | 新手看完知道四层数据抽象的区别 |
| 第 2 集 | 检查 Lanelet、routing graph、route section、goal validation 是否都有页面 | 新手知道 route 如何从地图拓扑搜索出来 |
| 第 3 集 | 检查 start planner、shift pull out、Hybrid A*、碰撞检查、安全检查是否都有页面 | 新手知道车辆不在车道中心时为什么需要 pull out |
| 第 4 集 | 检查离散点、局部坐标、横向偏移、二阶差分、QP 是否逐步出现 | 新手不会被 QP 矩阵突然吓住 |
| 第 5 集 | 检查速度上限、曲率限速、jerk、前向/后向滤波、解析减速是否都有慢读 | 新手知道速度曲线为什么不能硬裁剪 |
| 第 6 集 | 检查车辆坐标系、前瞻点、圆弧、曲率、转角、前瞻距离搜索是否完整 | 新手能用几何图理解 Pure Pursuit |
| 第 7 集 | 检查阿克曼、自行车模型、误差状态、线性化、离散化、堆叠、QP 是否层层递进 | 新手能跟上 MPC 从几何到 QP 的链条 |
| 第 8 集 | 检查 MPT 输入输出、Elastic Band/MPT/MPC 区别、空间域模型、全链路复盘是否清晰 | 新手知道轨迹优化如何连接规划和控制 |
| 端到端实战 | 检查每阶段输入输出、topic、RViz、参数、正常异常、排查顺序 | 新手能照着视频定位数据断在哪一层 |

### 3.3 新手友好检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 第一次出现的概念 | 搜索核心词，如 Lanelet、Jerk、QP、MPC、MPT | 第一次出现时有通俗解释 | 后面讲得很好，但开头没铺垫 |
| 类比页 | 搜索“直觉”“类比”“一句话” | 抽象公式前有直觉页 | 公式和代码先出现 |
| 公式慢读 | 搜索 `公式慢读` | 重要公式都有变量解释、物理意义、工程作用 | 公式只贴出来，不讲每项惩罚什么 |
| 误区页 | 搜索 `常见误区` | route/path/trajectory、MPT/MPC、速度规划/控制等易混点有纠正 | 观众学完仍然混淆边界 |
| 课程衔接 | 检查每集前 5 页和后 5 页 | 开头接上一集，结尾引下一集 | 单集很完整，但学习路线断裂 |

### 3.4 画面可读性检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 字数 | 逐页预览 | 一页只讲一个主要问题 | 一页同时讲概念、公式、参数、故障 |
| 公式密度 | 检查公式页 | 每页公式不超过观众能在 1 到 2 分钟内理解的量 | 多个矩阵堆在一页 |
| 表格宽度 | 预览导出效果 | 表格不溢出页面 | topic 表太宽 |
| 中英文混排 | 预览页面 | topic、参数、公式不破坏阅读 | 长 topic 没换行 |
| 颜色含义 | 检查图示 | route、path、trajectory、control 使用固定颜色 | 每集颜色不一致 |
| 配图可执行 | 阅读配图建议 | 看到建议就能画图或截 RViz | 配图建议太泛泛 |

---

## 4. 公式检查

公式检查的目标不是把每个推导都讲得像数学课，而是保证新手知道：

```text
这个公式里的量是什么
它描述了什么物理关系
它在工程里解决什么问题
它会影响 RViz 或 topic 中的什么现象
```

### 4.1 公式页通用检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 变量定义 | 看公式后是否逐项解释符号 | 每个新符号都有解释 | `x`、`u`、`H`、`A` 直接出现 |
| 物理意义 | 看讲解备注 | 能用一句话说明公式在描述什么 | 只说“代入可得” |
| 工程作用 | 看公式后是否连接模块 | 能说明公式影响 route、path、velocity 或 control 的哪一环 | 公式孤立存在 |
| 观察方法 | 看是否提到 RViz/topic/参数 | 能告诉观众在哪里观察公式的效果 | 数学和实操断开 |
| 推导节奏 | 看公式页前后 | 从直觉、小例子、单步公式，再到矩阵 | 直接进入大矩阵 |
| 符号一致性 | 对比相关集 | 轴距、曲率、航向角、横向误差符号前后一致 | 同一个量多种符号且未说明 |

### 4.2 分集核心公式检查

| 集数 | 必查公式 | 检查方法 | 通过标准 |
|---|---|---|---|
| 第 1 集 | `Route -> Path -> Trajectory -> Control Command`，`TrajectoryPoint = pose + velocity + acceleration + time` | 看总览页和核心概念页 | 新手知道四层数据不是一个东西 |
| 第 2 集 | `G=(V,E)` | 看车道图页 | 能解释 `V` 是 lanelet 节点，`E` 是连接关系 |
| 第 2 集 | 起点/目标朝向过滤 | 看公式慢读页 | 能解释为什么不能只按最近距离匹配车道 |
| 第 2 集 | 最短路 `argmin sum w(e)` | 看图搜索页 | 能解释 route 是在车道图上搜索 |
| 第 2 集 | Route Section | 看 route section 页 | 能解释主路线和相邻候选车道的关系 |
| 第 3 集 | A* `f(n)=g(n)+h(n)` | 看 A* 核心公式页 | 能解释实际代价和启发代价的不同 |
| 第 3 集 | 自行车模型运动原语 | 看运动原语页 | 能解释状态扩展受车辆转弯半径约束 |
| 第 3 集 | 倒车、换挡、转角变化、障碍距离代价 | 看代价组成页 | 能解释为什么路径不只看长度 |
| 第 3 集 | 目标窗口 | 看目标判定页 | 能解释搜索成功不是精确撞到一个点 |
| 第 4 集 | `p_i = r_i + l_i n_i` | 看横向偏移页 | 能解释为什么把二维点优化转成一维横向偏移 |
| 第 4 集 | 二阶差分 `p_{k+1}-2p_k+p_{k-1}` | 看三点和矩阵页 | 能解释它是在惩罚折角或曲率突变 |
| 第 4 集 | QP `min 1/2 l^T P l + q^T l` | 看标准 QP 页 | 能解释目标函数和边界约束 |
| 第 5 集 | `j=da/dt` | 看 jerk 页 | 能用乘坐体验解释 jerk |
| 第 5 集 | 曲率限速 `v <= sqrt(a_lat,max / |kappa|)` | 看曲率限速页 | 能解释弯道速度为什么不能太快 |
| 第 5 集 | 常 jerk 积分 | 看解析减速页 | 能解释速度、加速度、距离如何随时间变化 |
| 第 5 集 | `b=v^2` 空间变量 | 看空间变量页 | 能解释为什么速度规划常转到距离域 |
| 第 6 集 | 世界坐标到车辆坐标 | 看坐标变换页 | 能解释所有 Pure Pursuit 几何都在车辆坐标系下 |
| 第 6 集 | `kappa = 2y / (x^2+y^2)` | 看曲率页 | 能解释目标点越偏，曲率越大 |
| 第 6 集 | `delta = atan(L kappa)` | 看自行车模型页 | 能解释曲率如何变成前轮转角 |
| 第 6 集 | 动态前瞻距离 | 看参数直觉页 | 能解释看近更灵敏、看远更稳定 |
| 第 7 集 | `delta = atan(L kappa)` | 看阿克曼几何页 | 能连接第 6 集和第 7 集 |
| 第 7 集 | 单步状态预测 `x_{k+1}=A_d x_k + B_d u_k + W_d` | 看离散化页 | 能解释 MPC 用模型预测下一步误差 |
| 第 7 集 | 多步堆叠 | 看 `Y=d+MU` 页 | 能解释为什么最后只剩控制序列变量 |
| 第 7 集 | QP `min 1/2 U^T H U + g^T U` | 看标准 QP 页 | 能解释 H 和 g 来自误差代价和控制代价 |
| 第 7 集 | 输入约束与变化率约束 | 看约束页 | 能解释转向角和转向速度不能无限大 |
| 第 8 集 | 空间误差模型 `dl/ds=theta`，`dtheta/ds=tan(delta)/L-kappa_ref` | 看空间域模型页 | 能解释 MPT 沿路径距离推进状态 |
| 第 8 集 | 单步仿射模型 `x_{i+1}=A_i x_i+B_i u_i+w_i` | 看线性化页 | 能解释参考曲率进入模型 |
| 第 8 集 | QP 变量 `v=[X,U,S]^T` | 看决策变量页 | 能解释 MPT 为什么可能包含状态、输入和松弛变量 |
| 第 8 集 | 标准形式 `min 1/2 v^T H v + f^T v, lb <= A v <= ub` | 看最终 QP 页 | 能解释边界、安全距离、平滑目标如何统一 |
| 端到端 | `Route -> Path -> Trajectory -> Control Command` | 看全链路复盘 | 能解释故障排查为什么按 topic 链路上游找 |

### 4.3 公式讲解口播检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 开场口播 | 每个公式第一次出现前读一遍 | 先说“这个公式想解决什么问题” | 一上来读符号 |
| 变量口播 | 指着公式逐项解释 | 新手能知道每个符号对应车辆、轨迹、速度或控制中的什么量 | 变量解释藏在前几页 |
| 工程口播 | 公式后补一句工程作用 | 能连接到平滑、限速、跟踪、约束、排查 | 公式讲完就翻页 |
| 反例口播 | 至少给一个“如果没有这个约束会怎样” | 新手知道公式不是为了炫技 | 不知道为什么要这个公式 |
| RViz 口播 | 公式后给一个观察点 | 能说“这个公式的效果可以在某个图层或 topic 里看到” | 数学和实操割裂 |

---

## 5. RViz 演示检查

RViz 检查的目标是确认：录课时画面上能看到你正在讲的东西。

### 5.1 RViz 全局图层检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| Vector Map / Lanelet Map | 打开 RViz 地图图层 | 地图清晰显示，道路、车道边界可见 | fixed frame 错误或地图未加载 |
| TF | 打开 TF 图层 | `map`、`base_link` 等关键 frame 正常 | TF 断开导致车辆和地图错位 |
| Vehicle Model | 打开车辆模型 | 车辆在地图道路附近，朝向合理 | 初始位姿点错或坐标系错 |
| Localization Pose | 打开定位 pose | pose 和车辆模型一致 | 定位跳变或频率异常 |
| Goal Pose | 点击目标点 | 目标点可见，位置落在可达车道附近 | Goal 工具 topic 配错 |
| Route Marker | 点击目标后观察 | route marker 沿车道出现 | route 未生成或图层未开 |
| Behavior Path | route 后观察 | path 出现在车道内且稳定 | scenario 不对或 behavior planner 无输出 |
| Path Candidate / Reference | 打开候选和参考 | 能展示候选 path 与最终 path 的区别 | 图层太多颜色混乱 |
| Drivable Area | 打开可行驶区域 | 可行驶区域包住合理道路范围 | 边界太窄或地图边界异常 |
| Path Smoother / MPT | 打开优化轨迹 | 平滑前后或优化前后可对比 | topic 名称与图层配置不一致 |
| Velocity Trajectory | 打开速度着色轨迹 | 速度变化能通过颜色或 marker 看出 | 颜色图例不清 |
| Virtual Wall | 触发停车或慢行场景 | 停止墙出现在合理位置 | 不知道是哪个模块生成 |
| Planning Validator Marker | 打开 validator marker | 正常时无错误，异常时有提示 | 中间轨迹正常但 final trajectory 被拦截 |
| Controller Reference | 打开控制器参考轨迹 | 能看到控制器重采样参考线 | 控制器 debug 图层没开 |
| Predicted Trajectory | 打开预测轨迹 | 预测轨迹与参考轨迹大方向一致 | 观众不知道 predicted 和 final 区别 |
| Vehicle Status Overlay | 打开车辆状态 | 速度、转角变化可见 | 只看命令，不看实际状态 |

### 5.2 分阶段 RViz 检查

| 阶段 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 初始位姿 | 点击 `2D Pose Estimate` | 车辆出现在车道附近，朝向正确 | 车头朝反或偏离道路 |
| Mission Planner | 点击 `2D Goal Pose` | goal、route marker、route state 都正常 | route 没出现或绕远 |
| Behavior Path | 打开 behavior path 图层 | path 在 route 对应车道内 | 有 route 无 path |
| Pull Out | 使用起点偏离车道场景 | candidate、bound、vehicle footprint 可见 | 起点太极端导致搜索失败 |
| Elastic Band | 对比平滑前后 | 平滑后更顺但不越界 | 平滑后贴边或离原 path 过远 |
| MPT | 打开 optimizer 输出 | optimized trajectory 稳定 | 输出为空、每帧跳、贴边 |
| Motion Velocity Planner | 触发停止或慢行 | virtual wall 和 stop reason 合理 | 突然停车原因不明 |
| Velocity Smoother | 看速度颜色或曲线 | 速度连续，没有硬裁剪 | 急刹、速度忽高忽低 |
| Planning Validator | 对比 smoother 和 final trajectory | final trajectory 持续发布 | smoother 有，final 没有 |
| Controller | 打开 reference 和 predicted | predicted 能解释车辆执行趋势 | 控制器无 debug 输出 |
| Cmd Gate | 对比 follower cmd 和 gate cmd | engage 后 gate 后命令存在 | follower 有命令但 gate 拦住 |

### 5.3 录制画面检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 分辨率 | 录一段 10 秒测试视频 | 文字、topic、RViz 图层名称可读 | 分辨率太低，topic 看不清 |
| 鼠标高亮 | 试讲一页 | 能清楚指出 route/path/trajectory | 观众不知道你在指哪里 |
| 图层颜色 | 打开核心图层 | route/path/trajectory/control 颜色可区分 | 多条线颜色接近 |
| 画面节奏 | 录制每个关键瞬间 | route 出现、path 出现、trajectory 出现都有停顿 | 画面切太快 |
| 终端布局 | RViz + 终端同屏 | 终端命令不遮挡 RViz 关键区域 | 终端挡住地图和车辆 |
| 音画同步 | 录制测试 | 口播提到的对象正在画面中 | 先讲完了画面才出现 |

---

## 6. Topic 检查

topic 检查的目标是确认：每一层的输入输出都能被观众看到。

### 6.1 最小闭环 topic 检查

| 顺序 | topic | 检查方法 | 通过标准 | 异常时优先排查 |
|---|---|---|---|---|
| 1 | `/localization/kinematic_state` | `ros2 topic hz /localization/kinematic_state` | 持续发布，频率稳定 | 初始位姿、定位、TF |
| 2 | `/planning/mission_planning/goal` | 点击目标后 `ros2 topic echo` | 能收到目标点 | RViz Goal 工具 topic |
| 3 | `/planning/mission_planning/route` | 点击目标后 `ros2 topic echo` 或看 hz | 有 route 消息 | goal、定位、Lanelet 拓扑 |
| 4 | `/planning/mission_planning/route_marker` | RViz 或 topic list | marker 可见 | route 是否生成、图层是否打开 |
| 5 | `/planning/scenario_planning/lane_driving/behavior_planning/path` | route 后检查 hz | 有 path 输出 | scenario、behavior planner、RTC |
| 6 | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | 检查 hz | 有优化轨迹 | path_smoother 输入、边界、MPT |
| 7 | `/planning/scenario_planning/lane_driving/trajectory` | 检查 hz | 有 lane driving trajectory | Motion Velocity Planner |
| 8 | `/planning/scenario_planning/velocity_smoother/trajectory` | 检查 hz | 有速度平滑轨迹 | smoother 输入、当前速度、jerk 参数 |
| 9 | `/planning/trajectory` | `ros2 topic hz /planning/trajectory` | 最终轨迹持续发布 | Planning Validator |
| 10 | `/control/trajectory_follower/control_cmd` | `ros2 topic hz /control/trajectory_follower/control_cmd` | 控制器有输出 | final trajectory、odometry、steering |
| 11 | `/control/command/control_cmd` | `ros2 topic hz /control/command/control_cmd` | gate 后有命令 | engage、operation mode、emergency |
| 12 | `/vehicle/status/velocity_status` | `ros2 topic echo` | 车辆速度状态变化 | vehicle interface、底盘状态 |
| 13 | `/vehicle/status/steering_status` | `ros2 topic echo` | 实际转角状态变化 | steering interface、底盘状态 |

### 6.2 常用命令检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| planning topic 列表 | `ros2 topic list | grep planning` | 能看到 mission、scenario、trajectory 相关 topic | launch 未启动或命名空间不同 |
| control topic 列表 | `ros2 topic list | grep control` | 能看到 follower 和 command gate 相关 topic | 控制节点未启动 |
| route 状态 | `ros2 topic echo /planning/mission_planning/route_state` | 点击目标后状态合理 | route 未生成或状态机未更新 |
| final trajectory 频率 | `ros2 topic hz /planning/trajectory` | 频率稳定，不间断 | validator 拦截或上游轨迹间断 |
| follower cmd 频率 | `ros2 topic hz /control/trajectory_follower/control_cmd` | 控制器持续输出 | 缺少轨迹或车辆状态 |
| gate cmd 频率 | `ros2 topic hz /control/command/control_cmd` | engage 后有输出 | command gate 拦截 |
| vehicle velocity | `ros2 topic echo /vehicle/status/velocity_status` | 车辆运动后速度变化 | 底盘接口未执行 |
| vehicle steering | `ros2 topic echo /vehicle/status/steering_status` | 转弯时转角变化 | steering 状态未接入 |

### 6.3 rosbag 检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| bag topic 覆盖 | `ros2 bag info <bag>` | 包含 localization、planning、control、vehicle status | bag 只录了 RViz 可视化 marker |
| bag 时长 | 查看 duration | 能覆盖从点击目标到车辆运动的完整过程 | 开始或结尾缺失 |
| bag 可回放 | `ros2 bag play <bag>` 后看 topic list | 回放能复现主要 topic | 时间戳或 QoS 问题 |
| 故障 bag | 至少保存一个异常片段 | 能用于讲故障排查 | 只有正常片段，没有教学价值 |

---

## 7. 参数检查

参数检查的目标不是鼓励一上来调参，而是确认你录课时能解释“参数影响什么现象”。

### 7.1 参数讲解原则

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 不先调参 | 看故障排查口播 | 先查输入输出，再查参数 | 看到异常就改权重 |
| 参数有物理意义 | 查看参数页 | 每个参数都能连接到距离、速度、曲率、边界或转角 | 参数只是名称列表 |
| 参数有观察点 | 查看讲解备注 | 能说“调这个会影响 RViz 中哪条线或哪个 topic” | 参数和现象断开 |
| 参数不神化 | 查看口播 | 强调参数只能改善合理输入，不能修复错误输入 | 用调参掩盖上游错误 |

### 7.2 分模块关键参数检查

| 模块 | 参数类型 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|---|
| Mission Planner | goal topic、route state、goal validation | 查看第 2 集参数页和端到端 Mission Planner 段 | 能解释目标点、朝向、可达 lanelet 对 route 的影响 | 目标点不合法却怀疑控制器 |
| Behavior Path Planner | module enable、drivable area、RTC、安全检查 | 查看 behavior path 相关页 | 能解释 path 不输出时为什么先看 scenario 和模块状态 | 有 route 无 path 时不知道查模块状态 |
| Start Planner / Pull Out | shift pull out、freespace pull out、碰撞 margin、搜索分辨率 | 查看第 3 集参数页 | 能解释候选路径生成失败的主要原因 | 把所有失败归咎于 Hybrid A* |
| Elastic Band | 平滑权重、参考保持权重、边界约束 | 查看第 4 集 QP 和调参页 | 能解释路径变顺、贴边、偏离参考的原因 | 越平滑越好这个误区没有纠正 |
| MPT / Path Optimizer | `option.enable_skip_optimization`、`output_delta_arc_length`、steer input weight、soft constraint、warm start | 查看第 8 集和端到端 MPT 段 | 能解释无输出、贴边、每帧跳的排查方向 | 只会改权重，不看边界 |
| Motion Velocity Planner | stop distance、slow down、曲率限速、obstacle 参数 | 查看端到端 velocity planner 段 | 能解释 virtual wall 和 stop reason | 急刹时不知道停止点来源 |
| Velocity Smoother | max accel、max decel、max jerk、resample interval、engage 参数 | 查看第 5 集参数和端到端 smoother 段 | 能解释舒适减速、急刹、速度跳变 | 把上游停止点问题误认为 smoother 问题 |
| Planning Validator | trajectory checker、latency checker、collision checker | 查看端到端 validator 段 | 能解释 smoother 有输出但 final trajectory 没有的原因 | 忽略 validator |
| Pure Pursuit | lookahead distance、速度增益、曲率项、转角限幅 | 查看第 6 集调参页 | 能解释看近灵敏、看远稳定 | 前瞻距离只当固定常数讲 |
| MPC | prediction horizon、prediction dt、lat error weight、heading weight、steering input weight、steer rate weight、vehicle model | 查看第 7 集参数页 | 能解释跟踪慢、抖动、外切的参数方向 | 一上来调 Q/R，不看轨迹曲率和速度 |
| Vehicle Cmd Gate | engage、operation mode、emergency、input source | 查看端到端 gate 段 | 能解释 follower cmd 有但车辆不动 | 忽略安全门 |

### 7.3 参数演示检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 参数前后对比 | 如果要演示调参，准备同一场景前后对比 | 只改变一个关键参数，并说明影响 | 同时改很多参数，看不出原因 |
| 参数安全边界 | 讲解参数时提醒不要盲目放宽 | 观众知道参数不是越大越好 | 为了通过 demo 盲目放松约束 |
| 参数和故障关联 | 每个故障只列 2 到 3 个优先参数 | 排查顺序清晰 | 故障表变成参数菜单 |

---

## 8. 故障案例检查

故障案例检查的目标是确认：课程不仅能讲“正常链路”，还能教观众“坏了怎么查”。

### 8.1 必备故障案例

| 故障案例 | 检查方法 | 通过标准 | 优先排查顺序 |
|---|---|---|---|
| 点击目标没有 route | 准备一个目标 topic 配错或目标点不合法的例子 | 能演示 goal topic、localization、route_state、lanelet 可达性 | goal -> localization -> route_state -> lanelet |
| route 方向错 | 准备目标朝向或起点投影异常例子 | 能讲清起终点匹配和 lanelet 方向 | 起点朝向 -> 目标朝向 -> lanelet direction |
| 有 route 无 path | 准备 scenario 或 behavior path 不输出例子 | 能演示 route 和 path 是两层 | scenario -> behavior path topic -> module debug |
| Pull Out 失败 | 准备起点偏离、边界太窄或障碍挡路例子 | 能演示 candidate、bound、collision check | current pose -> drivable area -> objects -> search params |
| 平滑后仍锯齿 | 准备输入 path 抖动例子 | 能说明先看上游 path，再看平滑权重 | input path -> bound -> weights |
| MPT 无输出 | 准备边界过窄或输入为空例子 | 能演示 path_smoother/path 和 path_optimizer/trajectory 的断点 | input -> bound -> vehicle circles -> solver log |
| 突然停车 | 准备 virtual wall 或 stop reason 例子 | 能说明停车来自速度规划或规则触发 | stop reason -> virtual wall -> perception/map |
| 急刹 | 准备停止点太近或 jerk 受限例子 | 能区分上游停止点和 smoother 参数 | stop point -> current speed -> jerk/decel |
| smoother 有但 final 没有 | 准备 validator 拦截例子 | 能演示 Planning Validator 的作用 | smoother trajectory -> validator marker -> final trajectory |
| final 有但 follower 无输出 | 准备车辆状态缺失或轨迹起点偏差例子 | 能说明控制器需要轨迹和车辆状态 | final trajectory -> localization -> steering status |
| follower 有但车不动 | 准备 command gate 未 engage 例子 | 能说明安全门存在 | follower cmd -> gate cmd -> engage -> operation mode |
| 转向抖动 | 准备轨迹曲率抖或控制参数激进例子 | 能说明先看轨迹，再看控制参数 | final trajectory -> resampled reference -> predicted trajectory -> params |

### 8.2 故障讲解结构检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 先展示现象 | 每个故障先放 RViz 或终端现象 | 观众先知道“哪里坏了” | 直接解释原因，缺少问题感 |
| 再定位层级 | 用全链路图标出故障层 | 能说清它属于 Route、Path、Trajectory、Velocity、Control 还是 Gate | 排查散乱 |
| 再查 topic | 至少检查上游、当前、下游三个 topic | 能找到第一个异常输出 | 只看一个 topic 就下结论 |
| 再看 RViz marker | 打开对应 marker 或图层 | topic 数据和画面现象能对应 | 终端和 RViz 讲两套东西 |
| 最后看参数 | 只有输入输出正常时才讲参数 | 参数排查不会抢在链路排查前面 | 故障讲成调参教程 |
| 给出结论 | 每个故障最后总结一句 | 观众知道下次遇到同类现象先查哪里 | 讲完后没有可复用方法 |

### 8.3 故障决策树检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 总决策树 | 打开端到端详解版 `18A.1` | 从车辆执行端能往上游逐层查 | 只有分散表格 |
| 无 route 决策树 | 打开 `18A.2` | goal、localization、target lanelet、route_state 都覆盖 | 忘记检查 RViz goal topic |
| 无 path 决策树 | 打开 `18A.3` | scenario、behavior topic、RTC、module marker 都覆盖 | 只查 route |
| 无优化轨迹决策树 | 打开 `18A.4` | smoother input、optimizer input、solver、bound 都覆盖 | 直接怀疑求解器 |
| 速度异常决策树 | 打开 `18A.5` | virtual wall、stop reason、曲率限速、smoother 都覆盖 | 所有速度问题都归因于 jerk |
| 无 final trajectory 决策树 | 打开 `18A.6` | validator marker、topic、launch、输入轨迹都覆盖 | 忽略 Planning Validator |
| 无 control cmd 决策树 | 打开 `18A.7` | 控制器节点、车辆状态、轨迹时间戳都覆盖 | 只看 `/planning/trajectory` |
| 车不动决策树 | 打开 `18A.8` | follower cmd、gate cmd、engage、vehicle interface 都覆盖 | 忽略 command gate |
| 表现不好决策树 | 打开 `18A.9` | 抖动、外切、急刹、绕远都能分流 | 把所有表现问题都归给控制器 |

---

## 9. 发布检查

发布检查的目标是让观众看完视频后能继续学习，而不是只看一遍热闹。

### 9.1 视频文件检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 文件命名 | 检查导出文件名 | 包含集数、主题和版本 | 文件名只有日期或 `final_final` |
| 音频 | 试听开头、中间、结尾 | 声音清晰，无明显爆音或环境噪声 | 公式段音量忽大忽小 |
| 画面 | 抽查 PPT、RViz、终端段 | 文字可读，鼠标指示清楚 | 终端 topic 太小看不清 |
| 章节 | 按课程结构添加章节 | 观众能跳到公式、演示、排查段 | 长视频没有章节 |
| 字幕 | 至少检查关键术语字幕 | Lanelet、Trajectory、Velocity Smoother、Pure Pursuit、MPC、MPT 不出错 | 自动字幕把术语识别错 |
| 封面 | 检查封面主题 | 一眼看出本集主题和链路位置 | 封面只写 Autoware，信息太少 |

### 9.2 视频简介检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 本集解决的问题 | 简介第一段 | 用一句话说明本集回答什么问题 | 简介只堆关键词 |
| 适合人群 | 简介中写明 | 新手、做 Autoware 规划控制、准备读源码的人都能判断是否适合 | 观众不知道前置知识 |
| 章节时间戳 | 按实际视频添加 | 至少包含开场、核心概念、公式、实操、故障、总结 | 长视频没有导航 |
| 配套资料 | 列出对应 Markdown 文件 | 观众能找到 PPT 和 concept 素材 | 资料名和本地文件名不一致 |
| 相关前后集 | 写明上一集和下一集 | 形成学习路线 | 单集发布，没有课程感 |
| 免责声明 | 简短说明版本差异 | topic 和参数可能随 Autoware 版本变化 | 观众拿不同版本照抄报错 |

### 9.3 配套资料发布检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| PPT Markdown | 确认最终版本文件名 | 文件名和视频集数一致 | 发布了旧版 PPT |
| 源 concept 文档 | 归档对应 concept 文件 | 观众能追溯公式来源 | 只给 PPT，不给详细推导 |
| 端到端实战文档 | 提供新手友好详解版 | 观众能照着 topic 清单复现 | 实操内容只在视频里 |
| 检查清单 | 提供本文件 | 观众也能自查自己的环境 | 课程资料没有组织 |
| 图片素材 | 如果有手绘图，统一命名 | PPT 引用或说明不丢失 | 图片散落在临时目录 |
| rosbag 或示例命令 | 如果可公开，附最小复现说明 | 观众知道如何回放 | bag 很大但没有说明 |

### 9.4 发布后复盘检查

| 检查项 | 检查方法 | 通过标准 | 常见问题 |
|---|---|---|---|
| 评论问题收集 | 发布后记录高频问题 | 下一版能补 FAQ | 问题散在评论区 |
| 错误修正 | 如果发现公式或 topic 错误，记录 errata | 观众能看到修正 | 错误只在心里记着 |
| 补充专题 | 根据问题选择专题 | RViz 调试、rosbag 复盘、故障案例等可继续扩展 | 后续内容没有优先级 |

---

## 10. 分集最终检查表

正式录制前建议按这一节逐集打勾。

### 10.1 第 1 集：规划控制总览

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 核心主线 | 看是否反复出现 `Route -> Path -> Trajectory -> Control Command` | 新手能分清四层数据 |
| 模块地图 | 检查 Mission Planner、Behavior Planner、Smoother、Controller 都在图中 | 后续课程有统一地图 |
| 误区纠正 | 检查“目标点不能直接控制车辆” | 观众不会把目标点当控制命令 |
| 后续路线 | 检查结尾预告 | 能自然进入 Lanelet 与 Mission Planner |

### 10.2 第 2 集：Lanelet 与 Mission Planner

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| Lanelet 基础 | 检查 node、way、relation、左右边界、中心线 | 新手知道 Lanelet 是地图结构 |
| 图搜索 | 检查 `G=(V,E)` 和最短路页 | 知道 route 是在车道图中搜索 |
| 起终点匹配 | 检查朝向过滤、目标点合法性 | 知道为什么最近车道不一定正确 |
| Route Section | 检查 route section 和相邻候选 | 知道 route 不只是单条中心线 |
| 实操观察 | 检查 route topic 和 route marker | 能在 RViz 看到 route |

### 10.3 第 3 集：Hybrid A* Pull Out

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| Start Planner 定位 | 检查本集在链路中的位置 | 知道 Pull Out 是起步到车道的过渡 |
| Shift vs Hybrid A* | 检查二者边界 | 知道简单横移和自由空间搜索的区别 |
| 状态空间 | 检查 `(x,y,yaw)` 状态和运动原语 | 知道搜索考虑车辆朝向 |
| 代价函数 | 检查长度、倒车、换挡、转角、障碍距离 | 知道路径不只按长度最短 |
| 碰撞检查 | 检查 footprint、grid、bound | 知道为什么候选路径会被拒绝 |
| 安全检查 | 检查外层安全检查页 | 知道搜索成功不等于能发布 |

### 10.4 第 4 集：Elastic Band 路径平滑

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 离散点直觉 | 检查折线路径页 | 知道离散 path 不能直接给控制器 |
| 横向偏移 | 检查 `p_i=r_i+l_i n_i` | 知道为什么用局部法向偏移 |
| 二阶差分 | 检查三点图和矩阵图 | 知道平滑项惩罚折角 |
| QP 标准形式 | 检查目标和约束 | 知道 H、q、A、上下界各自作用 |
| 约束边界 | 检查 box constraint 和固定点 | 知道平滑不能越界 |

### 10.5 第 5 集：Velocity Smoother

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 不能硬裁剪 | 检查开头问题页 | 知道速度突变会带来加速度和 jerk 问题 |
| Jerk 直觉 | 检查 jerk 页 | 能用乘坐体验解释 jerk |
| 曲率限速 | 检查横向加速度限制 | 知道弯道速度为什么要降 |
| 前向/后向滤波 | 检查模块卡片 | 知道加速、减速约束如何传播 |
| 解析减速 | 检查三类减速曲线 | 知道不同距离下减速形状不同 |
| 调参排查 | 检查典型现象和排查页 | 急刹时知道先看 stop reason |

### 10.6 第 6 集：Pure Pursuit

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 追前方点 | 检查“为什么不能追最近点” | 知道前瞻目标点的稳定性意义 |
| 车辆坐标系 | 检查坐标变换页 | 知道公式中的 x、y 是车辆坐标 |
| 曲率公式 | 检查 `kappa=2y/(x^2+y^2)` | 知道目标点横向偏差决定曲率 |
| 转角公式 | 检查 `delta=atan(L kappa)` | 知道曲率如何变成前轮转角 |
| 前瞻点搜索 | 检查最近点、线段与圆求交 | 知道工程上目标点怎么找 |
| 参数影响 | 检查前瞻距离上下限 | 知道看近灵敏、看远稳定 |

### 10.7 第 7 集：横向 MPC

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 阿克曼到模型 | 检查阿克曼和自行车模型页 | 知道转角、曲率、轴距的关系 |
| 误差状态 | 检查横向误差、航向误差 | 知道 MPC 优化的是误差 |
| 线性化 | 检查线性化过渡页 | 知道为什么要把非线性模型局部近似 |
| 离散化 | 检查单步预测 | 知道每一步如何从当前状态到下一步 |
| 多步堆叠 | 检查预测矩阵 | 知道 MPC 看未来一段时间 |
| QP 目标 | 检查 H、g 来源 | 知道误差和控制输入如何变成二次代价 |
| 约束 | 检查转角和转角变化率 | 知道控制输入必须受物理限制 |
| 调参排查 | 检查抖动、外切、跟踪慢 | 知道先看轨迹和速度，再看权重 |

### 10.8 第 8 集：MPT 与整套链路复盘

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| MPT 定位 | 检查 MPT 在链路中的位置 | 知道 MPT 优化轨迹，不直接控制底盘 |
| MPT / Elastic Band | 检查区别页 | 知道平滑和模型预测优化的边界 |
| MPT / MPC | 检查区别页 | 知道 MPT 优化轨迹，MPC 优化控制输入 |
| 空间域模型 | 检查 `dl/ds`、`dtheta/ds` | 知道为什么 MPT 沿参考线距离推进 |
| QP 变量 | 检查 `v=[X,U,S]^T` | 知道状态、输入、松弛变量的作用 |
| 软约束 | 检查 soft constraint 页 | 知道为什么工程上常需要可放松约束 |
| 全链路复盘 | 检查 route/path/trajectory/control 大图 | 知道 MPT 在规划和控制之间的位置 |

### 10.9 端到端实战复盘

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 完整案例 | 检查从目标点到控制命令主线 | 能一口气讲完整链路 |
| 每阶段输入输出 | 检查 4A 阶段卡片 | 每层都有输入、输出、topic、RViz、参数 |
| topic 链路 | 检查 17.1 最小闭环 topic 清单 | 能按顺序查到第一个断点 |
| 故障决策树 | 检查 18A | 能按症状分流排查 |
| 录制操作 | 检查 19A | 能按导演表录制 |
| PPT 大纲 | 检查 20 | 能转成一集实战复盘视频 |

---

## 11. 录制当天流程

### 11.1 开始前 30 分钟

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 打开材料 | 打开本集 PPT、concept 文档、总索引、检查清单 | 录制时不需要临时找文件 |
| 启动环境 | 启动 Autoware / 仿真 / RViz | 地图、车辆、TF 正常 |
| 测试麦克风 | 录 10 秒试听 | 声音清楚 |
| 测试屏幕 | 录 10 秒 RViz + 终端 | topic 和 RViz 图层文字可读 |
| 测试 topic | 执行最小闭环命令 | planning/control topic 存在 |
| 测试目标点 | 点击一次目标点 | route 正常生成 |
| 测试控制输出 | 让链路走到 control cmd | follower cmd 和 gate cmd 可观察 |

### 11.2 录制中

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 开场 | 先讲本集要解决的问题 | 观众知道为什么要看这一集 |
| 主线 | 每 10 到 15 分钟回到总链路图 | 观众不会迷路 |
| 公式 | 每个公式按变量、物理意义、工程作用讲 | 新手能跟上 |
| 实操 | 每个模块至少展示一次输入输出 | 理论和系统能对应 |
| 故障 | 至少讲一个常见故障 | 课程有排查价值 |
| 总结 | 结尾用三句话收束 | 观众知道带走什么 |

### 11.3 录制后

| 检查项 | 检查方法 | 通过标准 |
|---|---|---|
| 回看开头 | 看前 3 分钟 | 开场清楚，没有卡顿 |
| 回看公式段 | 抽查最难公式 | 讲解不跳步 |
| 回看 RViz 段 | 抽查关键演示 | 画面清楚，topic 可读 |
| 回看故障段 | 确认排查顺序 | 不是只说结论 |
| 记录问题 | 写下需要补录或修正的点 | 发布前有修正清单 |

---

## 12. 下一次给 Codex 的指令

完成录制总检查清单后，建议下一步生成“课程发布包目录结构与 README”。这样可以把所有 PPT、concept 文档、实战复盘、检查清单、图片素材和后续补充专题组织成一个观众能直接使用的资料包。

你可以复制下面这段给 Codex：

```text
请执行优化任务 12：
在不覆盖原文件的前提下，
根据 AUTOWARE规划控制层课程总索引.md、
AUTOWARE规划控制层课程录制总检查清单.md、
当前根目录第1集到第8集的新手友好详解版 PPT、
以及 AUTOWARE规划控制层端到端实战复盘_新手友好详解版.md，
生成 AUTOWARE规划控制层课程发布包README.md。

要求：
1. 面向课程发布和观众自学；
2. 给出推荐目录结构；
3. 列出每个文件的用途、适合谁看、建议阅读顺序；
4. 给出视频简介模板、章节时间戳模板、资料下载说明模板；
5. 给出后续补充专题规划；
6. 保持新手友好，不精简关键内容；
7. 输出到项目根目录，不覆盖原文件；
8. 任务完成后，给出下一次任务的指令。
```
