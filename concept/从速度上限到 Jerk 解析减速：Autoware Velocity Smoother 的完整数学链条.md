参考：`concept/从阿克曼几何到 QP 求解器输入：自动驾驶横向 MPC 的完整数学链条.md`

---

# 目录

1. 为什么要单独理解 Velocity Smoother
2. 模块在 Autoware 规划链中的位置
3. 输入轨迹到底是什么：参考路径还是速度上限
4. 主回调的数据流：从 `onCurrentTrajectory()` 到最终发布
5. 外部限速：为什么不能立刻把速度硬改掉
6. 外部限速插入距离的完整推导
7. 外部限速距离为什么要逐帧递减
8. `calcTrajectoryVelocity()`：局部轨迹、限速与平滑调度
9. `smoothVelocity()`：速度平滑的总装配线
10. 初始速度和初始加速度为什么这么重要
11. 横向加速度约束：从 $a_y=v^2\kappa$ 到弯道限速
12. 方向盘角速度约束：从 $\delta=\arctan(L\kappa)$ 到速度上限
13. 重采样：为什么速度平滑前后都要改变轨迹点
14. 纵向速度规划的核心变量：为什么用 $b=v^2$
15. 从 $b'=2a$ 到 pseudo jerk：QP smoother 的数学骨架
16. Analytical smoother 的主思想：前向追踪 + 后向刹车
17. 减速目标点：为什么要寻找速度谷底
18. 多个减速目标如何分段：那个 `<` 条件的意义
19. 前向 jerk filter：如何在 jerk/acc 限制下追踪速度上限
20. 后向 decel filter：如何反推从哪里开始减速
21. `calcEnoughDistForDecel()`：判断距离是否足够
22. jerk 约束下的三类减速曲线
23. Type 1 梯形加速度曲线的完整推导
24. Type 2 三角形加速度曲线的完整推导
25. Type 3 仅恢复加速度曲线的完整推导
26. `calcStopVelocityWithConstantJerkAccLimit()`：如何把连续曲线写回离散轨迹
27. 为什么处理完所有目标后，还要再做一次前向积分
28. 后处理：停车点、后方点、最大速度和发布
29. 整套链条的统一大图景
30. 常见误区与统一澄清

---

# 1. 为什么要单独理解 Velocity Smoother

自动驾驶规划链里，路径规划器往往先回答这个问题：

> 车应该沿着哪条几何轨迹走？

但车真正能不能舒服、安全地走，还取决于另一个问题：

> 车应该以什么速度走？

如果只看路径点上的速度，你会发现上游轨迹经常已经带了速度字段。但这些速度并不一定已经满足所有真实车辆约束，例如：

* 当前车速和当前加速度能不能连续接上
* 前方限速能不能在 jerk 约束下平滑达到
* 弯道速度是否满足横向加速度限制
* 曲率变化是否会导致方向盘转太快
* 停车点是否能准确停住
* 每一帧规划之间速度和加速度是否连续

`autoware_velocity_smoother` 就是处理这些问题的模块。

它并不是重新规划道路几何，而是在已有轨迹上重新生成速度曲线。更准确地说，它把上游轨迹中的速度看成一组**速度上限**，然后根据车辆动力学、舒适性和外部约束，生成真正要发布给下游控制的速度。

它的核心链条可以概括为：

```text
输入轨迹速度上限
→ 外部限速/停车点/弯道/转向限制
→ 初始速度与加速度确定
→ 纵向 acc / jerk 约束下生成速度曲线
→ 后处理并发布最终 trajectory
```

---

# 2. 模块在 Autoware 规划链中的位置

在 core planning launch 里，velocity smoother 位于 motion velocity planner 后面，最终发布 `/planning/trajectory`。

相关源码入口主要是：

* `autoware_velocity_smoother/src/node.cpp`
* `smoother/smoother_base.cpp`
* `smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp`
* `smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.cpp`

它的输入主要包括：

```text
~/input/trajectory
/localization/kinematic_state
~/input/acceleration
~/input/external_velocity_limit_mps
~/input/operation_mode_state
```

输出是：

```text
~/output/trajectory
```

这意味着 velocity smoother 是一个典型的“最后速度整形层”：

```text
路径/行为/障碍物规划
→ 带速度上限的轨迹
→ velocity smoother
→ 满足车辆纵向/横向舒适性约束的最终轨迹
→ 控制器
```

---

# 3. 输入轨迹到底是什么：参考路径还是速度上限

`TrajectoryPoint` 里有位姿、速度、加速度等字段。对 velocity smoother 来说，输入轨迹里的 `longitudinal_velocity_mps` 更应该理解成：

> 这个位置处允许的最大速度，或者希望尽量靠近的参考速度。

为什么不是“必须照抄的速度”？

因为如果输入速度突然从 $10\,m/s$ 变成 $0\,m/s$，车辆不可能瞬间停车。真实车辆至少要满足：

$$
a_{\min} \le a \le a_{\max}
$$

以及：

$$
j_{\min} \le j \le j_{\max}
$$

其中：

$$
j = \frac{da}{dt}
$$

所以 smoother 的任务不是简单覆盖速度字段，而是在这些速度上限下面寻找一条可执行速度曲线。

可以把输入输出关系理解成：

```text
输入速度：上游模块给出的速度上限 v_max(s)
输出速度：车辆可平滑执行的速度 v(s)

要求：
0 <= v(s) <= v_max(s)
a、j 尽量平滑且不超限
```

---

# 4. 主回调的数据流：从 `onCurrentTrajectory()` 到最终发布

主入口是：

```cpp
VelocitySmootherNode::onCurrentTrajectory(...)
```

它由 `~/input/trajectory` 触发。其他数据不是各自 callback 里立即计算，而是在轨迹 callback 内主动取：

```cpp
current_odometry_ptr_ = sub_current_odometry_.take_data();
current_acceleration_ptr_ = sub_current_acceleration_.take_data();
external_velocity_limit_ptr_ = sub_external_velocity_limit_.take_data();
```

这是一种“以轨迹为主时钟”的处理方式。

主流程可以画成：

```text
收到轨迹
  ↓
取当前 odom / acceleration / external velocity limit / operation mode
  ↓
检查数据是否齐全
  ↓
转换为 TrajectoryPoints
  ↓
删除重叠点
  ↓
末点速度设 0
  ↓
在上一帧输出轨迹上投影 ego，得到上一帧规划的当前 v/a
  ↓
计算外部限速插入距离
  ↓
倒车时先把速度翻成正值
  ↓
calcTrajectoryVelocity()
  ↓
post resampling
  ↓
更新 prev_output_
  ↓
倒车时速度翻回负值
  ↓
发布最终轨迹
```

这里最关键的一点是：

> 当前帧的初始状态通常来自上一帧输出轨迹在当前 ego 位置的投影，而不只是当前 odom。

这样做是为了保证帧间速度和加速度连续。

---

# 5. 外部限速：为什么不能立刻把速度硬改掉

外部限速来自：

```text
~/input/external_velocity_limit_mps
```

比如某个外部模块要求最大速度从 $10\,m/s$ 降到 $5\,m/s$。

最简单粗暴的做法是：

```text
从当前点开始，把所有速度都 clamp 到 5 m/s
```

但这在物理上不合理。因为当前车辆可能正在：

* 以 $10\,m/s$ 行驶
* 甚至还带有正加速度 $a_0 > 0$

如果立刻把当前速度上限改为 $5\,m/s$，下游控制器会看到一个非常突兀的速度目标，等价于要求车辆产生很大的减速度和 jerk。

所以代码的策略是：

```text
先计算在 jerk / acc 约束下，从当前状态降到新限速需要多远
然后在前方这个距离处插入限速点
从该点之后才应用新限速
```

这就是：

```cpp
calcExternalVelocityLimit()
```

的核心意义。

---

# 6. 外部限速插入距离的完整推导

设当前规划状态为：

$$
v_0,\quad a_0
$$

新外部限速为：

$$
v_{\text{lim}}
$$

负 jerk 为：

$$
j_{\min} < 0
$$

正 jerk 为：

$$
j_{\max} > 0
$$

最小加速度为：

$$
a_{\min} < 0
$$

代码里有一段非常关键：

```cpp
if (a0 > 0) {
  max_velocity_with_deceleration_ = v0 - 0.5 * a0 * a0 / j_min;
} else {
  max_velocity_with_deceleration_ = v0;
}
```

为什么是这个公式？

如果当前 $a_0>0$，车辆正在加速。即使立刻施加负 jerk，让加速度下降到 0，也需要时间。

加速度变化为：

$$
a(t)=a_0+j_{\min}t
$$

令加速度降到 0：

$$
0=a_0+j_{\min}t
$$

得到：

$$
t=-\frac{a_0}{j_{\min}}
$$

这段时间内速度仍然增加：

$$
v(t)=v_0+a_0t+\frac{1}{2}j_{\min}t^2
$$

代入 $t=-a_0/j_{\min}$：

$$
\Delta v
=a_0\left(-\frac{a_0}{j_{\min}}\right)
+\frac{1}{2}j_{\min}
\left(\frac{a_0^2}{j_{\min}^2}\right)
$$

$$
\Delta v
=-\frac{a_0^2}{j_{\min}}
+\frac{1}{2}\frac{a_0^2}{j_{\min}}
$$

$$
\Delta v
=-\frac{1}{2}\frac{a_0^2}{j_{\min}}
$$

由于 $j_{\min}<0$，所以 $\Delta v>0$。

于是峰值速度为：

$$
v_{\text{peak}}
=v_0-\frac{1}{2}\frac{a_0^2}{j_{\min}}
$$

这就是代码里的：

```cpp
v0 - 0.5 * a0 * a0 / j_min
```

如果：

$$
v_{\text{lim}} < v_{\text{peak}}
$$

说明车辆需要提前减速。代码会调用：

```cpp
trajectory_utils::calcStopDistWithJerkConstraints(...)
```

计算满足 jerk 和加速度限制的减速距离：

$$
s_{\text{stop}}
$$

最后：

$$
\text{external\_velocity\_limit.dist}
=s_{\text{stop}}+\text{margin}
$$

也就是在前方这个距离插入外部限速点。

---

# 7. 外部限速距离为什么要逐帧递减

外部限速插入距离一旦算出来，例如：

$$
30\,m
$$

它表示：

> 从当前 ego 位置往前 30m 插入限速点。

但车辆每一帧都在往前走。假设下一帧车前进了 $2m$，限速点相对 ego 的剩余距离就应该变成：

$$
30-2=28\,m
$$

这就是：

```cpp
updateDataForExternalVelocityLimit()
```

做的事情：

```cpp
const double travel_dist = calcTravelDistance();
external_velocity_limit_.dist =
  std::max(external_velocity_limit_.dist - travel_dist, 0.0);
```

如果没有这个函数，限速点就会每一帧都像“固定在 ego 前方 30m”一样往前漂，车辆可能永远到不了真正限速生效的位置。

---

# 8. `calcTrajectoryVelocity()`：局部轨迹、限速与平滑调度

`calcTrajectoryVelocity()` 是速度平滑的预处理调度层。

它做的事情是：

```text
完整输入轨迹
  ↓
找 ego 最近点
  ↓
截取 ego 前后局部轨迹
  ↓
应用外部限速
  ↓
重新找局部轨迹最近点
  ↓
应用停车点接近速度
  ↓
smoothVelocity()
```

为什么要截取局部轨迹？

因为速度优化不需要处理无限远的轨迹。当前帧真正有意义的是：

```text
ego 后方 extract_behind_dist
ego 前方 extract_ahead_dist
```

后方点主要用于输出轨迹连续；前方点用于实际速度规划。

外部限速会直接修改这段局部轨迹的速度上限。停车点接近速度也会修改速度上限。

所以进入 `smoothVelocity()` 的输入可以理解成：

```text
已经叠加外部限速、停车点限制的局部速度上限轨迹
```

---

# 9. `smoothVelocity()`：速度平滑的总装配线

`smoothVelocity()` 是 node 层真正的核心调度函数。

流程是：

```text
input 局部轨迹
  ↓
calcInitialMotion() 得到初始 v0/a0
  ↓
横向加速度限速
  ↓
方向盘角速度限速
  ↓
重采样
  ↓
从 ego 最近点裁剪前方轨迹 clipped
  ↓
smoother_->apply(v0, a0, clipped, traj_smoothed)
  ↓
停车点强制修正
  ↓
补回 ego 后方点
  ↓
最大速度保护
```

这里的 `smoother_` 可以是：

* `JerkFiltered`
* `L2`
* `Linf`
* `Analytical`

本文重点展开 `AnalyticalJerkConstrainedSmoother`，但中间的横向加速度、转向角速度、空间域速度建模，对其他 smoother 也有帮助。

---

# 10. 初始速度和初始加速度为什么这么重要

纵向规划不是只规划未来速度，还要保证当前点接得上。

数学上，速度曲线需要初始条件：

$$
v(0)=v_0
$$

$$
a(0)=a_0
$$

如果每帧都随意选择 $v_0,a_0$，输出曲线会抖动。

`calcInitialMotion()` 会根据场景选择初值：

```text
首次计算：
  v0 = 当前车速
  a0 = 0

速度跟踪偏差过大：
  v0 = 当前车速
  a0 = 上一帧规划加速度

engage 起步：
  v0 = engage_velocity
  a0 = engage_acceleration

正常情况：
  v0 = 上一帧输出轨迹在当前 ego 位置的投影速度
  a0 = 上一帧输出轨迹在当前 ego 位置的投影加速度
```

这就是 velocity smoother 能做到帧间连续的重要原因。

---

# 11. 横向加速度约束：从 $a_y=v^2\kappa$ 到弯道限速

车辆沿曲线行驶时，横向加速度为：

$$
a_y=\frac{v^2}{R}
$$

曲率定义为：

$$
\kappa=\frac{1}{R}
$$

所以：

$$
a_y=v^2\kappa
$$

为了让横向加速度不超过限制：

$$
|a_y| \le a_{y,\max}
$$

代入：

$$
v^2|\kappa| \le a_{y,\max}
$$

得到速度上限：

$$
v \le \sqrt{\frac{a_{y,\max}}{|\kappa|}}
$$

代码中：

```cpp
applyLateralAccelerationFilter()
```

做的就是：

```text
等间距重采样
  ↓
三点法计算曲率
  ↓
根据 a_y = v^2 κ 算每个点的最大速度
  ↓
只降速，不提速
```

为什么 `lateral_acceleration_limits` 可以按速度区间配置？

从纯摩擦极限看：

$$
a_y \le \mu g
$$

这个极限主要由路面附着系数决定。但规划里设置的横向加速度上限通常不是轮胎极限，而是舒适性、控制裕度和安全冗余。

高速下同样的横向加速度会带来更明显的横摆响应、轨迹误差放大和乘坐不适，所以工程上常常让高速区间更保守。

---

# 12. 方向盘角速度约束：从 $\delta=\arctan(L\kappa)$ 到速度上限

用单轨自行车模型，前轮转角和曲率满足：

$$
\tan\delta=L\kappa
$$

因此：

$$
\delta=\arctan(L\kappa)
$$

方向盘角速度限制关注的是：

$$
\dot\delta=\frac{d\delta}{dt}
$$

而轨迹按弧长 $s$ 排列：

$$
\dot\delta
=\frac{d\delta}{ds}\frac{ds}{dt}
$$

由于：

$$
\frac{ds}{dt}=v
$$

所以：

$$
\dot\delta=v\frac{d\delta}{ds}
$$

若方向盘角速度限制为：

$$
|\dot\delta|\le \dot\delta_{\max}
$$

则：

$$
v\left|\frac{d\delta}{ds}\right|
\le \dot\delta_{\max}
$$

得到速度上限：

$$
v \le
\frac{\dot\delta_{\max}}
{\left|\frac{d\delta}{ds}\right|}
$$

代码中的 `applySteeringRateLimit()` 做的就是：

```text
曲率 κ
  ↓
前轮转角 δ = atan(Lκ)
  ↓
计算 |Δδ/Δs|
  ↓
根据 δ_dot = v |Δδ/Δs| 反推速度上限
  ↓
如果当前速度会让转向角速度超限，则降速
```

横向加速度限制看的是“弯本身有多急”：

$$
a_y=v^2\kappa
$$

转向角速度限制看的是“曲率变化有多快”：

$$
\dot\delta=v\frac{d\delta}{ds}
$$

所以它们互补：

```text
大半径长弯：横向加速度可能主导
S 弯/曲率突变：方向盘角速度可能主导
```

---

# 13. 重采样：为什么速度平滑前后都要改变轨迹点

速度规划里经常会用到：

$$
\Delta s
$$

例如：

$$
\frac{\Delta \delta}{\Delta s}
$$

$$
\frac{\Delta a}{\Delta s}
$$

如果轨迹点间距忽大忽小，差分量会非常不稳定。

所以 smoother 会在不同阶段重采样：

* 横向加速度/转向角速度计算前：让曲率和转角差分稳定
* 优化/解析速度规划前：让速度规划点距合适
* 发布前 post resampling：让输出轨迹适合下游控制器

重采样不是改变道路几何意图，而是在同一条轨迹上重新选择采样点。

---

# 14. 纵向速度规划的核心变量：为什么用 $b=v^2$

许多纵向速度优化器不直接优化 $v$，而是优化：

$$
b=v^2
$$

原因是车辆纵向动力学在空间域下会变得非常整齐。

加速度定义：

$$
a=\frac{dv}{dt}
$$

用链式法则：

$$
\frac{dv}{dt}
=\frac{dv}{ds}\frac{ds}{dt}
$$

而：

$$
\frac{ds}{dt}=v
$$

所以：

$$
a=v\frac{dv}{ds}
$$

再看：

$$
b=v^2
$$

对 $s$ 求导：

$$
\frac{db}{ds}=2v\frac{dv}{ds}
$$

代入 $a=v\frac{dv}{ds}$：

$$
\frac{db}{ds}=2a
$$

离散化：

$$
\frac{b_{i+1}-b_i}{\Delta s_i}=2a_i
$$

这就是 QP smoother 中最重要的动力学约束。

---

# 15. 从 $b'=2a$ 到 pseudo jerk：QP smoother 的数学骨架

jerk 定义：

$$
j=\frac{da}{dt}
$$

同样换到空间域：

$$
j
=\frac{da}{ds}\frac{ds}{dt}
=v\frac{da}{ds}
$$

离散化：

$$
j_i
\approx v_{\text{ref},i}
\frac{a_{i+1}-a_i}{\Delta s_i}
$$

其中：

$$
\frac{a_{i+1}-a_i}{\Delta s_i}
$$

常被称为 pseudo jerk，乘上参考速度后近似真实时间 jerk。

QP smoother 的变量通常类似：

$$
x=
\begin{bmatrix}
b_0,\dots,b_N,
a_0,\dots,a_N,
\delta_0,\dots,\delta_N,
\sigma_0,\dots,\sigma_N
\end{bmatrix}
$$

其中：

* $b_i=v_i^2$
* $a_i$ 是加速度
* $\delta_i$ 是速度上限松弛变量
* $\sigma_i$ 是加速度约束松弛变量

典型目标函数是：

$$
\min
\left(
-\sum_i b_i
+w_j\sum_i
\left(
\frac{a_{i+1}-a_i}{\Delta s_i}
\right)^2
+w_v\sum_i \delta_i^2
+w_a\sum_i \sigma_i^2
\right)
$$

第一项 $-\sum b_i$ 表示尽量快，后面几项表示平滑和惩罚约束违反。

约束包括：

$$
0 \le b_i-\delta_i \le v_{\max,i}^2
$$

$$
a_{\min}\le a_i-\sigma_i\le a_{\max}
$$

$$
\frac{b_{i+1}-b_i}{\Delta s_i}=2a_i
$$

$$
b_0=v_0^2,\quad a_0=a_0
$$

Analytical smoother 不直接解这个 QP，但它处理的是同一个物理问题。

---

# 16. Analytical smoother 的主思想：前向追踪 + 后向刹车

`AnalyticalJerkConstrainedSmoother::apply()` 的思路是：

```text
1. 找出轨迹中的减速目标点
2. 前向 jerk filter：从当前状态开始，尽量追踪速度上限
3. 后向 decel filter：对每个低速目标反推能否及时减速
4. 最后一个目标之后，再前向积分一遍
```

它不求全局最优，而是用解析式和分段逻辑快速生成速度曲线。

优点：

* 快
* 可解释
* 减速距离可以解析计算

缺点：

* 不像 QP 那样天然全局优化
* 依赖减速目标点和启发式分段逻辑

---

# 17. 减速目标点：为什么要寻找速度谷底

函数：

```cpp
searchDecelTargetIndices()
```

会寻找输入速度上限中的局部低速点。

如果：

$$
v_i-v_{i-1}<0
$$

且：

$$
v_{i+1}-v_i>0
$$

说明速度先下降后上升，$i$ 是一个速度谷底。

例如：

```text
10, 9, 7, 4, 6, 8
          ↑
       减速目标
```

这些速度谷底可能来自：

* 弯道横向加速度限速
* 方向盘角速度限速
* 外部限速
* 停车点
* 轨迹末端速度 0

Analytical smoother 的关键就是保证：

> 车辆能在 jerk/acc 约束下平滑到达这些低速目标。

---

# 18. 多个减速目标如何分段：那个 `<` 条件的意义

代码里有一段：

```cpp
if (decel_target_indices.at(j - 1).second <
    decel_target_indices.at(j).second) {
  ...
}
```

每个减速目标是：

```cpp
std::pair<size_t, double>
```

其中：

```text
first  = 目标点 index
second = 目标速度
```

所以这个条件的含义是：

> 如果前一个减速目标的速度，比当前减速目标更低。

例如：

```text
target0 = 2 m/s
target1 = 8 m/s
```

当前处理 `target1` 时，前面已经有一个更低速的 `target0`。车辆经过 `target0` 后速度已经很低，再处理 `target1` 时，可以从 `target0` 作为新的分界点开始。

```text
ego ---- target0(2m/s) ---- target1(8m/s)
              ↑
       这里已经是更严格低速约束
```

反过来：

```text
target0 = 8 m/s
target1 = 2 m/s
```

当前目标更低，可能需要在 `target0` 之前就开始为 `target1` 减速，不能简单从 `target0` 才开始。

所以这个 `<` 条件的本质是：

> 找到一个比当前目标更低速的前一个目标，把它作为减速区间分界。

这避免把多个独立减速事件混在一起，也避免低估更低速目标所需的减速距离。

---

# 19. 前向 jerk filter：如何在 jerk/acc 限制下追踪速度上限

`applyForwardJerkFilter()` 从某个起点状态开始向前积分。

已知上一点：

$$
v_{i-1},\quad a_{i-1}
$$

两点间距离：

$$
\Delta s_i
$$

估计时间间隔：

$$
\Delta t_i
=\frac{\Delta s_i}{\max(v_{i-1},1.0)}
$$

先用上一点加速度预测当前速度：

$$
v_i^{\text{pred}}
=v_{i-1}+a_{i-1}\Delta t_i
$$

速度误差：

$$
e_v=v_{\text{ref},i}-v_i^{\text{pred}}
$$

用比例控制得到期望加速度：

$$
a_i^{\text{des}}=k_p e_v
$$

再做加速度限幅：

$$
a_i^{\text{lim}}
=\text{clamp}
\left(
a_i^{\text{des}},
a_{\min},
a_{\max}
\right)
$$

由加速度变化得到期望 jerk：

$$
j_i^{\text{des}}
=\frac{a_i^{\text{lim}}-a_{i-1}}{\Delta t_i}
$$

再做 jerk 限幅：

$$
j_i
=\text{clamp}
\left(
j_i^{\text{des}},
j_{\min},
j_{\max}
\right)
$$

最后更新加速度：

$$
a_i=a_{i-1}+j_i\Delta t_i
$$

这是一种启发式的 jerk-limited 前向追踪：

```text
速度尽量追上参考上限
但加速度和 jerk 不能跳变
```

---

# 20. 后向 decel filter：如何反推从哪里开始减速

前向滤波擅长追踪速度上限，但它不能保证前方低速点一定能刹住。

所以对每个减速目标，代码会调用：

```cpp
applyBackwardDecelFilter()
```

它做的是：

```text
给定目标点 index 和目标速度
  ↓
尝试多个候选起点
  ↓
尝试多个负 jerk
  ↓
解析计算从起点降到目标速度需要多远
  ↓
如果 stop_dist <= 起点到目标点距离，则可行
  ↓
选择最温和且可行的 jerk
  ↓
把解析减速曲线写回轨迹
```

这里的关键判断是：

$$
0\le s_{\text{stop}}\le s_{\text{allowed}}
$$

其中：

* $s_{\text{stop}}$ 是解析减速所需距离
* $s_{\text{allowed}}$ 是起点到目标点的轨迹距离

---

# 21. `calcEnoughDistForDecel()`：判断距离是否足够

`calcEnoughDistForDecel()` 是一个可行性检查器。

它取起点状态：

$$
v_0=\text{trajectory[start\_index].velocity}
$$

$$
a_0=\text{trajectory[start\_index].acceleration}
$$

给定负 jerk：

$$
j_{\text{dec}}<0
$$

正 jerk：

$$
j_{\text{acc}}=|j_{\text{dec}}|
$$

目标速度：

$$
v_{\text{target}}
$$

然后调用：

```cpp
calcStopDistWithJerkAndAccConstraints()
```

得到：

$$
s_{\text{stop}}
$$

最后判断：

$$
0\le s_{\text{stop}}
\le
\text{dist\_to\_target[start\_index]}
$$

如果成立，说明从这个点开始，用这个 jerk 和加速度限制，来得及降到目标速度。

---

# 22. jerk 约束下的三类减速曲线

解析减速的基础公式来自：

$$
j=\frac{da}{dt}
$$

积分得到：

$$
a(t)=a_0+jt
$$

$$
v(t)=v_0+a_0t+\frac{1}{2}jt^2
$$

$$
x(t)=x_0+v_0t+\frac{1}{2}a_0t^2+\frac{1}{6}jt^3
$$

为了从：

$$
(v_0,a_0)
$$

到：

$$
(v_{\text{target}},0)
$$

代码分三类曲线：

```text
Type 1：梯形加速度
  负 jerk -> 恒定 min_acc -> 正 jerk

Type 2：三角形加速度
  负 jerk -> 正 jerk

Type 3：仅恢复加速度
  正 jerk
```

---

# 23. Type 1 梯形加速度曲线的完整推导

Type 1 假设加速度会降到：

$$
a_{\min}
$$

并保持一段时间。

三个阶段为：

```text
阶段 1：a0 -> a_min，jerk = j_dec < 0
阶段 2：保持 a_min，jerk = 0
阶段 3：a_min -> 0，jerk = j_acc > 0
```

阶段 1 时间：

$$
t_1=\frac{a_{\min}-a_0}{j_{\text{dec}}}
$$

阶段 3 时间：

$$
t_3=\frac{0-a_{\min}}{j_{\text{acc}}}
$$

对于常 jerk 阶段，加速度是线性的，速度变化量等于加速度曲线面积：

$$
\Delta v
=\frac{a_{\text{start}}+a_{\text{end}}}{2}t
$$

阶段 1 的速度变化可以拆成两部分：

$$
\Delta v_1
=
\frac{1}{2}
\frac{0-a_0}{j_{\text{dec}}}a_0
+
\frac{1}{2}
\frac{a_{\min}-0}{j_{\text{dec}}}a_{\min}
$$

也就是代码里的：

```cpp
0.5 * (0 - a0) / jerk_dec * a0
+ 0.5 * min_acc / jerk_dec * min_acc
```

阶段 3：

$$
\Delta v_3
=
\frac{1}{2}
\frac{0-a_{\min}}{j_{\text{acc}}}a_{\min}
$$

阶段 2：

$$
\Delta v_2=a_{\min}t_{\min}
$$

总速度变化：

$$
v_{\text{target}}-v_0
=
\Delta v_1
+a_{\min}t_{\min}
+\Delta v_3
$$

解出：

$$
t_{\min}
=
\frac{
v_{\text{target}}-v_0-\Delta v_1-\Delta v_3
}{a_{\min}}
$$

如果：

$$
t_{\min}>0
$$

说明确实需要一个恒定 $a_{\min}$ 阶段，这就是 Type 1。

最终距离由三段积分得到：

$$
x_1=v_0t_1+\frac{1}{2}a_0t_1^2+\frac{1}{6}j_{\text{dec}}t_1^3
$$

$$
x_2=x_1+v_1t_2+\frac{1}{2}a_1t_2^2
$$

$$
x_3=x_2+v_2t_3+\frac{1}{2}a_2t_3^2+\frac{1}{6}j_{\text{acc}}t_3^3
$$

最后：

$$
s_{\text{stop}}=x_3
$$

---

# 24. Type 2 三角形加速度曲线的完整推导

如果：

$$
t_{\min}\le 0
$$

说明速度下降量不大，不需要真的达到 $a_{\min}$。

此时曲线为：

```text
a0 -> a1 -> 0
```

其中 $a_1<0$ 是负加速度峰值。

总速度变化为：

$$
v_{\text{target}}-v_0
=
\Delta v_{a_0\to 0}
+\Delta v_{0\to a_1}
+\Delta v_{a_1\to 0}
$$

第一项：

$$
\Delta v_{a_0\to 0}
=
\frac{1}{2}
\frac{0-a_0}{j_{\text{dec}}}a_0
$$

第二项：

$$
\Delta v_{0\to a_1}
=
\frac{1}{2}
\frac{a_1-0}{j_{\text{dec}}}a_1
=
\frac{1}{2}
\frac{a_1^2}{j_{\text{dec}}}
$$

第三项：

$$
\Delta v_{a_1\to 0}
=
\frac{1}{2}
\frac{0-a_1}{j_{\text{acc}}}a_1
=
-\frac{1}{2}
\frac{a_1^2}{j_{\text{acc}}}
$$

因此：

$$
v_{\text{target}}-v_0-\Delta v_{a_0\to 0}
=
\frac{1}{2}a_1^2
\left(
\frac{1}{j_{\text{dec}}}
-
\frac{1}{j_{\text{acc}}}
\right)
$$

通分：

$$
\frac{1}{j_{\text{dec}}}
-
\frac{1}{j_{\text{acc}}}
=
\frac{j_{\text{acc}}-j_{\text{dec}}}
{j_{\text{acc}}j_{\text{dec}}}
$$

所以：

$$
a_1^2
=
\left(
v_{\text{target}}-v_0-\Delta v_{a_0\to 0}
\right)
\frac{2j_{\text{acc}}j_{\text{dec}}}
{j_{\text{acc}}-j_{\text{dec}}}
$$

代码对应：

```cpp
const double a1_square =
  (target_vel - v0 - 0.5 * (0 - a0) / jerk_dec * a0)
  * (2 * jerk_acc * jerk_dec / (jerk_acc - jerk_dec));
const double a1 = -std::sqrt(a1_square);
```

取负根是因为这是减速曲线，$a_1$ 必须为负。

两段时间：

$$
t_1=\frac{a_1-a_0}{j_{\text{dec}}}
$$

$$
t_2=\frac{0-a_1}{j_{\text{acc}}}
$$

距离同样由积分得到：

$$
x_1=v_0t_1+\frac{1}{2}a_0t_1^2+\frac{1}{6}j_{\text{dec}}t_1^3
$$

$$
x_2=x_1+v_1t_2+\frac{1}{2}a_1t_2^2+\frac{1}{6}j_{\text{acc}}t_2^3
$$

---

# 25. Type 3 仅恢复加速度曲线的完整推导

如果当前已经在足够减速，不需要再施加负 jerk，只需要让加速度恢复到 0。

曲线为：

```text
a0 -> 0，jerk = j_acc
```

时间：

$$
t_1=\frac{0-a_0}{j_{\text{acc}}}
$$

速度：

$$
v_1=v_0+a_0t_1+\frac{1}{2}j_{\text{acc}}t_1^2
$$

距离：

$$
x_1=v_0t_1+\frac{1}{2}a_0t_1^2+\frac{1}{6}j_{\text{acc}}t_1^3
$$

如果 $v_1$ 接近目标速度，且 $a_1$ 接近 0，则 Type 3 成立。

代码用：

```cpp
validCheckCalcStopDist(...)
```

检查终点：

$$
v_{\text{end}}\approx v_{\text{target}}
$$

$$
a_{\text{end}}\approx 0
$$

---

# 26. `calcStopVelocityWithConstantJerkAccLimit()`：如何把连续曲线写回离散轨迹

前面的函数只算出了：

```text
type
times
stop_dist
```

但轨迹是离散点，最终要更新：

```text
output_trajectory[i].longitudinal_velocity_mps
output_trajectory[i].acceleration_mps2
```

`calcStopVelocityWithConstantJerkAccLimit()` 做的是：

```text
连续时间解析曲线
  ↓
按 dt = 0.1s 采样
  ↓
得到 ts, xs, vs, as, js
  ↓
计算轨迹点相对 start_index 的距离 distances
  ↓
用 xs 作为横轴，将 v/a/j 插值到 distances
  ↓
写回轨迹点
```

为什么用距离 $x$ 插值，而不是时间 $t$？

因为轨迹点是按空间弧长排列的，某个轨迹点表示“从起点走了多少米”，不是“经过了多少秒”。

所以用：

$$
v(x),\quad a(x)
$$

把连续曲线投影回轨迹点最自然。

若解析曲线结束后还有轨迹点，则后续点都设为：

$$
v=v_{\text{target}},\quad a=0
$$

表示已经完成减速，后面保持目标速度。

---

# 27. 为什么处理完所有目标后，还要再做一次前向积分

在 `AnalyticalJerkConstrainedSmoother::apply()` 末尾，有一段：

```cpp
applyForwardJerkFilter(
  reference_trajectory, start_index, start_vel, start_acc,
  smoother_param_, filtered_trajectory);
```

看起来像“已经处理完了，为什么还要做一次”。

原因是：

> 前面的循环只保证能到达每个减速目标，但最后一个减速目标之后的剩余轨迹还需要继续生成速度。

例如：

```text
ego ---- 弯道低速点 3m/s ---- 出弯长直路 15m/s ---- 终点
```

后向减速会保证车辆在弯道低速点达到 $3m/s$。

但弯道之后需要重新加速到 $15m/s$，而加速也要满足：

$$
a\le a_{\max}
$$

$$
j\le j_{\max}
$$

所以必须从最后一个减速目标点的实际速度和加速度开始，再前向积分一遍。

如果没有任何减速目标，这最后一次前向积分则负责整条轨迹的速度生成。

---

# 28. 后处理：停车点、后方点、最大速度和发布

速度平滑之后，还会做一些安全后处理：

## 28.1 停车点强制归零

优化或解析过程可能因为约束冲突，导致停车点附近速度没有精确到 0。

所以：

```cpp
overwriteStopPoint(...)
```

会从输入停车点对应位置之后强制速度为 0。

## 28.2 补回 ego 后方点

平滑器只处理 ego 前方 clipped 轨迹。

但输出 trajectory 仍然需要包含 ego 后方点，于是把重采样轨迹中最近点之前的点插回去。

后方点速度由：

```cpp
insertBehindVelocity(...)
```

补齐。正常情况下参考上一帧输出；如果是首次、engage、偏差重规划，则用最近点速度填充。

## 28.3 全局最大速度保护

最终再做：

```cpp
applyMaximumVelocityLimit(...)
```

确保不超过 `node_param_.max_velocity`。

## 28.4 post resampling

最后发布前还会做 post resampling，使输出轨迹点间距适合下游控制器。

---

# 29. 整套链条的统一大图景

把所有内容合起来，Velocity Smoother 的完整链条是：

```text
上游轨迹
  ↓
去重、末点速度设 0
  ↓
上一帧轨迹投影，确定连续初始 v/a
  ↓
外部限速：
  用 jerk/acc 约束计算限速插入距离
  ↓
截取 ego 附近局部轨迹
  ↓
应用外部限速和停车接近速度
  ↓
横向加速度限速：
  a_y = v^2 κ
  ↓
方向盘角速度限速：
  δ = atan(Lκ), δ_dot = v dδ/ds
  ↓
重采样
  ↓
Analytical smoother：
    找速度谷底
    前向 jerk filter
    后向解析 decel filter
    最后一段前向积分
  ↓
停车点强制归零
  ↓
补回 ego 后方点
  ↓
最大速度保护
  ↓
post resampling
  ↓
发布 /planning/trajectory
```

它的数学主线是：

```text
几何曲率
→ 横向速度限制
→ 转向速度限制
→ 纵向速度上限
→ jerk/acc 约束下的可执行速度曲线
```

核心公式是：

$$
a_y=v^2\kappa
$$

$$
\delta=\arctan(L\kappa)
$$

$$
\dot\delta=v\frac{d\delta}{ds}
$$

$$
a(t)=a_0+jt
$$

$$
v(t)=v_0+a_0t+\frac{1}{2}jt^2
$$

$$
x(t)=x_0+v_0t+\frac{1}{2}a_0t^2+\frac{1}{6}jt^3
$$

---

# 30. 常见误区与统一澄清

## 30.1 输入轨迹速度不是最终速度

输入速度更像速度上限。最终速度必须满足 acc、jerk、横向加速度、转向角速度等约束。

## 30.2 横向加速度上限不等于轮胎摩擦极限

摩擦极限是：

$$
a_y\le \mu g
$$

但代码参数通常是舒适性和控制裕度限制，远低于真实轮胎极限。

## 30.3 外部限速不能立即硬切

因为车辆当前可能速度高、加速度为正。必须先计算满足 jerk/acc 的减速距离，再插入限速点。

## 30.4 `updateDataForExternalVelocityLimit()` 不是重新计算减速距离

它只是根据车辆已经行驶的距离，递减外部限速点相对 ego 的剩余距离。

## 30.5 Analytical smoother 不是 QP

它不是全局优化，而是：

```text
前向追踪速度上限
后向确保低速目标可达
```

## 30.6 最后一次 forward filter 不是重复计算

它负责最后一个减速目标之后的剩余轨迹；如果没有减速目标，它负责整条轨迹。

## 30.7 Type 1 / Type 2 / Type 3 的区别

```text
Type 1：需要达到 min_acc，并保持一段时间
Type 2：不需要达到 min_acc，只需要一个负加速度峰值
Type 3：当前已经在减速，只需要把加速度恢复到 0
```

---

# 结语

Velocity Smoother 看起来只是“把速度平滑一下”，但它背后其实连接了三类数学：

* 路径几何：曲率、前轮转角、转向角速度
* 车辆运动学：速度、加速度、jerk 的积分关系
* 优化/解析规划：速度上限、可行减速距离、离散轨迹插值

如果把这条链条看清楚，就会发现它不是一个孤立的滤波器，而是把上游规划意图翻译成车辆可执行运动的关键桥梁。

