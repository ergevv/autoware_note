# MPT 数学原理与变量传递说明

这份说明对应 `autoware_path_optimizer` 中的 MPT，也就是 Model Predictive
Trajectory。重点对应这些实现：

- `MPTOptimizer::optimizeTrajectory()`：主流程
- `MPTOptimizer::calcReferencePoints()`：生成优化用参考点
- `StateEquationGenerator::calcMatrix()`：生成离散状态方程
- `MPTOptimizer::calcValueMatrix()`：生成状态/输入权重矩阵
- `MPTOptimizer::calcObjectiveMatrix()`：生成 QP 的 Hessian 和 gradient
- `MPTOptimizer::calcConstraintMatrix()`：生成 QP 约束
- `MPTOptimizer::calcOptimizedSteerAngles()`：调用 OSQP 求解
- `MPTOptimizer::calcMPTPoints()`：把优化变量还原成轨迹点

这份文档不是只解释“代码做了什么”，而是解释它背后的建模链条：

```text
输入轨迹和边界
  -> 参考点序列 ReferencePoint
  -> Frenet 坐标下的误差状态
  -> 线性化自行车模型
  -> 二次代价函数
  -> 线性等式/不等式约束
  -> OSQP 求解
  -> 全局 TrajectoryPoint
```

## 1. MPT 到底在优化什么

MPT 不是直接优化每个轨迹点的全局坐标：

$$
(x_i, y_i, \psi_i)
$$

而是在每个参考点的局部坐标系里，优化相对参考轨迹的误差：

$$
\mathbf{x}_i =
\begin{bmatrix}
y_i \\
\theta_i
\end{bmatrix}
$$

其中：

| 符号 | 代码变量 | 物理意义 |
|---|---|---|
| $i$ | loop index | 第 `i` 个离散参考点 |
| $N$ | `N_ref` | 参考点数量 |
| $y_i$ | `lat_error` | 第 `i` 点相对参考路径的横向偏移，左侧通常为正 |
| $\theta_i$ | `yaw_error` | 第 `i` 点相对参考路径切向的航向角误差 |
| $\delta_i$ | `steer_angles(i)` / `optimized_input` | 第 `i` 段的优化转向输入 |
| $\lambda_i$ | `slack_variables` | 软约束松弛变量，表示允许边界/碰撞约束被违反多少 |

优化后的全局轨迹点由参考点叠加误差得到：

$$
\tilde{p}_i = p_i^{ref} + y_i n_i^{ref}
$$

$$
\tilde{\psi}_i = \psi_i^{ref} + \theta_i
$$

这里：

| 符号 | 物理意义 |
|---|---|
| $p_i^{ref}$ | 参考点的全局位置 |
| $\psi_i^{ref}$ | 参考点的全局朝向 |
| $n_i^{ref}$ | 参考点局部坐标系的横向单位向量 |
| $\tilde{p}_i, \tilde{\psi}_i$ | 优化后的全局位置和朝向 |

对应代码在 `ReferencePoint::offsetDeviation()`：

```cpp
auto pose_with_deviation = autoware_utils::calc_offset_pose(pose, 0.0, lat_dev, 0.0);
pose_with_deviation.orientation =
  autoware_utils::create_quaternion_from_yaw(getYaw() + yaw_dev);
```

所以 `MPTOptimizer::calcMPTPoints()` 里的：

```cpp
const double lat_error = states(i * D_x);
const double yaw_error = states(i * D_x + 1);
traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
```

就是把局部误差还原成真实轨迹。

## 2. 为什么不直接优化全局坐标

如果直接优化全局坐标，每个点至少有 `x/y/yaw` 三类变量，车辆运动学、道路边界和碰撞约束都会变成明显非线性问题。

MPT 选择在参考路径局部坐标中优化 `lat/yaw`，有三个好处：

1. 变量维度低。
2. 横向误差和航向误差有明确物理意义。
3. 在“小误差”假设下，车辆运动学和碰撞边界都能线性化，最后可以写成凸 QP。

所谓“小误差”主要是：

$$
\sin \theta_i \approx \theta_i,\qquad
\cos \theta_i \approx 1
$$

这要求优化结果不要偏离参考路径太远。代码里也有优化结果验证：

```cpp
if (
  mpt_param_.max_validation_lat_error < std::abs(lat_error) ||
  mpt_param_.max_validation_yaw_error < std::abs(yaw_error)) {
  return std::nullopt;
}
```

## 3. `PlannerData` 到 `ReferencePoint`

`optimizeTrajectory()` 一开始做：

```cpp
const auto & p = planner_data;
const auto & traj_points = p.traj_points;
auto ref_points = calcReferencePoints(planner_data, traj_points);
```

`PlannerData` 是输入数据包：

| 字段 | 物理意义 |
|---|---|
| `traj_points` | 上游模块给出的原始/平滑轨迹 |
| `left_bound` | 可行驶区域左边界 |
| `right_bound` | 可行驶区域右边界 |
| `ego_pose` | 自车当前位姿 |
| `ego_vel` | 自车当前速度 |
| `header` | ROS 消息时间戳和 frame 信息 |

`calcReferencePoints()` 会把普通轨迹点变成包含优化信息的 `ReferencePoint`。核心步骤是：

1. 按 `mpt_param_.delta_arc_length` 重采样。
2. 在自车附近裁剪，只保留前方一定长度和后方少量历史段。
3. 用样条计算参考点 yaw 和 curvature。
4. 用上一帧结果固定车前附近点，提升连续性。
5. 计算每个参考点到左右边界的横向距离。
6. 根据车辆形状圆，计算每个碰撞检查圆对应的边界。
7. 计算每段的 `delta_arc_length`。
8. 计算 `alpha` 和 `beta`，它们后面分别用于代价函数和碰撞约束。

`ReferencePoint` 的重要字段如下：

| 字段 | 数学符号 | 物理意义 |
|---|---|---|
| `pose` | $p_i^{ref}, \psi_i^{ref}$ | 第 `i` 个参考点的位置和朝向 |
| `longitudinal_velocity_mps` | $v_i$ | 参考速度，会被复制到输出轨迹 |
| `curvature` | $\kappa_i$ | 参考路径曲率 |
| `delta_arc_length` | $\Delta s_i$ | 第 `i` 点到第 `i+1` 点的弧长间隔 |
| `alpha` | $\alpha_i$ | 优化中心前移时，前方参考切向和当前参考切向的夹角 |
| `bounds` | $[b_i^-, b_i^+]$ | 参考点中心处的左右可行横向边界 |
| `beta` | $\beta_{i,l}$ | 第 `l` 个碰撞圆对应位置的参考朝向差 |
| `bounds_on_constraints` | $[b_{i,l}^-, b_{i,l}^+]$ | 第 `l` 个碰撞圆使用的边界 |
| `fixed_kinematic_state` | $\bar{x}_i$ | 如果该点要继承上一帧结果，这里存固定状态 |
| `optimized_kinematic_state` | $x_i^\star$ | 求解后的横向/航向误差 |
| `optimized_input` | $u_i^\star$ | 求解后的转向输入 |
| `slack_variables` | $\lambda_i^\star$ | 求解后的软约束松弛量 |

## 4. Frenet 误差运动学

设参考路径按弧长 $s$ 参数化：

$$
p^{ref}(s) =
\begin{bmatrix}
x^{ref}(s) \\
y^{ref}(s)
\end{bmatrix},
\quad
\psi^{ref}(s),
\quad
\kappa(s) = \frac{d\psi^{ref}}{ds}
$$

车辆实际姿态为：

$$
p(s),\quad \psi(s)
$$

定义相对参考路径的误差：

$$
y = \text{车辆相对参考路径的横向误差}
$$

$$
\theta = \psi - \psi^{ref}
$$

这里的 $\theta$ 不是车辆全局 yaw，而是车辆 yaw 和参考路径 yaw 的差。

自行车模型的连续形式是：

$$
\dot{\psi} = \frac{v}{L}\tan\delta
$$

其中：

| 符号 | 物理意义 |
|---|---|
| $v$ | 车辆纵向速度 |
| $L$ | 轴距，对应 `vehicle_info_.wheel_base_m` |
| $\delta$ | 前轮转角 |

参考路径的航向变化是：

$$
\dot{\psi}^{ref} = \frac{d\psi^{ref}}{ds}\dot{s}
= \kappa v
$$

因此航向误差的变化为：

$$
\dot{\theta}
= \dot{\psi} - \dot{\psi}^{ref}
= \frac{v}{L}\tan\delta - \kappa v
$$

横向误差的变化为：

$$
\dot{y} = v \sin\theta
$$

在小角度假设下：

$$
\sin\theta \approx \theta
$$

于是：

$$
\dot{y} \approx v\theta
$$

把时间离散步长 $dt$ 换成弧长步长：

$$
\Delta s = v dt
$$

得到空间离散形式：

$$
y_{i+1} = y_i + \Delta s_i \theta_i
$$

$$
\theta_{i+1}
= \theta_i
+ \Delta s_i\left(\frac{\tan\delta_i}{L} - \kappa_i\right)
$$

这就是 MPT 状态方程的物理来源。

## 5. 为什么要线性化转向角

上面的式子里有：

$$
\tan\delta_i
$$

这是非线性的。QP 需要二次目标和线性约束，所以需要把它线性化。

参考路径曲率对应的参考转角是：

$$
\delta_{r,i} = \arctan(L\kappa_i)
$$

考虑物理转向极限：

$$
\bar{\delta}_{r,i}
= \mathrm{clamp}(\delta_{r,i}, -\delta_{\max}, \delta_{\max})
$$

对 $\tan\delta$ 在参考转角附近做一阶泰勒展开：

$$
\tan\delta_i
\approx
\tan\bar{\delta}_{r,i}
+ \frac{1}{\cos^2\bar{\delta}_{r,i}}
(\delta_i - \bar{\delta}_{r,i})
$$

整理为：

$$
\tan\delta_i
\approx
\frac{1}{\cos^2\bar{\delta}_{r,i}}\delta_i
+ \tan\bar{\delta}_{r,i}
- \frac{\bar{\delta}_{r,i}}{\cos^2\bar{\delta}_{r,i}}
$$

代回航向误差方程：

$$
\theta_{i+1}
= \theta_i
+ \frac{\Delta s_i}{L\cos^2\bar{\delta}_{r,i}}\delta_i
+ \Delta s_i
\left(
\frac{1}{L}\tan\bar{\delta}_{r,i}
- \frac{\bar{\delta}_{r,i}}{L\cos^2\bar{\delta}_{r,i}}
- \kappa_i
\right)
$$

于是状态方程可以写成：

$$
\mathbf{x}_{i+1}
= A_i\mathbf{x}_i + B_i u_i + W_i
$$

其中：

$$
\mathbf{x}_i =
\begin{bmatrix}
y_i \\
\theta_i
\end{bmatrix},
\quad
u_i = \delta_i
$$

$$
A_i =
\begin{bmatrix}
1 & \Delta s_i \\
0 & 1
\end{bmatrix}
$$

$$
B_i =
\begin{bmatrix}
0 \\
\dfrac{\Delta s_i}{L\cos^2\bar{\delta}_{r,i}}
\end{bmatrix}
$$

$$
W_i =
\begin{bmatrix}
0 \\
\Delta s_i
\left(
\dfrac{1}{L}\tan\bar{\delta}_{r,i}
- \dfrac{\bar{\delta}_{r,i}}{L\cos^2\bar{\delta}_{r,i}}
- \kappa_i
\right)
\end{bmatrix}
$$

对应代码在 `KinematicsBicycleModel::calculateStateEquationMatrix()`：

```cpp
Ad << 1.0, ds, 0.0, 1.0;

Bd << 0.0, ds / wheelbase_ / std::pow(std::cos(delta_r), 2.0);

Wd << 0.0, -ds * curvature + ds / wheelbase_ *
                       (std::tan(cropped_delta_r) -
                        cropped_delta_r / std::pow(std::cos(cropped_delta_r), 2.0));
```

注意一个实现细节：当前 `StateEquationGenerator::calcMatrix()` 里暂时没有把
`p.curvature` 传进去，而是传了 `0.0`：

```cpp
vehicle_model_ptr_->calculateStateEquationMatrix(Ad, Bd, Wd, 0.0, p.delta_arc_length);
```

因此当前实际用于求解的模型退化为：

$$
A_i =
\begin{bmatrix}
1 & \Delta s_i \\
0 & 1
\end{bmatrix},
\quad
B_i =
\begin{bmatrix}
0 \\
\dfrac{\Delta s_i}{L}
\end{bmatrix},
\quad
W_i =
\begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

也就是：

$$
y_{i+1} = y_i + \Delta s_i\theta_i
$$

$$
\theta_{i+1} = \theta_i + \frac{\Delta s_i}{L}u_i
$$

这点很重要：文档上的完整模型包含曲率项，但当前代码为了稳定性暂时关闭了曲率输入。

## 6. 单步状态方程如何堆成整段轨迹约束

令：

$$
X =
\begin{bmatrix}
\mathbf{x}_0 \\
\mathbf{x}_1 \\
\vdots \\
\mathbf{x}_{N-1}
\end{bmatrix}
\in \mathbb{R}^{2N}
$$

$$
U =
\begin{bmatrix}
u_0 \\
u_1 \\
\vdots \\
u_{N-2}
\end{bmatrix}
\in \mathbb{R}^{N-1}
$$

代码里：

```cpp
const size_t D_x = state_equation_generator_.getDimX(); // 2
const size_t D_u = state_equation_generator_.getDimU(); // 1

const size_t N_x = N_ref * D_x;
const size_t N_u = (N_ref - 1) * D_u;
```

`StateEquationGenerator::calcMatrix()` 构造三个大矩阵：

```cpp
Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_x, N_x);
Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N_x, N_u);
Eigen::VectorXd W = Eigen::VectorXd::Zero(N_x);
```

它们不是直接消元得到的 $X = BU + W$，而是用来表达整段的一步递推约束：

$$
X = A_{\text{blk}}X + B_{\text{blk}}U + W_{\text{blk}}
$$

其中第一行块设置成：

$$
A_{\text{blk}}(0,0)=I
$$

所以第一段约束实际是：

$$
\mathbf{x}_0 = \mathbf{x}_0
$$

它不会固定初始状态。这样做的含义是：MPT 允许轨迹优化窗口的起点不是当前自车精确位姿，初始误差也作为优化变量。如果某些点必须固定，会通过额外 fixed point constraint 加进去。

对后续点：

$$
\mathbf{x}_i
= A_{i-1}\mathbf{x}_{i-1}
+ B_{i-1}u_{i-1}
+ W_{i-1}
\quad i=1,\dots,N-1
$$

在 QP 约束里写成：

$$
(I - A_{\text{blk}})X - B_{\text{blk}}U = W_{\text{blk}}
$$

对应代码：

```cpp
A.block(0, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x, N_x) - mpt_mat.A;
A.block(0, N_x, N_x, N_u) = -mpt_mat.B;
lb.segment(0, N_x) = mpt_mat.W;
ub.segment(0, N_x) = mpt_mat.W;
```

所以状态方程在最终 QP 中是一个线性等式约束。

## 7. 最终优化变量 `v`

在 `calcObjectiveMatrix()` 和 `calcConstraintMatrix()` 里，最终决策变量是：

$$
v =
\begin{bmatrix}
X \\
U \\
\Lambda
\end{bmatrix}
$$

其中：

$$
\Lambda =
\begin{bmatrix}
\lambda_{0,0} \\
\lambda_{1,0} \\
\vdots
\end{bmatrix}
$$

是软约束松弛变量。

代码中：

```cpp
const size_t N_s = N_ref * N_slack;
const size_t N_v = N_x + N_u + N_s;
```

各块含义如下：

| 范围 | 变量 | 物理意义 |
|---|---|---|
| `[0, N_x)` | `X` | 每个点的横向误差和航向误差 |
| `[N_x, N_x + N_u)` | `U` | 每段的转向输入 |
| `[N_x + N_u, N_v)` | `S` / `Lambda` | 边界/碰撞软约束松弛变量 |

`calcMPTPoints()` 也是按这个布局拆解：

```cpp
const Eigen::VectorXd states = optimized_variables.segment(0, N_x);
const Eigen::VectorXd steer_angles = optimized_variables.segment(N_x, N_u);
const Eigen::VectorXd slack_variables =
  optimized_variables.segment(N_x + N_u, N_ref * N_slack);
```

## 8. `Q` 矩阵：状态误差惩罚

`calcValueMatrix()` 先构造状态权重矩阵 `Q`：

$$
Q =
\mathrm{diag}
(
w_{y,0}, w_{\theta,0},
w_{y,1}, w_{\theta,1},
\dots,
w_{y,N-1}, w_{\theta,N-1}
)
$$

对应代价：

$$
J_X =
\sum_{i=0}^{N-1}
\left(
w_{y,i}y_i^2 + w_{\theta,i}\theta_i^2
\right)
$$

其中：

| 权重 | 代码参数 | 物理意义 |
|---|---|---|
| $w_y$ | `lat_error_weight` | 惩罚横向偏离参考路径 |
| $w_\theta$ | `yaw_error_weight` | 惩罚航向偏离参考路径 |
| terminal weights | `terminal_*` | 窗口末端跟踪权重 |
| goal weights | `goal_*` | 如果参考点末端就是目标点，用目标点权重 |
| avoidance weights | `avoidance_*` | 避障区域使用的自适应权重 |

避障区域的权重会通过 `normalized_avoidance_cost` 插值：

$$
w_y =
\mathrm{lerp}
(
w_y^{normal},
w_y^{avoidance},
c_i
)
$$

$$
w_\theta =
\mathrm{lerp}
(
w_\theta^{normal},
w_\theta^{avoidance},
c_i
)
$$

其中：

$$
c_i = \texttt{ref\_points.at(i).normalized\_avoidance\_cost}
$$

物理上，`normalized_avoidance_cost` 越大，说明该点越靠近需要避让的窄边界或障碍区域，MPT 会改变跟踪和转向权重，让轨迹更容易离开原参考线。

## 9. `R` 矩阵：转向输入和转向变化率惩罚

`R` 对应控制输入代价：

$$
J_U =
\sum_{i=0}^{N-2} w_{\delta,i}u_i^2
+ \sum_{i=0}^{N-3} w_{\Delta\delta}(u_{i+1}-u_i)^2
$$

第一项惩罚转向输入本身，避免轨迹需要过大转角。

第二项惩罚相邻转向输入变化，避免转向突然跳变。

代码里基础转向权重来自：

```cpp
adaptive_steer_weight = lerp(
  mpt_param_.steer_input_weight,
  mpt_param_.avoidance_steer_input_weight,
  ref_points.at(i).normalized_avoidance_cost);
```

转向变化率权重由 `addSteerWeightR()` 加入：

```cpp
// weight for (u(i) - u(i-1))^2
R_triplet_vec.push_back(Eigen::Triplet<double>(i, i, w));
R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i, -w));
R_triplet_vec.push_back(Eigen::Triplet<double>(i, i + 1, -w));
R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i + 1, w));
```

因为：

$$
w(u_{i+1}-u_i)^2
=
w u_i^2 - 2w u_i u_{i+1} + w u_{i+1}^2
$$

所以它会在 `R` 里给相邻两个控制量增加：

$$
\begin{bmatrix}
w & -w \\
-w & w
\end{bmatrix}
$$

## 10. `alpha` 和优化中心前移

`calcObjectiveMatrix()` 里没有直接对 $X$ 做：

$$
X^T Q X
$$

而是先构造：

```cpp
Z = sparse_T_mat * X + T_vec
```

然后惩罚：

$$
J_Z = Z^TQZ
$$

这是为了把横向误差的惩罚点从参考点本身向车头方向前移一段：

$$
d_c = \texttt{optimization\_center\_offset}
$$

为什么要前移？因为车辆控制更关心车身前部/前轮附近是否平顺、是否贴近参考，而不只是后轴中心的误差。前视一点也会让轨迹更稳定。

对第 `i` 个点，代码构造：

```cpp
triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
triplet_T_vec.push_back(
  Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));

T_vec(i * D_x) = -offset * std::sin(alpha);
```

所以单个点的变换是：

$$
z_{y,i}
=
\cos\alpha_i\,y_i
+ d_c\cos\alpha_i\,\theta_i
- d_c\sin\alpha_i
$$

$$
z_{\theta,i} = \theta_i
$$

这里：

| 符号 | 代码变量 | 物理意义 |
|---|---|---|
| $d_c$ | `optimization_center_offset` | 优化误差评估点向前移动的距离 |
| $\alpha_i$ | `ref_points.at(i).alpha` | 从当前参考点到前方点的参考方向变化 |
| $z_{y,i}$ | `Z` 的横向分量 | 前移优化中心处的近似横向误差 |

这个式子可以从几何近似得到。

当前点横向偏移 $y_i$ 之后，再考虑车辆航向误差 $\theta_i$，前方距离 $d_c$ 的点会额外产生约：

$$
d_c\theta_i
$$

的横向偏移。参考路径本身如果已经弯曲，前方参考点方向相对当前方向有夹角 $\alpha_i$，所以需要投影到前方参考法向上，并减去参考曲线自然弯曲造成的项：

$$
-d_c\sin\alpha_i
$$

小角度线性化后就是代码里的：

$$
z_{y,i}
=
\cos\alpha_i\,y_i
+ d_c\cos\alpha_i\,\theta_i
- d_c\sin\alpha_i
$$

如果参考路径近似直线，那么 $\alpha_i \approx 0$：

$$
z_{y,i} \approx y_i + d_c\theta_i
$$

这就是最直观的前视点误差：车头附近的横向偏移约等于当前横向偏移加上航向误差造成的前方偏移。

## 11. 从 `Q/R` 到 QP 的 `H/g`

优化目标可以概括为：

$$
J(v)
=
J_Z(X) + J_U(U) + J_\lambda(\Lambda)
$$

其中：

$$
Z = T X + t
$$

所以：

$$
J_Z
=
(TX+t)^T Q (TX+t)
$$

展开：

$$
J_Z
=
X^T T^TQT X
+ 2t^TQTX
+ t^TQt
$$

常数项 $t^TQt$ 不影响最优解，可以丢掉。

代码构造：

```cpp
H_x = sparse_T_mat.transpose() * val_mat.Q * sparse_T_mat;
H.block(0, 0, N_x, N_x) = H_x;
H.block(N_x, N_x, N_u, N_u) = val_mat.R;

g.segment(0, N_x) = T_vec.transpose() * val_mat.Q * sparse_T_mat;
g.segment(N_x + N_u, N_s) =
  mpt_param_.soft_collision_free_weight * Eigen::VectorXd::Ones(N_s);
```

也就是：

$$
H =
\begin{bmatrix}
T^TQT & 0 & 0 \\
0 & R & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

$$
g =
\begin{bmatrix}
t^TQT \\
0 \\
w_\lambda \mathbf{1}
\end{bmatrix}
$$

其中 $w_\lambda$ 是：

```cpp
mpt_param_.soft_collision_free_weight
```

松弛变量没有二次项，只有线性惩罚：

$$
J_\lambda = w_\lambda \sum_j \lambda_j
$$

这表示只要碰撞/边界约束被违反，优化器就要付出代价。

严格来说，OSQP 的标准目标一般写成：

$$
\frac{1}{2}v^THv + g^Tv
$$

而代码中的 `H/g` 可能和理论展开差一个统一的常数因子。对同一子目标同时缩放通常不改变该子目标的极小点；实际权重参数是按这份实现调出来的，所以理解时以代码矩阵为准。

## 12. 约束 1：状态方程等式约束

最终 QP 约束形式是：

$$
l \le A_{qp}v \le u
$$

第一块约束来自车辆运动学：

$$
(I - A_{\text{blk}})X - B_{\text{blk}}U = W_{\text{blk}}
$$

写进 QP：

$$
\begin{bmatrix}
I - A_{\text{blk}} & -B_{\text{blk}} & 0
\end{bmatrix}
\begin{bmatrix}
X \\
U \\
\Lambda
\end{bmatrix}
=
W_{\text{blk}}
$$

所以：

$$
l_{\text{dyn}} = W_{\text{blk}},
\quad
u_{\text{dyn}} = W_{\text{blk}}
$$

这就是等式约束。

物理意义：所有优化出来的横向误差和航向误差，必须能由线性化自行车模型和转向输入串起来，不能每个点各自乱动。

## 13. 约束 2：车辆边界和碰撞约束

MPT 用多个圆近似车辆形状。每个圆有：

| 符号 | 代码变量 | 物理意义 |
|---|---|---|
| $l$ | `vehicle_circle_longitudinal_offsets_[l_idx]` | 该圆心相对 `base_link` 的纵向偏移 |
| $r$ | `vehicle_circle_radiuses_[l_idx]` | 该圆的半径 |
| $\beta_{i,l}$ | `ref_points.at(i).beta.at(l_idx)` | 当前参考点朝向与圆心位置参考朝向的差 |

对第 `i` 个参考点、第 `l` 个车辆圆，圆心横向位置近似为：

$$
y_{i,l}^{circle}
\approx
\cos\beta_{i,l}\,y_i
+ l\cos\beta_{i,l}\,\theta_i
+ l\sin\beta_{i,l}
$$

代码构造：

```cpp
C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x, std::cos(beta)));
C_triplet_vec.push_back(
  Eigen::Triplet<double>(i, i * D_x + 1, lon_offset * std::cos(beta)));
C_vec(i) = lon_offset * std::sin(beta);
```

也就是：

$$
y_{i,l}^{circle} = C_{i,l}X + c_{i,l}
$$

其中：

$$
C_{i,l}
=
\begin{bmatrix}
\cdots & \cos\beta_{i,l} & l\cos\beta_{i,l} & \cdots
\end{bmatrix}
$$

边界要求圆心在可行范围内：

$$
b_{i,l}^- \le y_{i,l}^{circle} \le b_{i,l}^+
$$

其中 $b_{i,l}^-$ 和 $b_{i,l}^+$ 来自：

```cpp
const auto & [part_ub, part_lb] = extractBounds(ref_points, l_idx, bounds_offset);
```

`bounds_offset` 的作用是把“车辆中心线可行边界”转换成“某个圆心可行边界”：

```cpp
const double bounds_offset =
  vehicle_info_.vehicle_width_m / 2.0 - vehicle_circle_radiuses_.at(l_idx);
```

如果使用软约束，就允许违反边界，但要引入松弛变量：

$$
b_{i,l}^- - \lambda_{i,l}
\le
y_{i,l}^{circle}
\le
b_{i,l}^+ + \lambda_{i,l}
$$

$$
\lambda_{i,l} \ge 0
$$

为了写成 OSQP 的 `lb <= A v <= ub`，代码把它改写成三组下界约束：

第一组，下边界：

$$
C_{i,l}X + \lambda_{i,l}
\ge
b_{i,l}^- - c_{i,l}
$$

第二组，上边界：

$$
-C_{i,l}X + \lambda_{i,l}
\ge
c_{i,l} - b_{i,l}^+
$$

第三组，松弛变量非负：

$$
\lambda_{i,l} \ge 0
$$

对应代码：

```cpp
A_blk.block(0, 0, N_ref, N_x) = C_sparse_mat;
A_blk.block(N_ref, 0, N_ref, N_x) = -C_sparse_mat;

A_blk.block(0, local_A_offset_cols, N_ref, N_ref) = I;
A_blk.block(N_ref, local_A_offset_cols, N_ref, N_ref) = I;
A_blk.block(2 * N_ref, local_A_offset_cols, N_ref, N_ref) = I;

lb_blk.segment(0, N_ref) = -C_vec + part_lb;
lb_blk.segment(N_ref, N_ref) = C_vec - part_ub;
```

这就是为什么软约束每个圆会增加 `3 * N_ref` 行约束。

如果启用 hard constraint，则直接写成：

$$
b_{i,l}^- - c_{i,l}
\le
C_{i,l}X
\le
b_{i,l}^+ - c_{i,l}
$$

## 14. `l_inf_norm` 对 slack 的影响

`getNumberOfSlackVariables()` 里：

```cpp
if (mpt_param_.soft_constraint) {
  if (mpt_param_.l_inf_norm) {
    return 1;
  }
  return vehicle_circle_longitudinal_offsets_.size();
}
return 0;
```

普通模式下，每个车辆圆都有自己的 slack：

$$
\lambda_{i,0}, \lambda_{i,1}, \dots, \lambda_{i,L-1}
$$

如果开启 `l_inf_norm`，每个参考点只用一个共享 slack：

$$
\lambda_i
$$

这表示该点所有车辆圆共享同一个最大违反量，更接近：

$$
\max_l \text{violation}_{i,l}
$$

的惩罚形式。

## 15. 约束 3：固定点约束

为了防止每一帧规划结果在车头附近跳变，MPT 会把一部分前方点固定到上一帧优化结果。

如果某个 `ReferencePoint` 里有：

```cpp
fixed_kinematic_state
```

就添加约束：

$$
\mathbf{x}_i = \bar{\mathbf{x}}_i
$$

其中：

$$
\bar{\mathbf{x}}_i =
\begin{bmatrix}
\bar{y}_i \\
\bar{\theta}_i
\end{bmatrix}
$$

对应代码：

```cpp
A.block(A_rows_end, D_x * i, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

lb.segment(A_rows_end, D_x) =
  ref_points.at(i).fixed_kinematic_state->toEigenVector();
ub.segment(A_rows_end, D_x) =
  ref_points.at(i).fixed_kinematic_state->toEigenVector();
```

物理意义：这些点的横向误差和航向误差必须沿用上一帧，使轨迹连续，避免自车附近路径忽左忽右。

## 16. 约束 4：转向角限制

如果启用 `steer_limit_constraint`，会加入：

$$
u_i^{min} \le u_i \le u_i^{max}
$$

代码里：

```cpp
const double ref_steer_angle =
  std::atan2(vehicle_info_.wheel_base_m * ref_points.at(i).curvature, 1.0);
lb(A_rows_end + i) = ref_steer_angle - mpt_param_.max_steer_rad;
ub(A_rows_end + i) = ref_steer_angle + mpt_param_.max_steer_rad;
```

也就是：

$$
\delta_i^{ref} - \delta_{\max}
\le
u_i
\le
\delta_i^{ref} + \delta_{\max}
$$

其中：

$$
\delta_i^{ref} = \arctan(L\kappa_i)
$$

从物理上看，这限制了优化转向输入不要偏离参考曲率对应的转向太多。当前状态方程里曲率项暂时被关掉，但转向约束仍然使用参考曲率计算上下界。

## 17. QP 标准形式和代码对象对照

最终问题是：

$$
\min_v
\frac{1}{2}v^THv + g^Tv
$$

满足：

$$
l \le A_{qp}v \le u
$$

代码对象对照：

| 代码对象 | 数学符号 | 含义 |
|---|---|---|
| `optimized_variables` | $v$ | OSQP 求出来的完整决策变量 |
| `obj_mat.hessian` | $H$ | 二次项矩阵 |
| `obj_mat.gradient` | $g$ | 一次项向量 |
| `const_mat.linear` | $A_{qp}$ | 约束矩阵 |
| `const_mat.lower_bound` | $l$ | 约束下界 |
| `const_mat.upper_bound` | $u$ | 约束上界 |
| `mpt_mat.A` | $A_{\text{blk}}$ | 整段状态递推矩阵 |
| `mpt_mat.B` | $B_{\text{blk}}$ | 整段控制输入矩阵 |
| `mpt_mat.W` | $W_{\text{blk}}$ | 整段状态方程偏置 |
| `val_mat.Q` | $Q$ | 横向/航向误差权重 |
| `val_mat.R` | $R$ | 转向/转向变化率权重 |

`calcOptimizedSteerAngles()` 里将矩阵转换成 OSQP 使用的 CSC 格式：

```cpp
const autoware::osqp_interface::CSC_Matrix P_csc =
  autoware::osqp_interface::calCSCMatrixTrapezoidal(H);
const autoware::osqp_interface::CSC_Matrix A_csc =
  autoware::osqp_interface::calCSCMatrix(A);
```

然后调用：

```cpp
const autoware::osqp_interface::OSQPResult osqp_result =
  osqp_solver_ptr_->optimize();
```

## 18. Warm start 的数学意义

MPT 有两层 warm start。

第一层是 OSQP solver 复用：

```cpp
if (
  prev_solution_status_ == 1 &&
  mpt_param_.enable_warm_start &&
  prev_mat_n_ == H.rows() &&
  prev_mat_m_ == A.rows()) {
  osqp_solver_ptr_->updateCscP(P_csc);
  osqp_solver_ptr_->updateQ(f);
  osqp_solver_ptr_->updateCscA(A_csc);
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
}
```

如果上一帧求解成功，并且矩阵尺寸没变，就更新矩阵数据而不是重新创建 solver。

第二层是 manual warm start。它构造上一帧解作为初值：

$$
v = u_0 + \Delta v
$$

然后优化变量改成 $\Delta v$。

原问题：

$$
\min_v \frac{1}{2}v^THv + g^Tv
$$

令：

$$
v = u_0 + \Delta v
$$

展开：

$$
\frac{1}{2}(u_0+\Delta v)^TH(u_0+\Delta v)
+ g^T(u_0+\Delta v)
$$

去掉常数项后：

$$
\min_{\Delta v}
\frac{1}{2}\Delta v^TH\Delta v
+ (g + Hu_0)^T\Delta v
$$

所以代码更新：

```cpp
f += H * *u0;
```

约束也要变换：

$$
l \le A(u_0+\Delta v) \le u
$$

等价于：

$$
l - Au_0 \le A\Delta v \le u - Au_0
$$

对应代码：

```cpp
const Eigen::VectorXd A_times_u0 = A * *u0;
ub -= A_times_u0;
lb -= A_times_u0;
```

求解完成后再加回初值：

```cpp
return optimized_variables + *u0;
```

实现提醒：上面的推导要求 `u0` 和当前 QP 决策变量 `v=[X,U,\Lambda]` 同维度。当前源码里的
`calcInitialSolutionForManualWarmStart()` 使用的是 `D_x + N_u + N_ref * N_slack` 形式，更像旧版
`[x_0, U, \Lambda]` 的 reduced formulation；而当前 `H/A` 是按 `[X, U, \Lambda]` 构造的。如果开启
`enable_manual_warm_start`，需要确认这块在你的分支里是否已经同步修正，否则会有维度不一致风险。

## 19. 解出来之后如何回到轨迹

OSQP 成功后返回：

$$
v^\star =
\begin{bmatrix}
X^\star \\
U^\star \\
\Lambda^\star
\end{bmatrix}
$$

`calcMPTPoints()` 遍历每个参考点：

```cpp
const double lat_error = states(i * D_x);
const double yaw_error = states(i * D_x + 1);
```

并保存回 `ReferencePoint`：

```cpp
ref_point.optimized_kinematic_state =
  KinematicState{lat_error, yaw_error};
ref_point.optimized_input = steer_angles(i * D_u);
ref_point.slack_variables = tmp_slack_variables;
```

然后还原全局轨迹：

```cpp
traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;
```

数学上就是：

$$
\tilde{p}_i
=
p_i^{ref} + y_i^\star n_i^{ref}
$$

$$
\tilde{\psi}_i
=
\psi_i^{ref} + \theta_i^\star
$$

速度不优化，直接继承：

$$
\tilde{v}_i = v_i^{ref}
$$

## 20. 变量传递总表

| 阶段 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `optimizeTrajectory()` | `PlannerData` | `mpt_traj_points` | MPT 总入口 |
| `calcReferencePoints()` | `traj_points`, bounds, ego | `ref_points` | 构造参考路径、边界、曲率、固定点信息 |
| `state_equation_generator_.calcMatrix()` | `ref_points` | `mpt_mat` | 构造状态递推矩阵 |
| `calcValueMatrix()` | `ref_points`, `traj_points` | `val_mat.Q/R` | 构造状态和输入权重 |
| `calcObjectiveMatrix()` | `mpt_mat`, `val_mat`, `ref_points` | `obj_mat.H/g` | 构造 QP 目标 |
| `calcConstraintMatrix()` | `mpt_mat`, `ref_points` | `const_mat.A/lb/ub` | 构造运动学、碰撞、固定点、转向约束 |
| `calcOptimizedSteerAngles()` | `obj_mat`, `const_mat` | `optimized_variables` | 调 OSQP 求解 |
| `calcMPTPoints()` | `ref_points`, `optimized_variables` | `TrajectoryPoint[]` | 局部误差还原成全局轨迹 |
| `publishDebugTrajectories()` | `ref_points`, `mpt_traj_points` | debug topic | 发布调试轨迹 |
| 保存上一帧 | `ref_points`, `mpt_traj_points` | `prev_*` | 下一帧固定点和 warm start 使用 |

## 21. 一句话总结

MPT 的核心不是“把轨迹点简单平滑一下”，而是：

1. 把输入轨迹变成带边界、曲率、车辆几何信息的参考点序列。
2. 在参考点局部坐标中优化横向误差 $y_i$ 和航向误差 $\theta_i$。
3. 用线性化自行车模型把相邻误差状态串起来。
4. 用二次代价惩罚横向误差、航向误差、转向输入和转向变化。
5. 用线性约束处理车辆运动学、道路边界、碰撞圆、固定点和转向角限制。
6. 用 slack 把部分碰撞/边界约束软化，避免问题轻易 infeasible。
7. 最后把优化出的局部误差叠加回参考点，生成全局 `TrajectoryPoint`。

所以它本质上是一个“**参考路径局部坐标系下，带车辆运动学和边界软约束的轨迹 QP 优化器**”。
