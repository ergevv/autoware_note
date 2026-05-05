# Elastic Band 数学原理说明

这份说明对应 `elastic_band.cpp` 中两段核心实现：

- `makePMatrix()`：构造平滑项的二次型矩阵
- `EBPathSmoother::updateConstraint()`：把轨迹平滑问题写成标准 QP

重点分析的是 `/planning/autoware_path_smoother/src/elastic_band.cpp` 里 `343-434` 行附近的实现，以及它和 `36-71` 行 `makePMatrix()` 的数学对应关系。

## 1. 这段代码到底在优化什么

它优化的不是每个点的完整二维坐标 `(x_i, y_i)`，而是每个采样点沿着路径**法向方向**的横向偏移量：

$$
d_i \in \mathbb{R}, \quad i=0,\dots,N-1
$$

其中：

$$
p_i =
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
$$

是输入轨迹的第 `i` 个点，航向角为 `\theta_i`。定义：

$$
t_i =
\begin{bmatrix}
\cos\theta_i \\
\sin\theta_i
\end{bmatrix},
\quad
n_i =
\begin{bmatrix}
-\sin\theta_i \\
\cos\theta_i
\end{bmatrix}
$$

其中 `t_i` 是切向单位向量，`n_i` 是法向单位向量。

优化后的点不是任意移动，而是只允许在法向上平移：

$$
\tilde{p}_i = p_i + d_i n_i
$$

这有两个很重要的含义：

1. 优化变量从二维点坐标降成了一维横向偏移，维度从 `2N` 降到 `N`。
2. 切向方向不动，只在横向“抻一抻、拉一拉”，这很符合路径平滑的需求。

从数学上说，这也是一次**线性化**：每次求解时，法向 `n_i` 都由输入轨迹的朝向预先固定，因此 `\tilde{p}_i` 对 `d_i` 是线性的，整个问题就能写成凸 QP。

## 2. 它被写成了什么标准问题

OSQP 求解的标准形式是：

$$
\min_{d}
\frac{1}{2} d^\top P d + q^\top d
$$

满足：

$$
l \le A d \le u
$$

在这段代码里，优化变量就是：

$$
d =
\begin{bmatrix}
d_0 & d_1 & \cdots & d_{N-1}
\end{bmatrix}^\top
$$

而约束矩阵直接取成单位阵：

$$
A = I
$$

因此约束退化成最简单的**盒约束**：

$$
l_i \le d_i \le u_i
$$

也就是“每个点最多能向左/向右偏多少”。

## 3. 约束项的数学意义

`updateConstraint()` 里先构造：

```cpp
const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(p.num_points, p.num_points);
```

这表示每个决策变量 `d_i` 直接受上下界限制。

随后代码根据点的角色，给每个 `d_i` 设置不同的允许范围：

$$
-c_i \le d_i \le c_i
$$

其中 `c_i` 由下面几类情况决定：

- 第一个点使用 `clearance_for_fix`
- 如果终点包含在当前窗口内，那么最后两个点使用 `clearance_for_fix`
- 前面若干个连接点使用 `clearance_for_joint`
- 其他点使用 `clearance_for_smooth`

所以本质上是：

$$
l_i = -c_i,\quad u_i = c_i
$$

### 3.1 为什么第一个点要固定或近似固定

第一个点通常继承自上一轮优化结果，用来保证当前平滑结果和上一帧结果连续，不会每次重规划都在车头附近突然跳一下。

如果 `clearance_for_fix = 0`，那就是严格固定：

$$
d_0 = 0
$$

如果它大于 0，那就不是“完全固定”，而是“只能在很小范围内移动”。

### 3.2 为什么终点附近要固定两个点

代码里在终点包含于当前窗口时，会把末端点及其前一个点也设成 `clearance_for_fix`。原因不是只为了位置，还为了**保持终点朝向**。

因为最终朝向不是作为优化变量直接求出来的，而是在 `convertOptimizedPointsToTrajectory()` 之后通过相邻点重新插入的。如果只固定最后一个点，不固定倒数第二个点，那么最后一段线段的方向仍然可能改变，终点姿态就不稳。

### 3.3 `pad_start_idx` 的作用

在轨迹点不足 `num_points` 时，代码会把最后一个点重复填充到固定长度。这样做之后，那些“补出来”的点不能再随便移动，否则会在末端制造假形状，所以也要通过约束把它们锁住。

## 4. 为什么 `sparse_theta_mat` 表示“法向投影”

代码构造了一个矩阵 `sparse_theta_mat`，这里记作 `T`。

先把原始二维坐标堆成一个大向量：

$$
X =
\begin{bmatrix}
x_0 & x_1 & \cdots & x_{N-1} &
y_0 & y_1 & \cdots & y_{N-1}
\end{bmatrix}^\top
\in \mathbb{R}^{2N}
$$

代码里就是：

```cpp
x_mat(i) = traj_points.at(i).pose.position.x;
x_mat(i + p.num_points) = traj_points.at(i).pose.position.y;
```

而矩阵 `T` 的第 `i` 行只有两个非零元：

$$
T_{i,i} = -\sin\theta_i,\qquad
T_{i,i+N} = \cos\theta_i
$$

所以第 `i` 行乘上 `X` 得到：

$$
(TX)_i = -\sin\theta_i \, x_i + \cos\theta_i \, y_i = n_i^\top p_i
$$

也就是点 `p_i` 在法向方向上的标量投影。

更重要的是：

$$
T^\top d =
\begin{bmatrix}
-\sin\theta_0 d_0 \\
\vdots \\
-\sin\theta_{N-1} d_{N-1} \\
\cos\theta_0 d_0 \\
\vdots \\
\cos\theta_{N-1} d_{N-1}
\end{bmatrix}
$$

这正好就是所有点沿法向位移后在全局 `x/y` 上产生的改变量。因此优化后的堆叠坐标可以写成：

$$
\tilde{X} = X + T^\top d
$$

这一步是整段代码最关键的建模思想。

## 5. `makePMatrix()` 到底在惩罚什么

`makePMatrix()` 构造出的矩阵记作 `H`。它是一个 `2N \times 2N` 的块对角矩阵：

$$
H =
\begin{bmatrix}
B & 0 \\
0 & B
\end{bmatrix}
$$

其中 `B` 的结构是：

$$
B =
\begin{bmatrix}
1 & -2 & 1 & 0 & \cdots & 0 \\
-2 & 5 & -4 & 1 & \cdots & 0 \\
1 & -4 & 6 & -4 & \ddots & \vdots \\
0 & 1 & -4 & 6 & \ddots & 0 \\
\vdots & \ddots & \ddots & \ddots & 5 & -2 \\
0 & \cdots & 0 & 1 & -2 & 1
\end{bmatrix}
$$

这不是随便写出来的，它正好等于二阶差分矩阵 `D_2` 的 Gram 矩阵：

$$
B = D_2^\top D_2
$$

其中：

$$
D_2 =
\begin{bmatrix}
1 & -2 & 1 & 0 & \cdots & 0 \\
0 & 1 & -2 & 1 & \cdots & 0 \\
\vdots & & \ddots & \ddots & \ddots & \vdots \\
0 & \cdots & 0 & 1 & -2 & 1
\end{bmatrix}
$$

于是对任意坐标序列 `x=[x_0,\dots,x_{N-1}]^\top`，有：

$$
x^\top B x = x^\top D_2^\top D_2 x = \| D_2 x \|^2
$$

同理对 `y` 也成立。因此：

$$
\tilde{X}^\top H \tilde{X}
= \|D_2 \tilde{x}\|^2 + \|D_2 \tilde{y}\|^2
$$

也就是说，它惩罚的是每个点坐标序列的**二阶差分**：

$$
\tilde{x}_{i-1} - 2\tilde{x}_i + \tilde{x}_{i+1},\qquad
\tilde{y}_{i-1} - 2\tilde{y}_i + \tilde{y}_{i+1}
$$

二阶差分越小，曲线越平顺，弯折越少。

### 5.1 它和曲率有什么关系

如果轨迹按固定弧长 `\Delta s` 重采样，那么：

$$
\tilde{p}''(s_i) \approx
\frac{\tilde{p}_{i-1} - 2\tilde{p}_i + \tilde{p}_{i+1}}{\Delta s^2}
$$

也就是说，二阶差分本质上是二阶导数的离散近似。对于按弧长参数化的平面曲线，二阶导数的大小和曲率密切相关，所以这个平滑项本质上是在惩罚一种**离散弯曲能量**。

这也解释了前面为什么一定要把轨迹按固定 `delta_arc_length` 重采样：如果点间距不均匀，同一套二阶差分矩阵就不再对应统一的导数近似，数值稳定性会明显变差。

### 5.2 一个容易误解的点

虽然名字叫 Elastic Band，但这段实现的核心惩罚并不是经典“相邻点弹簧长度”的一阶项，而更接近“带子弯曲能量”的二阶项。直观上它确实像一根被拉直的带子，但数学上更像是在最小化离散曲线的弯曲程度。

## 6. 从“平滑优化后的坐标”推到代码里的 `P` 和 `q`

平滑项如果直接写在优化后坐标 `\tilde{X}` 上，就是：

$$
J_{\text{smooth}}(d)
= \frac{w_s}{2}\tilde{X}^\top H \tilde{X}
= \frac{w_s}{2}(X + T^\top d)^\top H (X + T^\top d)
$$

其中 `w_s = smooth_weight`。

把它展开：

$$
J_{\text{smooth}}(d)
= \frac{1}{2} d^\top (w_s T H T^\top) d
+ (w_s T H X)^\top d
+ \text{const}
$$

这里用到了 `H` 是对称矩阵。

因此：

$$
P_{\text{smooth}} = w_s T H T^\top
$$

$$
q_{\text{smooth}} = w_s T H X
$$

这恰好对应代码：

```cpp
const Eigen::SparseMatrix<double> raw_P_for_smooth = p.smooth_weight * makePMatrix(p.num_points);
const Eigen::MatrixXd theta_P_mat = sparse_theta_mat * raw_P_for_smooth;
const Eigen::MatrixXd P_for_smooth = theta_P_mat * sparse_theta_mat.transpose();
const Eigen::VectorXd raw_q_for_smooth = theta_P_mat * x_mat;
```

也就是：

$$
\texttt{raw\_P\_for\_smooth} = w_s H
$$

$$
\texttt{theta\_P\_mat} = T(w_s H)
$$

$$
\texttt{P\_for\_smooth} = T(w_s H)T^\top = w_s THT^\top
$$

$$
\texttt{raw\_q\_for\_smooth} = T(w_s H)X = w_s THX
$$

## 7. 为什么还要再加一个 `lat_error_weight * I`

代码又加了一项：

```cpp
const Eigen::MatrixXd P_for_lat_error =
  p.lat_error_weight * Eigen::MatrixXd::Identity(p.num_points, p.num_points);
```

这表示额外加入了一个横向偏移惩罚：

$$
J_{\text{lat}}(d) = \frac{w_l}{2} d^\top I d = \frac{w_l}{2}\sum_{i=0}^{N-1} d_i^2
$$

其中 `w_l = lat_error_weight`。

它的作用是防止优化过度追求平滑，结果把整条路径整体向一边推走。这个项会把解拉回原始参考轨迹附近。

所以总目标函数是：

$$
J(d)
= \frac{1}{2} d^\top \left(w_s THT^\top + w_l I\right)d
+ \left(w_s THX\right)^\top d
$$

于是最终：

$$
P = w_s THT^\top + w_l I
$$

$$
q = w_s THX
$$

这正对应：

```cpp
const Eigen::MatrixXd P = P_for_smooth + P_for_lat_error;
const auto q = toStdVector(raw_q_for_smooth);
```

## 8. 求解结束后，结果怎么还原成路径点

OSQP 解出来的是：

$$
d^\star = [d_0^\star,\dots,d_{N-1}^\star]^\top
$$

它不是新的 `x/y` 坐标，而是每个点的横向偏移。

在 `convertOptimizedPointsToTrajectory()` 中，代码执行：

```cpp
eb_traj_point.pose = autoware_utils::calc_offset_pose(
  eb_traj_point.pose, 0.0, lat_offset, 0.0);
```

这里 `(0.0, lat_offset, 0.0)` 是在该点自身局部坐标系中的位移，即：

- 纵向偏移为 0
- 横向偏移为 `d_i`
- 高度偏移为 0

由于局部 `y` 轴就是法向方向，所以这一步正好实现：

$$
\tilde{p}_i = p_i + d_i^\star n_i
$$

然后再重新插入朝向，使姿态和更新后的几何形状一致。

## 9. 这段实现为什么是凸的、而且比较稳定

它能稳定地交给 OSQP，有几个关键原因：

1. 变量只有 `d_i`，维度低。
2. 约束是线性的盒约束。
3. `H = D_2^\top D_2` 半正定，`w_l I` 进一步增加了正定性。
4. 法向 `n_i` 在一次求解中固定，因此 `\tilde{X} = X + T^\top d` 是线性的。

所以整个问题是标准凸二次规划，不会出现局部极小值乱跳的问题。

## 10. 与代码逐项对照

| 代码对象 | 数学含义 |
|---|---|
| `optimized_points` | 横向偏移向量 `d` |
| `A = Identity(...)` | 盒约束矩阵 `A = I` |
| `lower_bound`, `upper_bound` | 每个 `d_i` 的上下界 `l_i, u_i` |
| `x_mat` | 原始坐标堆叠向量 `X` |
| `sparse_theta_mat` | 法向投影/位移矩阵 `T` |
| `makePMatrix(num_points)` | 二阶差分能量矩阵 `H = blkdiag(D_2^\top D_2, D_2^\top D_2)` |
| `P_for_smooth` | `w_s THT^\top` |
| `P_for_lat_error` | `w_l I` |
| `q` | `w_s THX` |
| `calc_offset_pose(..., 0.0, lat_offset, 0.0)` | `\tilde{p}_i = p_i + d_i n_i` |

## 11. 一句话总结

这段 Elastic Band 的核心不是“直接改二维轨迹点”，而是：

1. 固定每个点的切向/法向坐标系；
2. 只优化法向偏移 `d_i`；
3. 用二阶差分惩罚优化后曲线的弯曲程度；
4. 用盒约束限制每个点最多能横向移动多少；
5. 最后再把 `d_i` 还原成真实的世界坐标轨迹。

所以它本质上是一个“**固定参考朝向下的横向偏移平滑 QP**”，而 `343-434` 行正是把这个几何问题翻译成矩阵形式的地方。
