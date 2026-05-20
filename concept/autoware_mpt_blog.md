参考：autoware、ChatGPT

# 从零理解 Autoware 的 MPT 轨迹优化

这篇文章讲的是 Autoware path optimizer 里的 MPT，也就是 Model Predictive Trajectory。它不是一个全局路径搜索算法，而是一个局部轨迹优化算法：上游已经给出一条可行驶参考路径、左右边界和速度信息，MPT 在自车附近截取一段局部窗口，在车辆运动学、道路边界、障碍边界和平滑性之间做权衡，输出一条更平滑、更符合车辆运动学、更不容易碰撞的轨迹。


## 1. MPT 到底在优化什么

自动驾驶规划链路里，上游通常已经有一条路线或轨迹。它可能来自行为规划、车道中心线、避障初步结果或其他路径平滑器。可是这条输入路径不一定满足几个重要要求：

- 车辆可能无法按照这条路径的姿态变化执行，因为车辆有轴距、转向角和曲率限制。
- 路径点可能贴近道路边界或障碍边界，车辆真实车身会越界。
- 路径点之间的航向、曲率或转向变化可能不够平滑，控制器跟踪时会抖。
- 每一帧输入路径都会变化，如果直接重新输出，轨迹会在自车附近跳动。

MPT 的思路是：不直接把输入路径当成最终轨迹，而是把它当成一条参考路径。优化器允许车辆相对参考路径产生横向误差和航向误差，但这些误差要满足车辆运动学和边界约束，同时还要被代价函数惩罚。

直观地说，MPT 每一帧都在解这样一个问题：

$$
\text{在未来一小段距离内，找到一组横向偏移、航向偏移和转向角，使轨迹既平滑又安全。}
$$

这里有一个重要点：MPT 的预测维度主要沿路径弧长 $s$ 离散，而不是直接沿时间 $t$ 离散。所以它更像一个空间域的模型预测轨迹优化器。

## 2. 先把全局路径变成局部参考线

MPT 不会一次优化完整全局路径。它只优化自车附近的一段窗口。假设优化点数为 $N$，离散弧长间隔为 $\Delta s$，那么前向优化视野大约是：

$$
L_\text{horizon}=N\Delta s
$$

例如 $N=100$，$\Delta s=1\ \mathrm{m}$，优化器关注前方约 $100\ \mathrm{m}$。

参考线通常要经历几个处理：

1. 按固定弧长 $\Delta s$ 重采样，使相邻点间距稳定。
2. 以自车为中心裁剪，只保留前方优化视野和少量后方轨迹。
3. 用样条插值重新计算航向角和曲率。
4. 计算每个参考点到左右边界的横向距离。
5. 在车辆前后多个圆形碰撞检查点上插值边界。
6. 计算优化需要的额外几何量，比如 $\alpha$、$\beta$、避障代价和固定点信息。

这里的“参考点”不只是一个普通路径点。它可以理解成一个带有丰富附加信息的局部坐标系：

$$
r_i=(p_i^\text{ref},\psi_i^\text{ref},\kappa_i,\Delta s_i,\text{bounds}_i,\alpha_i,\beta_i,\cdots)
$$

其中 $p_i^\text{ref}$ 是参考点位置，$\psi_i^\text{ref}$ 是参考线切线方向，$\kappa_i$ 是参考曲率，$\Delta s_i$ 是到下一个参考点的弧长。

为什么要用样条重新计算航向和曲率？因为输入点的姿态可能不够连续，离散差分也容易受点间距影响。MPT 的车辆模型和边界约束都依赖参考线方向，如果参考方向抖动，优化结果也会抖。

## 3. 用参考线局部坐标描述车辆状态

MPT 不直接优化全局坐标 $(x,y,\psi)$。它在每个参考点上定义局部误差状态：

$$
x_i=
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix}
$$

其中：

- $l_i$ 是横向误差，也就是车辆基准点相对参考点沿左法向的偏移。
- $\theta_i$ 是航向误差，也就是车辆航向相对参考线切线方向的偏差。

参考线切向和左法向可以写成：

$$
t_i=
\begin{bmatrix}
\cos\psi_i^\text{ref}\\
\sin\psi_i^\text{ref}
\end{bmatrix},
\qquad
n_i=
\begin{bmatrix}
-\sin\psi_i^\text{ref}\\
\cos\psi_i^\text{ref}
\end{bmatrix}
$$

如果优化得到 $l_i$ 和 $\theta_i$，最终轨迹点大致可以恢复为：

$$
p_i^\text{veh}=p_i^\text{ref}+l_i n_i
$$

$$
\psi_i^\text{veh}=\psi_i^\text{ref}+\theta_i
$$

这样做的好处是非常明显的。道路边界、车道中心线和避障边界天然都是“相对参考线左右多少米”的概念，用 $l_i$ 描述比直接优化全局 $x,y$ 更自然。

## 4. 自行车模型：为什么有 $\dot\psi=\frac{v}{L}\tan\delta$

MPT 使用车辆运动学模型来约束相邻参考点之间的状态变化。最常见的是运动学自行车模型。

车辆轴距为 $L$，前轮转角为 $\delta$。在低速或忽略轮胎侧偏时，车辆可以看作绕瞬时圆心运动。后轴中心的运动半径记为 $R$。由几何关系：

$$
\tan\delta=\frac{L}{R}
$$

所以：

$$
R=\frac{L}{\tan\delta}
$$

车辆沿圆弧运动，横摆角速度等于线速度除以半径：

$$
\dot\psi=\frac{v}{R}
$$

代入 $R$：

$$
\dot\psi=\frac{v}{L}\tan\delta
$$

这就是自行车模型中航向角变化率的来源。它表达的是：转角越大，曲率越大；轴距越长，同样转角下转弯越慢。

车辆自身的曲率为：

$$
\kappa_\text{veh}=\frac{\dot\psi}{v}=\frac{\tan\delta}{L}
$$

## 5. 从时间域模型变成路径误差模型

MPT 关心的是车辆相对参考路径的误差如何沿弧长 $s$ 变化。设参考路径曲率为 $\kappa_\text{ref}$，车辆航向误差为：

$$
\theta=\psi_\text{veh}-\psi_\text{ref}
$$

参考路径自身沿弧长的航向变化为：

$$
\frac{d\psi_\text{ref}}{ds}=\kappa_\text{ref}
$$

车辆航向沿自身行驶距离的变化近似为：

$$
\frac{d\psi_\text{veh}}{ds}\approx \frac{\tan\delta}{L}
$$

因此航向误差的空间导数为：

$$
\frac{d\theta}{ds}
=\frac{d\psi_\text{veh}}{ds}-\frac{d\psi_\text{ref}}{ds}
=\frac{\tan\delta}{L}-\kappa_\text{ref}
$$

横向误差 $l$ 的变化来自车辆航向和参考线方向的夹角。如果车辆比参考线向左偏一个小角度 $\theta$，沿路径前进 $ds$ 后，横向误差会增加约 $ds\sin\theta$：

$$
\frac{dl}{ds}=\sin\theta
$$

在小角度近似下：

$$
\sin\theta\approx\theta
$$

所以得到线性误差模型的基础形式：

$$
\frac{dl}{ds}=\theta
$$

$$
\frac{d\theta}{ds}=\frac{\tan\delta}{L}-\kappa_\text{ref}
$$

这两个式子非常重要。它们说明：

- 横向误差的变化由航向误差累积出来。
- 航向误差的变化由车辆曲率和参考曲率的差决定。
- 控制量 $\delta$ 通过 $\tan\delta/L$ 影响航向误差。

## 6. 离散化：把连续模型写成一步状态方程

MPT 在参考线上取离散点。令相邻点之间的弧长为 $\Delta s_i$。使用前向欧拉离散：

$$
l_{i+1}=l_i+\Delta s_i\theta_i
$$

$$
\theta_{i+1}=\theta_i+\Delta s_i\left(\frac{\tan\delta_i}{L}-\kappa_i\right)
$$

如果直接使用 $\tan\delta_i$，问题会变成非线性优化。为了保持二次规划，需要把它线性化。

参考曲率 $\kappa_i$ 对应的理想转角为：

$$
\delta_r=\arctan(L\kappa_i)
$$

因为理想情况下：

$$
\frac{\tan\delta_r}{L}=\kappa_i
$$

在 $\delta_r$ 附近对 $\tan\delta$ 做一阶泰勒展开：

$$
\tan\delta\approx \tan\delta_r+\frac{1}{\cos^2\delta_r}(\delta-\delta_r)
$$

展开整理：

$$
\tan\delta
\approx
\frac{\delta}{\cos^2\delta_r}
+\tan\delta_r-\frac{\delta_r}{\cos^2\delta_r}
$$

代入航向误差方程：

$$
\theta_{i+1}
=\theta_i
+\Delta s_i
\left(
\frac{1}{L\cos^2\delta_r}\delta_i
-\kappa_i
+\frac{1}{L}
\left(
\tan\delta_r-\frac{\delta_r}{\cos^2\delta_r}
\right)
\right)
$$

于是可以写成仿射离散系统：

$$
x_{i+1}=A_i x_i+B_i u_i+w_i
$$

其中：

$$
x_i=
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix},
\qquad
u_i=\delta_i
$$

$$
A_i=
\begin{bmatrix}
1 & \Delta s_i\\
0 & 1
\end{bmatrix}
$$

$$
B_i=
\begin{bmatrix}
0\\
\frac{\Delta s_i}{L\cos^2\delta_r}
\end{bmatrix}
$$

$$
w_i=
\begin{bmatrix}
0\\
\Delta s_i
\left(
-\kappa_i
+\frac{1}{L}
\left(
\tan\delta_r-\frac{\delta_r}{\cos^2\delta_r}
\right)
\right)
\end{bmatrix}
$$

如果为了稳定性暂时不使用参考曲率前馈，可以令 $\kappa_i=0$，于是：

$$
\delta_r=0
$$

$$
A_i=
\begin{bmatrix}
1 & \Delta s_i\\
0 & 1
\end{bmatrix},
\qquad
B_i=
\begin{bmatrix}
0\\
\frac{\Delta s_i}{L}
\end{bmatrix},
\qquad
w_i=
\begin{bmatrix}
0\\
0
\end{bmatrix}
$$

这时模型变成更简单的直线参考误差传播：

$$
l_{i+1}=l_i+\Delta s_i\theta_i
$$

$$
\theta_{i+1}=\theta_i+\frac{\Delta s_i}{L}\delta_i
$$

这种模型牺牲了一些曲率前馈精度，但在某些地图和边界场景下会更稳。

## 7. 把所有点堆叠成整段预测模型

单步模型只描述 $x_i$ 到 $x_{i+1}$。MPT 要优化整段窗口，所以把所有状态和输入堆叠：

$$
X=
\begin{bmatrix}
x_0\\
x_1\\
\vdots\\
x_{N-1}
\end{bmatrix}
$$

$$
U=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_{N-2}
\end{bmatrix}
$$

整段状态方程可以写成：

$$
X=A_\text{seq}X+B_\text{seq}U+W_\text{seq}
$$

很多人第一次看到这个式子会疑惑：为什么左右两边都是 $X$？左边不是预测后的状态，右边不是前一步状态吗？

关键在于：这里的 $X$ 是整段轨迹的状态堆叠向量，不是单个时刻的状态。矩阵 $A_\text{seq}$ 是一个块下移矩阵，它的第 $i$ 行块只会取到 $x_{i-1}$，不会取到同一个 $x_i$。

按块展开就是：

$$
x_1=A_0x_0+B_0u_0+w_0
$$

$$
x_2=A_1x_1+B_1u_1+w_1
$$

$$
\cdots
$$

$$
x_{N-1}=A_{N-2}x_{N-2}+B_{N-2}u_{N-2}+w_{N-2}
$$

所以左右两边写成同一个 $X$，只是为了把整段递推关系压缩成矩阵形式。

为了放进 QP 约束，把它移项：

$$
(I-A_\text{seq})X-B_\text{seq}U=W_\text{seq}
$$

这就是状态方程等式约束。

为什么不直接消元成 $X=GU+h$，只优化 $U$？因为保留 $X$ 作为决策变量后，可以非常方便地对每个轨迹点添加道路边界、碰撞圆、固定点、终端约束等条件。代价是变量更多，但约束表达更清晰。

## 8. 决策变量的完整形式

MPT 的二次规划决策变量不是只有转向角。完整变量通常写成：

$$
v=
\begin{bmatrix}
X\\
U\\
S
\end{bmatrix}
$$

其中：

- $X$ 是所有点的横向误差和航向误差。
- $U$ 是每段的转向角。
- $S$ 是软碰撞约束的松弛变量。

如果没有软约束，$S$ 的维度就是 $0$。如果有软约束，$S$ 用来允许少量边界违反，但违反量会被代价函数惩罚。

二次规划标准形式为：

$$
\min_v\ \frac{1}{2}v^THv+f^Tv
$$

$$
\text{s.t.}\quad l_b\le Av\le u_b
$$

接下来要做的所有事情，本质上都是把“想要的轨迹性质”翻译成 $H,f,A,l_b,u_b$。

## 9. 目标函数：跟踪、平滑和避障之间的权衡

最基础的目标函数可以写成：

$$
J=
\frac{1}{2}X^TQX
+\frac{1}{2}U^TRU
$$

$Q$ 惩罚状态误差，$R$ 惩罚控制输入。

对每个参考点：

$$
x_i=
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix}
$$

状态误差代价一般是：

$$
q_{l,i}l_i^2+q_{\theta,i}\theta_i^2
$$

$q_{l,i}$ 越大，轨迹越贴近参考线；$q_{\theta,i}$ 越大，车辆方向越贴近参考线方向。

控制输入代价一般是：

$$
r_i u_i^2
$$

它会抑制过大的转向角。

为了让转向变化平滑，还会加入转向变化率代价：

$$
J_\text{rate}=\sum_{i=1}^{N-2} r_\Delta (u_i-u_{i-1})^2
$$

展开单项：

$$
(u_i-u_{i-1})^2=u_i^2-2u_iu_{i-1}+u_{i-1}^2
$$

因此它会在 $R$ 矩阵里产生相邻输入之间的非对角项。直观上，这会让优化器不喜欢“这一段左打很多、下一段又右打很多”的抖动转向。

终端点通常有更大的权重。如果优化窗口最后一个点还不是最终目标点，就使用 terminal 权重，使局部窗口末端不要散得太开；如果它已经是真正目标点，就使用更强的 goal 权重，使轨迹尽量精确收敛到目标。

在避障区域，权重会自适应变化。设归一化避障代价为 $c_i\in[0,1]$，某个权重可以线性插值：

$$
w_i=(1-c_i)w_\text{normal}+c_iw_\text{avoid}
$$

当 $c_i=0$，使用普通跟踪权重；当 $c_i=1$，使用避障区域权重。这样优化器在障碍附近可以改变偏好，比如更强调姿态平顺或让出横向空间，而不是死贴参考线。

## 10. 为什么要引入优化中心

如果目标函数只惩罚参考点处的横向误差 $l_i$，会出现一个问题：车辆基准点贴着参考线，不代表车辆前方也贴着参考线。

在直线上，如果：

$$
l_i=0,\qquad \theta_i\ne 0
$$

那么车辆基准点在参考线上，但车头已经朝向一侧。距离车辆基准点前方 $d$ 处的点，会产生近似横向偏移：

$$
d\theta_i
$$

因此 MPT 不只看基准点，而是把横向误差的评价点前移一段距离。这个点可以叫优化中心。它的纵向前移距离记为：

$$
d=\text{optimization\_center\_offset}
$$

直线情况下，优化中心横向误差近似为：

$$
z_{l,i}=l_i+d\theta_i
$$

这会让目标函数主动抑制车头摆动。

## 11. $\alpha$ 的含义：弯道上优化中心该往哪里投影

在弯道上，不能简单认为前方参考点还沿当前参考点的 $x$ 轴方向。参考线会转弯，所以要引入 $\alpha_i$。

$\alpha_i$ 表示当前参考点的航向 $\psi_i^\text{ref}$ 和前方约一个轴距处参考方向之间的夹角。它可以理解为“车辆尺度下，参考线前方弯向哪里”。

这里容易混淆两个距离：

- $\alpha_i$ 的计算使用轴距 $L$，因为它想估计车辆尺度上的路径弯曲方向。
- 优化中心距离使用 $d=\text{optimization\_center\_offset}$，因为它是目标函数真正惩罚的前向距离，是可调参数。

它们不必相等。$\alpha_i$ 决定“横向误差往哪个方向投影”，$d$ 决定“前方多远的点被惩罚”。

下面推导弯道上的优化中心横向误差。

以当前参考点为局部坐标原点，$x$ 轴沿当前参考方向，$y$ 轴沿左法向。令：

$$
l=l_i,\qquad \theta=\theta_i,\qquad d=\text{optimization\_center\_offset},\qquad \alpha=\alpha_i
$$

车辆基准点的局部坐标为：

$$
\begin{bmatrix}
0\\
l
\end{bmatrix}
$$

车辆前方 $d$ 处的优化中心为：

$$
p_\text{veh}=
\begin{bmatrix}
0\\
l
\end{bmatrix}
+d
\begin{bmatrix}
\cos\theta\\
\sin\theta
\end{bmatrix}=
\begin{bmatrix}
d\cos\theta\\
l+d\sin\theta
\end{bmatrix}
$$

前方参考点近似位于：

$$
p_\text{ref}=d
\begin{bmatrix}
\cos\alpha\\
\sin\alpha
\end{bmatrix}
$$

前方参考方向的左法向为：

$$
n_\alpha=
\begin{bmatrix}
-\sin\alpha\\
\cos\alpha
\end{bmatrix}
$$

优化中心横向误差就是二者差值投影到 $n_\alpha$ 上：

$$
z_l=n_\alpha^T(p_\text{veh}-p_\text{ref})
$$

展开：

$$
z_l=
\begin{bmatrix}
-\sin\alpha & \cos\alpha
\end{bmatrix}
\left(
\begin{bmatrix}
d\cos\theta\\
l+d\sin\theta
\end{bmatrix}-
d
\begin{bmatrix}
\cos\alpha\\
\sin\alpha
\end{bmatrix}
\right)
$$

参考点自身那一项投影为 $0$，因为 $p_\text{ref}$ 沿前方切向，不沿法向。于是：

$$
z_l=
l\cos\alpha
d(-\sin\alpha\cos\theta+\cos\alpha\sin\theta)
$$

括号内是三角恒等式：

$$
-\sin\alpha\cos\theta+\cos\alpha\sin\theta
=\sin(\theta-\alpha)
$$

所以：

$$
z_l=l\cos\alpha+d\sin(\theta-\alpha)
$$

为了保持 QP 的线性结构，在 $\theta=0$ 附近一阶展开：

$$
\sin(\theta-\alpha)
\approx
\sin(-\alpha)+\theta\cos(-\alpha)
$$

因为：

$$
\sin(-\alpha)=-\sin\alpha,\qquad \cos(-\alpha)=\cos\alpha
$$

所以：

$$
\sin(\theta-\alpha)
\approx
-\sin\alpha+\theta\cos\alpha
$$

代回：

$$
z_{l,i}
\approx
\cos\alpha_i\ l_i
d\cos\alpha_i\ \theta_i
-d\sin\alpha_i
$$

航向误差仍然直接取：

$$
z_{\theta,i}=\theta_i
$$

因此每个点的状态变换为：

$$
\begin{bmatrix}
z_{l,i}\\
z_{\theta,i}
\end{bmatrix}=
\begin{bmatrix}
\cos\alpha_i & d\cos\alpha_i\\
0 & 1
\end{bmatrix}
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix}+
\begin{bmatrix}
-d\sin\alpha_i\\
0
\end{bmatrix}
$$

把所有点堆叠起来：

$$
Z=TX+t
$$

如果目标函数写成：

$$
J_Z=\frac{1}{2}Z^TQZ
$$

代入 $Z=TX+t$：

$$
J_Z=
\frac{1}{2}(TX+t)^TQ(TX+t)
$$

展开：

$$
J_Z=
\frac{1}{2}X^TT^TQTX
+t^TQTX
+\frac{1}{2}t^TQt
$$

最后一项是常数，不影响最优解。于是状态代价对 QP 的贡献可以理解为：

$$
H_X=T^TQT
$$

$$
f_X=T^TQt
$$

如果采用不同的 QP 系数约定，整体可能差一个 $2$ 的因子；只要代价和求解器标准形式一致，最优解不变。

## 12. 道路边界先是“点上的左右距离”

车辆不能只优化得平滑，还必须在可行驶区域内。参考路径有左右边界，MPT 会把边界转换成每个参考点处的横向可行范围：

$$
l_i^\text{lower}\le l_i\le l_i^\text{upper}
$$

注意这里的边界不是一个固定常数，而是沿路径变化的。道路变窄、障碍物侵入、车道边界弯曲，都会让每个参考点上的边界不同。

边界通常需要考虑安全裕度。假设道路边界距离是相对参考线中心的几何距离，车辆有宽度 $w$，还希望保留 clearance，那么真正可允许车辆中心出现的位置要向内收缩：

$$
\text{effective clearance}=\frac{w}{2}+\text{clearance}
$$

如果边界太窄，工程上还需要保证最小可行宽度。否则 QP 可能因为道路宽度小于车身宽度而无解。

还有一个稳定性问题：如果当前帧边界突然变窄，优化器可能突然大幅打方向。为避免这种情况，自车附近可以继承上一帧的边界宽度或做平滑处理，让边界约束不要瞬间跳变。

## 13. 为什么车辆用多个圆近似

真实车辆是一个矩形或多边形。如果直接把矩形和道路边界做精确碰撞约束，约束会比较复杂。MPT 常用多个圆来近似车辆外形。

每个圆由两个量描述：

$$
(d_l,r_l)
$$

其中 $d_l$ 是第 $l$ 个圆心相对车辆基准点的纵向偏移，$r_l$ 是圆半径。

圆形近似有几个好处：

- 圆的朝向无关，不需要考虑矩形旋转后的四个角。
- 只要每个圆都在边界内，就可以近似保证车身不越界。
- 多个圆可以覆盖车头、车身中部和车尾。

常见圆形布置方式有：

- uniform circle：沿车辆长度均匀放置多个等半径圆。
- bicycle model：重点放置后轴附近和前轴附近的圆，贴合自行车模型的关键几何点。
- fitting uniform circle：用固定半径沿车长拟合覆盖车辆轮廓。

圆越多，碰撞约束越精细，但 QP 约束行数也越多。

## 14. 车辆圆心横向位置的线性化推导

下面推导 MPT 中最重要的碰撞约束。

第 $i$ 个参考点的参考航向为 $\psi_i^\text{ref}$。车辆圆所在的路径位置通常不是当前参考点，而是在前后偏移 $d_l$ 后的位置。这个圆心对应路径位置的参考航向记为 $\psi_{i,l}^\text{circle}$。

定义：

$$
\beta_{i,l}=\psi_i^\text{ref}-\psi_{i,l}^\text{circle}
$$

$\beta$ 表达的是：当前参考点的切线方向，与车辆圆所在路径位置的切线方向之间的差。在直线上，$\beta=0$。在弯道上，前方圆、后方圆对应的路径方向可能不同，所以必须引入 $\beta$。

当前参考点的左法向为：

$$
n_i=
\begin{bmatrix}
-\sin\psi_i^\text{ref}\\
\cos\psi_i^\text{ref}
\end{bmatrix}
$$

车辆圆所在路径位置的左法向为：

$$
n_{i,l}^\text{circle}=
\begin{bmatrix}
-\sin\psi_{i,l}^\text{circle}\\
\cos\psi_{i,l}^\text{circle}
\end{bmatrix}
$$

优化状态仍然是：

$$
x_i=
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix}
$$

车辆基准点相对参考点的横向位移为：

$$
l_i n_i
$$

第 $l$ 个车辆圆心相对基准点的纵向位移为：

$$
d_l
\begin{bmatrix}
\cos(\psi_i^\text{ref}+\theta_i)\\
\sin(\psi_i^\text{ref}+\theta_i)
\end{bmatrix}
$$

因此圆心相对当前参考点的位移近似为：

$$
p_{i,l}^\text{rel}=
l_i n_i+
d_l
\begin{bmatrix}
\cos(\psi_i^\text{ref}+\theta_i)\\
\sin(\psi_i^\text{ref}+\theta_i)
\end{bmatrix}
$$

碰撞约束关心这个圆心在“圆心对应路径位置”的法向坐标，所以投影到 $n_{i,l}^\text{circle}$ 上：

$$
y_{i,l}=
\left(n_{i,l}^\text{circle}\right)^T p_{i,l}^\text{rel}
$$

展开第一项：

$$
\left(n_{i,l}^\text{circle}\right)^T n_i=
\cos(\psi_i^\text{ref}-\psi_{i,l}^\text{circle})=
\cos\beta_{i,l}
$$

展开第二项：

$$
\left(n_{i,l}^\text{circle}\right)^T
\begin{bmatrix}
\cos(\psi_i^\text{ref}+\theta_i)\\
\sin(\psi_i^\text{ref}+\theta_i)
\end{bmatrix}=
\sin(\psi_i^\text{ref}+\theta_i-\psi_{i,l}^\text{circle})
$$

也就是：

$$
\sin(\beta_{i,l}+\theta_i)
$$

所以精确形式是：

$$
y_{i,l}=
\cos\beta_{i,l}\ l_i+
d_l\sin(\beta_{i,l}+\theta_i)
$$

为了保持线性约束，在 $\theta_i=0$ 附近展开：

$$
\sin(\beta_{i,l}+\theta_i)
\approx
\sin\beta_{i,l}+
\theta_i\cos\beta_{i,l}
$$

因此：

$$
y_{i,l}
\approx
\cos\beta_{i,l}\ l_i+
d_l\cos\beta_{i,l}\ \theta_i+
d_l\sin\beta_{i,l}
$$

把它写成线性形式：

$$
y_{i,l}=C_{i,l}x_i+c_{i,l}
$$

其中：

$$
C_{i,l}=
\begin{bmatrix}
\cos\beta_{i,l} & d_l\cos\beta_{i,l}
\end{bmatrix}
$$

$$
c_{i,l}=d_l\sin\beta_{i,l}
$$

对所有参考点堆叠：

$$
y_l=C_lX+c_l
$$

这就是车辆圆边界约束的核心数学来源。

## 15. 圆半径如何影响边界

如果只约束圆心在道路边界内，圆的外缘仍然可能越界。所以边界要根据圆半径向内收缩。

设某处可行边界是：

$$
b^\text{lower}\le y\le b^\text{upper}
$$

如果第 $l$ 个圆半径为 $r_l$，圆心必须满足更严格的条件。直观上：

- 左边界方向要减去 $r_l$。
- 右边界方向也要向内减去 $r_l$。

在 Autoware 的边界处理里，基础边界往往已经考虑了车辆半宽 $\frac{w}{2}$。当换成圆约束时，需要把“半宽模型”转换成“圆半径模型”。一个常见修正量是：

$$
\Delta b=\frac{w}{2}-r_l
$$

如果 $r_l>\frac{w}{2}$，$\Delta b<0$，可行范围会收紧。这个情况很常见，因为为了覆盖车辆长度，圆半径可能比半车宽更大。

最终每个圆、每个参考点都有自己的边界：

$$
b_{i,l}^\text{lower}\le y_{i,l}\le b_{i,l}^\text{upper}
$$

结合上一节的线性化：

$$
b_{i,l}^\text{lower}
\le
C_{i,l}x_i+c_{i,l}
\le
b_{i,l}^\text{upper}
$$

移项：

$$
b_{i,l}^\text{lower}-c_{i,l}
\le
C_{i,l}x_i
\le
b_{i,l}^\text{upper}-c_{i,l}
$$

这就是硬碰撞边界约束。

## 16. 软约束：允许违反，但要付出代价

真实场景中，边界可能因为感知噪声、地图误差、障碍物侵入或参考线位置不佳而短时间不可行。如果所有碰撞边界都是硬约束，QP 可能直接无解。

软约束的思想是引入松弛变量 $s$：

$$
s\ge 0
$$

原本的硬约束：

$$
b^\text{lower}-c\le Cx\le b^\text{upper}-c
$$

变成：

$$
Cx+s\ge b^\text{lower}-c
$$

$$
-Cx+s\ge c-b^\text{upper}
$$

$$
s\ge 0
$$

理解一下第二个式子：

$$
-Cx+s\ge c-b^\text{upper}
$$

等价于：

$$
Cx\le b^\text{upper}-c+s
$$

也就是说，如果 $Cx$ 超过上界，$s$ 可以变大来让约束仍然可行。

软约束不是“不管碰撞”。因为目标函数会加入：

$$
J_\text{slack}=\rho\sum s_i
$$

$\rho$ 越大，优化器越不愿意违反边界。只有当严格满足边界会导致问题不可行，或代价极高时，它才会使用 slack。

如果使用 $L_\infty$ 风格的 slack，可以让同一个参考点的多个车辆圆共享一个松弛变量。这样 $s_i$ 表达的是该点所有圆中最大的违反程度。若不使用 $L_\infty$，每个车辆圆可以拥有独立 slack，表达更细，但变量更多。

## 17. 固定点：让相邻帧轨迹连续

MPT 是循环运行的，每一帧都会重新优化。如果每次都完全自由优化，自车附近轨迹可能每帧略微改变。控制器跟踪的是连续输出轨迹，这种跳变会导致不稳定。

固定点的思想是：当前帧优化窗口前端的一部分点继承上一帧优化结果，并把它们作为等式约束锁住。

如果某个参考点被固定，它的状态满足：

$$
x_i=x_i^\text{fixed}
$$

也就是：

$$
\begin{bmatrix}
l_i\\
\theta_i
\end{bmatrix}=
\begin{bmatrix}
l_i^\text{prev}\\
\theta_i^\text{prev}
\end{bmatrix}
$$

这不是把整条轨迹都固定，而是只固定当前窗口最前面的一个或两个点。固定一个点可以保证位置连续；固定两个点可以更稳定地继承前端方向，因为方向往往由相邻点共同决定。

哪些点可以成为固定点？一般需要满足：

- 存在上一帧优化轨迹。
- 当前窗口前端可以在上一帧参考点中找到对应位置。
- 对应点与当前参考点距离、方向足够匹配。

如果这些条件不成立，就不应该强行固定，否则会把当前优化绑定到错误的历史轨迹上。

## 18. 转向角约束

车辆转向角有物理限制。最简单的形式是：

$$
\delta_\text{min}\le u_i\le \delta_\text{max}
$$

也可以围绕参考曲率对应的理想转角设置局部范围：

$$
\delta_i^\text{ref}-\delta_\text{margin}
\le
u_i
\le
\delta_i^\text{ref}+\delta_\text{margin}
$$

其中：

$$
\delta_i^\text{ref}=\arctan(L\kappa_i)
$$

如果参考线曲率很大，参考转角也大；约束围绕它展开，可以允许车辆跟随弯道，同时避免转角偏离过多。

## 19. 把所有约束统一成 QP

现在所有元素都齐了。

决策变量：

$$
v=
\begin{bmatrix}
X\\
U\\
S
\end{bmatrix}
$$

目标函数：

$$
\min_v
\frac{1}{2}v^THv+f^Tv
$$

约束：

$$
l_b\le Av\le u_b
$$

约束矩阵里依次包含：

1. 状态方程：

$$
(I-A_\text{seq})X-B_\text{seq}U=W_\text{seq}
$$

2. 车辆圆边界硬约束：

$$
b^\text{lower}-c\le CX\le b^\text{upper}-c
$$

3. 车辆圆边界软约束：

$$
CX+s\ge b^\text{lower}-c
$$

$$
-CX+s\ge c-b^\text{upper}
$$

$$
s\ge 0
$$

4. 固定点：

$$
x_i=x_i^\text{fixed}
$$

5. 转向角限制：

$$
u_i^\text{lower}\le u_i\le u_i^\text{upper}
$$

这些约束都是线性的，目标函数是二次的，因此可以用 OSQP 这类二次规划求解器求解。

## 20. 求解结果如何变回轨迹

QP 求出的结果是：

$$
v^\star=
\begin{bmatrix}
X^\star\\
U^\star\\
S^\star
\end{bmatrix}
$$

其中 $X^\star$ 里每个点都有：

$$
x_i^\star=
\begin{bmatrix}
l_i^\star\\
\theta_i^\star
\end{bmatrix}
$$

把它变回轨迹点：

$$
p_i^\star=p_i^\text{ref}+l_i^\star n_i
$$

$$
\psi_i^\star=\psi_i^\text{ref}+\theta_i^\star
$$

转向角 $U^\star$ 不一定直接输出给控制器，但会被保存下来，用于下一帧 warm start、固定点和调试分析。slack $S^\star$ 也会被保存，用来理解哪些地方发生了软约束违反。

工程上还会做结果校验。如果某个点的横向误差或航向误差过大，例如：

$$
|l_i|>l_\text{max}
$$

或：

$$
|\theta_i|>\theta_\text{max}
$$

就可以认为本次优化结果不可靠，选择回退策略。

## 21. warm start：上一帧解如何帮助当前帧

MPT 每一帧都在解相似的 QP。当前帧参考点通常只是相对上一帧向前移动了一点，所以上一帧的最优解是很好的初始猜测。

有两种容易混淆的 warm start。

第一种是求解器层面的 warm start。矩阵尺寸不变时，可以复用上一帧求解器内部结构，只更新矩阵数值和边界。这样减少初始化开销。

第二种是手动变量平移。构造上一帧对应的完整初始解：

$$
u_0=
\begin{bmatrix}
X_0\\
U_0\\
S_0
\end{bmatrix}
$$

然后令：

$$
v=u_0+\Delta v
$$

原始目标函数：

$$
\frac{1}{2}v^THv+f^Tv
$$

代入 $v=u_0+\Delta v$：

$$
\frac{1}{2}(u_0+\Delta v)^TH(u_0+\Delta v)
+f^T(u_0+\Delta v)
$$

展开：

$$=
\frac{1}{2}\Delta v^TH\Delta v
+
(f+Hu_0)^T\Delta v
+
\text{const}
$$

常数项不影响最优解，所以新的线性项是：

$$
f'=f+Hu_0
$$

约束也要平移：

$$
l_b\le A(u_0+\Delta v)\le u_b
$$

得到：

$$
l_b-Au_0\le A\Delta v\le u_b-Au_0
$$

所以：

$$
l_b'=l_b-Au_0
$$

$$
u_b'=u_b-Au_0
$$

求解器得到的是 $\Delta v^\star$，最终还原：

$$
v^\star=u_0+\Delta v^\star
$$

这里必须注意：$u_0$ 必须和完整决策变量 $v=[X,U,S]^T$ 同维度。如果只给转向角初值，不能直接用于这个变量平移公式。

## 22. 为什么不是每一帧都必须重优化

MPT 计算有成本，而且轨迹输出需要稳定。工程系统通常会判断当前帧是否需要重规划。

如果路径变化很小、自车没有明显跳变、目标点没有重要变化，就可以复用上一帧优化轨迹。这样有两个好处：

- 节省 QP 求解时间。
- 输出轨迹更稳定，不会因为输入路径微小抖动而每帧变化。

一般会触发“重置并重规划”的情况包括：

- 没有上一帧数据。
- 自车附近路径形状明显变化。
- 自车位姿相对上一帧跳变过大。
- 车辆停止附近目标点发生较大变化。

也有“不重置但需要重新优化”的情况：

- 距离上次重规划已经超过设定时间。
- 前方路径形状变化超过阈值。

区别在于：重置会清空上一帧优化状态，不再信任固定点和 warm start；普通重规划仍可以继承上一帧数据，用它保持连续性。

前方路径变化可以这样理解：从上一帧轨迹上沿自车前方取若干点，比如每隔 $10\ \mathrm{m}$ 取一个点，最多检查前方 $100\ \mathrm{m}$。把这些上一帧前方点投影到当前输入路径上，如果横向偏差超过阈值，就说明前方路径形状变了，需要重新优化。

## 23. 局部优化之后，还要和原始轨迹拼接

MPT 只优化局部窗口。可是系统最终输出的轨迹可能需要比优化窗口更长。因此需要把 MPT 输出的局部轨迹和原始输入轨迹后半段拼接。

拼接时要注意几个问题：

- 拼接点附近需要平滑，避免优化轨迹末端和原始轨迹出现角度跳变。
- 输出轨迹要重新按较小间隔采样，满足控制器输入要求。
- 如果原始轨迹中有停止点，拼接和重采样后要显式恢复停止点。
- 速度通常来自输入轨迹，而不是 MPT 单独优化。MPT 主要优化几何形状。

也就是说，MPT 解决的是局部几何与运动学可行性；速度、停止点和长距离轨迹连续性还要由外层轨迹处理补齐。

## 24. 可行驶区域外停止

即使 MPT 做了边界约束，工程上仍可能额外检查最终轨迹是否越出可行驶区域。原因包括：

- 边界约束可能是软约束，允许少量违反。
- 车辆圆近似不是完整矩形精确碰撞。
- 轨迹末端或高曲率区域可能存在插值误差。

一种保守策略是：沿输出轨迹检查车辆 footprint 是否在可行驶区域内。如果发现第一个越界点，就在它之前插入停止点，并把后续速度置零。

这不是 MPT 的核心数学，但它是自动驾驶系统里很重要的安全兜底。

## 25. 避障代价如何形成一段区域，而不是一个点

边界变窄或障碍物侵入时，某一个参考点可能检测到较高避障代价。可是车辆不能只在一个离散点突然改变行为，因此避障代价会沿纵向扩散成一段区域。

设某点的归一化避障代价为：

$$
c_i\in[0,1]
$$

可以在其前后一定长度内设置同样或逐渐衰减的代价，形成避障带。衰减可以理解为：

$$
c_{i+k}=\max(0,c_i-\lambda |k|)
$$

其中 $\lambda$ 是每个离散点的衰减率。

这样权重不会在障碍物边缘突然跳变，优化结果也会更平滑。

为了防止感知或边界抖动，还可以继承上一帧附近点的避障代价：

$$
c_i^\text{new}=\max(c_i^\text{current},c_i^\text{prev})
$$

这会让避障意图具有短时间记忆，避免某一帧障碍边界消失导致轨迹突然回摆。

## 26. 参数直觉

MPT 的行为主要由几类参数决定。

优化窗口参数：

$$
N,\quad \Delta s
$$

$N\Delta s$ 决定前向视野。视野太短，轨迹容易短视；视野太长，QP 规模变大。

跟踪权重：

$$
q_l,\quad q_\theta
$$

$q_l$ 大会更贴参考线，$q_\theta$ 大会更贴参考方向。若 $q_l$ 太大，遇到障碍时可能不愿意绕；若 $q_\theta$ 太大，可能姿态很稳但横向调整变慢。

控制权重：

$$
r_u,\quad r_{\Delta u}
$$

$r_u$ 抑制转向角，$r_{\Delta u}$ 抑制转向变化率。后者越大，转向越平顺，但响应会变慢。

软约束权重：

$$
\rho
$$

$\rho$ 越大，越接近硬约束；$\rho$ 太小，轨迹可能更愿意侵入边界来换取平滑。

优化中心距离：

$$
d=\text{optimization\_center\_offset}
$$

$d$ 越大，同样的航向误差会产生越大的前方横向误差惩罚：

$$
z_l\approx l+d\theta
$$

因此它能抑制车头摆动。但 $d$ 太大也会让优化器对航向误差过于敏感。

车辆圆数量：

$$
N_\text{circle}
$$

圆越多，车身覆盖越精细，但约束更多，求解更重。圆太少，碰撞近似会粗糙。

## 27. 几个常见误解

第一个误解：MPT 优化的是全局路径。

不是。MPT 优化的是自车附近局部窗口。全局路线、行为决策、车道选择通常来自上游。

第二个误解：决策变量只有转向角。

不是。完整变量是：

$$
v=[X,U,S]^T
$$

保留 $X$ 是为了方便添加状态约束和碰撞约束。

第三个误解：$X=A X+B U+W$ 表示同一个状态等于自己。

不是。这里的 $X$ 是整段状态堆叠向量。右边的矩阵 $A$ 只会在第 $i$ 行块取到前一个状态 $x_{i-1}$。

第四个误解：soft constraint 就是不管碰撞。

不是。soft constraint 允许违反，但违反量 $s$ 会进入目标函数：

$$
\rho\sum s_i
$$

它是为了避免 QP 无解，而不是鼓励碰撞。

第五个误解：$\alpha$ 和 $\beta$ 是同一个东西。

不是。$\alpha$ 用在目标函数里，修正优化中心横向误差的投影方向。$\beta$ 用在碰撞约束里，修正车辆圆所在路径位置和当前参考点之间的方向差。

第六个误解：$\alpha$ 为什么用轴距，而优化中心用另一个 offset。

因为二者作用不同。$\alpha$ 是车辆尺度上的路径弯曲方向估计，轴距是物理尺度；optimization center offset 是代价函数评价点的前移距离，是调参量。

## 28. 一条完整的 MPT 思维路线

现在可以把 MPT 串起来。

首先，上游给出一条参考路径和左右边界。MPT 截取自车附近局部窗口，按固定弧长重采样，并用样条得到稳定的航向和曲率。

然后，在每个参考点建立局部误差状态：

$$
x_i=[l_i,\theta_i]^T
$$

用自行车模型推导相邻点之间的误差传播：

$$
x_{i+1}=A_ix_i+B_iu_i+w_i
$$

把整段窗口堆叠为：

$$
(I-A_\text{seq})X-B_\text{seq}U=W_\text{seq}
$$

接着构造目标函数。轨迹应该贴近参考线，但不是只惩罚基准点，而是惩罚前方优化中心：

$$
Z=TX+t
$$

同时惩罚转向角和转向变化率：

$$
J=
\frac{1}{2}Z^TQZ
+\frac{1}{2}U^TRU
+\rho\sum S
$$

然后构造边界约束。车辆用多个圆近似，每个圆心横向位置线性化为：

$$
y_{i,l}
\approx
\cos\beta_{i,l}l_i
+
d_l\cos\beta_{i,l}\theta_i
+
d_l\sin\beta_{i,l}
$$

要求圆心加半径后仍在边界内：

$$
b_{i,l}^\text{lower}
\le
y_{i,l}
\le
b_{i,l}^\text{upper}
$$

如果边界可能不可行，就用 slack 变成软约束。

再加入固定点约束保持相邻帧连续，加入转向角约束保持物理可执行。

最后，把所有内容交给 QP 求解器：

$$
\min_v\ \frac{1}{2}v^THv+f^Tv
$$

$$
\text{s.t.}\quad l_b\le Av\le u_b
$$

求解得到 $v^\star=[X^\star,U^\star,S^\star]^T$，再把每个 $x_i^\star=[l_i^\star,\theta_i^\star]^T$ 转回全局轨迹点。

MPT 的本质就是这样：它把“车辆应该走得平滑、安全、可执行、连续”这些直觉要求，全部翻译成一个结构清晰的二次规划问题。

## 29. 学会 MPT 后应该记住什么

MPT 最核心的不是某个求解器，而是建模方式。

第一，使用参考线局部误差，而不是直接优化全局坐标：

$$
x_i=[l_i,\theta_i]^T
$$

第二，用空间域自行车模型连接相邻点：

$$
\frac{dl}{ds}=\theta,\qquad
\frac{d\theta}{ds}=\frac{\tan\delta}{L}-\kappa
$$

第三，把所有状态堆叠，但不消元，方便添加约束：

$$
v=[X,U,S]^T
$$

第四，通过优化中心让目标函数感知车头摆动：

$$
z_l\approx \cos\alpha\ l+d\cos\alpha\ \theta-d\sin\alpha
$$

第五，通过车辆圆把车身碰撞约束线性化：

$$
y_\text{circle}
\approx
\cos\beta\ l
+d_l\cos\beta\ \theta
+d_l\sin\beta
$$

第六，用 slack 把不可行风险变成可控代价：

$$
J_\text{slack}=\rho\sum s
$$

第七，用固定点、warm start、重规划判断和轨迹拼接保证工程系统稳定运行。

如果把这些概念连起来，你就已经掌握了 Autoware MPT 的主要思想：它不是神秘的黑盒优化器，而是一套围绕参考线误差、车辆运动学、边界线性化和二次规划组织起来的局部轨迹生成方法。
