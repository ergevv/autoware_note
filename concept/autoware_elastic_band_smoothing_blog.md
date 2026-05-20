参考来源：ChatGPT、autoware

# Autoware Elastic Band 路径平滑算法

## 1. 先理解它要解决什么问题

自动驾驶规划链路里，上游模块通常已经给出了一条可以行驶的参考路径。可是这条路径未必足够光顺：它可能来自车道中心线、避障模块、路径拼接、离散采样或历史轨迹复用，因此局部会出现轻微锯齿、折角、航向跳变。

这些小的几何不连续看起来不严重，但会放大到后续模块里。尤其是基于 Frenet 坐标系或模型预测轨迹优化的方法，会频繁使用参考路径的切向、法向、曲率和横向误差。如果参考路径本身抖动，Frenet 投影、航向误差、曲率估计和转向输入都会变得不稳定。

Elastic Band 平滑器的目标不是重新规划一条全新的路径，而是在原始路径附近做一次局部几何修正：

$$
\text{在不大幅偏离原始路径的前提下，让离散路径尽量光顺。}
$$

这里有两个关键词。

第一，**局部**。它只处理自车附近一段有限长度的路径，而不是整条全局路径。

第二，**小幅修正**。它不会允许路径点随意移动，而是限制每个点只能沿参考路径的横向方向移动一小段距离。

所以 Autoware 里的 Elastic Band 可以先记成一句话：

$$
\text{Autoware Elastic Band 是一个带横向位移约束的离散路径平滑 QP。}
$$

它名字里有 Elastic Band，但它并不是传统机器人规划里那种“弹簧内力 + 障碍物斥力”的物理迭代法。Autoware 版本更像是把路径点当成一串受限的控制点，通过二次规划求出每个点的横向偏移量。

## 2. 路径为什么可以被看成一串离散点

一条连续路径可以写成平面曲线：

$$
\mathbf{p}(s)=\begin{bmatrix}
x(s) \\
y(s)
\end{bmatrix}
$$

其中 $s$ 是沿路径的弧长。实际工程里不直接处理连续函数，而是按一定弧长间隔采样，得到 $N$ 个离散点：

$$
\mathbf{r}_0,\mathbf{r}_1,\dots,\mathbf{r}_{N-1}
$$

第 $i$ 个参考点写作：

$$
\mathbf{r}_i=\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
$$

每个点还带有一个参考航向角 $\theta_i$。由航向角可以构造路径局部坐标系：

$$
\mathbf{t}_i=\begin{bmatrix}
\cos\theta_i \\
\sin\theta_i
\end{bmatrix},
\qquad
\mathbf{n}_i=\begin{bmatrix}
-\sin\theta_i \\
\cos\theta_i
\end{bmatrix}
$$

$\mathbf{t}_i$ 是切向量，表示路径前进方向；$\mathbf{n}_i$ 是左法向量，表示横向方向。

Autoware Elastic Band 的关键限制是：

$$
\text{每个点的纵向移动为 }0,\quad \text{只允许沿法向横向移动。}
$$

因此优化后的点不是任意二维变量，而是：

$$
\mathbf{p}_i = \mathbf{r}_i+l_i\mathbf{n}_i
$$

其中 $l_i$ 是第 $i$ 个点的横向偏移量。把所有横向偏移量收集起来：

$$
\mathbf{l}=\begin{bmatrix}
l_0 & l_1 & \cdots & l_{N-1}
\end{bmatrix}^{T}\in\mathbb{R}^{N}
$$

这就是 Elastic Band 真正优化的变量。

这个设计非常重要。如果直接优化每个点的 $x_i,y_i$，变量维度是 $2N$，路径点可能沿前后方向乱滑，原始路径的采样顺序和局部结构也更容易被破坏。改成只优化 $l_i$ 后，问题维度变成 $N$，每个点只能在一条很短的横向线段上移动，路径拓扑和前后顺序更稳定。

## 3. 什么叫“路径更平滑”

对离散路径来说，平滑性可以从相邻线段的变化来理解。

第 $k$ 个点前后的两段离散切向量近似为：

$$
\mathbf{p}_k-\mathbf{p}_{k-1},
\qquad
\mathbf{p}_{k+1}-\mathbf{p}_k
$$

如果路径局部很平顺，这两个向量应该接近；如果路径出现折角，它们差得就会比较大。因此可以用下面这个量衡量局部折角：

$$
(\mathbf{p}_{k+1}-\mathbf{p}_k)-(\mathbf{p}_k-\mathbf{p}_{k-1})
$$

整理后得到：

$$
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
$$

这就是离散二阶差分。它对应连续曲线里的二阶导数或曲率趋势：一阶差分描述“方向”，二阶差分描述“方向变化”。

于是路径平滑的核心代价可以写成：

$$
J_{\text{smooth}} = \sum_{k=1}^{N-2}
\left\|
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
\right\|^2
$$

如果某一段路径近似为直线，相邻线段方向几乎不变，二阶差分就接近 $0$。如果路径有尖角，二阶差分会变大。

这也解释了 Autoware 文档里提到的“菱形对角线”直觉：由前一点、当前点、后一点形成的局部几何关系中，二阶差分向量正是在描述前后两条边的差异。让这个向量短，就等价于让局部折角小。

展开到坐标形式，令：

$$
\mathbf{p}_k=\begin{bmatrix}
x_k \\
y_k
\end{bmatrix}
$$

则：

$$
\left\|
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
\right\|^2 = (x_{k+1}-2x_k+x_{k-1})^2 + (y_{k+1}-2y_k+y_{k-1})^2
$$

所以总代价是：

$$
J_{\text{smooth}} = \sum_{k=1}^{N-2}
\left[
(x_{k+1}-2x_k+x_{k-1})^2 + (y_{k+1}-2y_k+y_{k-1})^2
\right]
$$

这只是一个非常经典的离散曲线平滑目标。

## 4. 二阶差分如何变成矩阵

为了把问题交给 QP 求解器，需要把上面的平方和写成矩阵二次型。

先考虑一维点列：

$$
\mathbf{x}=\begin{bmatrix}
x_0 & x_1 & \cdots & x_{N-1}
\end{bmatrix}^{T}
$$

定义二阶差分矩阵 $D\in\mathbb{R}^{(N-2)\times N}$：

$$
D=\begin{bmatrix}
1 & -2 & 1 & 0 & \cdots & 0 \\
0 & 1 & -2 & 1 & \cdots & 0 \\
\vdots & & & & \ddots & \vdots \\
0 & \cdots & 0 & 1 & -2 & 1
\end{bmatrix}
$$

于是：

$$
D\mathbf{x} = \begin{bmatrix}
x_0-2x_1+x_2 \\
x_1-2x_2+x_3 \\
\vdots \\
x_{N-3}-2x_{N-2}+x_{N-1}
\end{bmatrix}
$$

一维二阶差分平方和就是：

$$
\|D\mathbf{x}\|^2 = (D\mathbf{x})^T(D\mathbf{x}) = \mathbf{x}^TD^TD\mathbf{x}
$$

记：

$$
P_D=D^TD
$$

那么 $P_D$ 是一个对称半正定矩阵。以 $N=5$ 为例：

$$
D=\begin{bmatrix}
1 & -2 & 1 & 0 & 0 \\
0 & 1 & -2 & 1 & 0 \\
0 & 0 & 1 & -2 & 1
\end{bmatrix}
$$

于是：

$$
D^TD=\begin{bmatrix}
1 & -2 & 1 & 0 & 0 \\
-2 & 5 & -4 & 1 & 0 \\
1 & -4 & 6 & -4 & 1 \\
0 & 1 & -4 & 5 & -2 \\
0 & 0 & 1 & -2 & 1
\end{bmatrix}
$$

中间行出现了典型的五点模板：

$$
\begin{bmatrix}
1 & -4 & 6 & -4 & 1
\end{bmatrix}
$$

这不是偶然。因为一个内点 $\mathbf{p}_j$ 会同时影响它附近多个二阶差分项：包含 $\mathbf{p}_{j-2}$、$\mathbf{p}_{j-1}$、$\mathbf{p}_j$、$\mathbf{p}_{j+1}$、$\mathbf{p}_{j+2}$ 的局部关系都会参与求导，所以 Hessian 自然是五对角带状结构。

二维路径有 $x$ 和 $y$ 两个坐标。把它们堆叠成：

$$
\mathbf{z}=\begin{bmatrix}
x_0 \\
x_1 \\
\vdots \\
x_{N-1} \\
y_0 \\
y_1 \\
\vdots \\
y_{N-1}
\end{bmatrix}\in\mathbb{R}^{2N}
$$

那么二维平滑矩阵可以写成块对角形式：

$$
P_0=\begin{bmatrix}
D^TD & 0 \\
0 & D^TD
\end{bmatrix}
$$

于是：

$$
J_{\text{smooth}} = \mathbf{z}^TP_0\mathbf{z}
$$

在标准 QP 中，目标函数常写成：

$$
\min_{\mathbf{v}}\quad \frac{1}{2}\mathbf{v}^TH\mathbf{v} + \mathbf{f}^T\mathbf{v}
$$

因此实际使用时可以把常数因子吸收到 $H$ 或权重里。对算法理解来说，关键不是 $\frac{1}{2}$ 的位置，而是二阶差分平方和会产生一个稀疏、对称、带状的二次型矩阵。

## 5. 为什么不能只最小化二阶差分

如果只优化：

$$
\min
\sum_{k=1}^{N-2}
\left\|
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
\right\|^2
$$

最优结果会倾向于把路径拉成尽可能直的线。但自动驾驶里的参考路径不是随便给的，它可能已经避开了障碍物，符合车道结构，包含停车点、目标点和行为规划意图。

所以 Autoware Elastic Band 同时做两件事：

$$
\text{一方面减少二阶差分，另一方面限制每个点不能离原路径太远。}
$$

限制有两层。

第一层是硬约束：每个点最多只能沿法向移动一定距离。

第二层是软代价：横向偏移越大，代价越大。

这使得最终轨迹不是“最直”的路径，而是“在允许范围内更平滑”的路径。

## 6. Autoware 的变量：从二维坐标压缩成横向偏移

回到第 $i$ 个参考点：

$$
\mathbf{r}_i=\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
$$

由航向角 $\theta_i$ 定义法向量：

$$
\mathbf{n}_i=\begin{bmatrix}
-\sin\theta_i \\
\cos\theta_i
\end{bmatrix}
$$

优化后的点为：

$$
\mathbf{p}_i(l_i)=\mathbf{r}_i+l_i\mathbf{n}_i
$$

展开为坐标：

$$
x_i'(l_i)=x_i-\sin\theta_i\,l_i
$$

$$
y_i'(l_i)=y_i+\cos\theta_i\,l_i
$$

这说明每个点只有一个自由度 $l_i$。$l_i>0$ 表示沿左法向偏移，$l_i<0$ 表示沿右法向偏移。

把参考路径坐标堆叠为：

$$
\mathbf{x}=\begin{bmatrix}
x_0 \\
\vdots \\
x_{N-1} \\
y_0 \\
\vdots \\
y_{N-1}
\end{bmatrix}\in\mathbb{R}^{2N}
$$

再定义法向映射矩阵 $T\in\mathbb{R}^{N\times 2N}$。它的第 $i$ 行只有两个非零元素：

$$
T_{i,i}=-\sin\theta_i,
\qquad
T_{i,i+N}=\cos\theta_i
$$

这个矩阵可以理解为“全局坐标与局部横向方向之间的桥”。

如果给定一个全局位移向量：

$$
\Delta\mathbf{z}=\begin{bmatrix}
\Delta x_0 \\
\vdots \\
\Delta x_{N-1} \\
\Delta y_0 \\
\vdots \\
\Delta y_{N-1}
\end{bmatrix}
$$

那么：

$$
(T\Delta\mathbf{z})_i = -\sin\theta_i\,\Delta x_i + \cos\theta_i\,\Delta y_i
$$

这正是第 $i$ 个点的全局位移在法向量 $\mathbf{n}_i$ 上的投影。

反过来，$T^T\mathbf{l}$ 会把横向偏移量映射回全局 $x/y$ 坐标增量：

$$
T^T\mathbf{l} = \begin{bmatrix}
-\sin\theta_0\,l_0 \\
\vdots \\
-\sin\theta_{N-1}\,l_{N-1} \\
\cos\theta_0\,l_0 \\
\vdots \\
\cos\theta_{N-1}\,l_{N-1}
\end{bmatrix}
$$

所以所有优化后的点可以统一写成：

$$
\mathbf{z}(\mathbf{l}) = \mathbf{x}+T^T\mathbf{l}
$$

这一步是理解 Autoware Elastic Band 的核心。原本平滑代价定义在全局坐标 $\mathbf{z}$ 上，但真正的优化变量是横向偏移 $\mathbf{l}$。只要把 $\mathbf{z}(\mathbf{l})$ 代入平滑代价，就能得到关于 $\mathbf{l}$ 的 QP。

## 7. 把平滑代价投影到横向偏移变量上

带权重的全局平滑矩阵记为：

$$
R=w_sP_0
$$

其中 $w_s$ 是平滑权重，$P_0$ 是二阶差分矩阵构成的块对角平滑矩阵。

优化后的全局坐标是：

$$
\mathbf{z}(\mathbf{l})=\mathbf{x}+T^T\mathbf{l}
$$

因此平滑代价可以写成：

$$
J_{\text{smooth}}(\mathbf{l}) = \frac{1}{2}
\left(
\mathbf{x}+T^T\mathbf{l}
\right)^T
R
\left(
\mathbf{x}+T^T\mathbf{l}
\right)
$$

展开：

$$
\begin{aligned}
J_{\text{smooth}}(\mathbf{l}) = \frac{1}{2}
\mathbf{x}^TR\mathbf{x} + \frac{1}{2}
\mathbf{x}^TRT^T\mathbf{l} + \frac{1}{2}
\mathbf{l}^TTR\mathbf{x} + \frac{1}{2}
\mathbf{l}^TTRT^T\mathbf{l}
\end{aligned}
$$

因为 $R$ 是对称矩阵，所以：

$$
\mathbf{x}^TRT^T\mathbf{l} = \mathbf{l}^TTR\mathbf{x}
$$

于是：

$$
J_{\text{smooth}}(\mathbf{l}) = \frac{1}{2}
\mathbf{l}^TTRT^T\mathbf{l} + \mathbf{l}^TTR\mathbf{x} + \frac{1}{2}\mathbf{x}^TR\mathbf{x}
$$

最后一项：

$$
\frac{1}{2}\mathbf{x}^TR\mathbf{x}
$$

只和原始参考路径有关，不含优化变量 $\mathbf{l}$，因此不会影响最优解，可以丢掉。

所以平滑项在 QP 中贡献：

$$
P_{\text{smooth}}=TRT^T
$$

$$
\mathbf{q}_{\text{smooth}}=TR\mathbf{x}
$$

也就是说：

$$
J_{\text{smooth}}(\mathbf{l}) = \frac{1}{2}
\mathbf{l}^T
P_{\text{smooth}}
\mathbf{l} + \mathbf{q}_{\text{smooth}}^T\mathbf{l} + \text{const}
$$

这里的线性项很值得注意。它来自原始路径 $\mathbf{x}$。如果原始路径本身已经很光顺，$TR\mathbf{x}$ 对偏移的驱动力会比较弱；如果原始路径有明显折角，线性项会推动某些 $l_i$ 取非零值，从而降低局部二阶差分。

## 8. 横向误差正则项

如果只用平滑项，路径可能会在允许区间内尽量漂向更直的位置。为了让结果仍然贴近原始路径，需要惩罚横向偏移本身：

$$
J_{\text{lat}}(\mathbf{l}) = \frac{1}{2}w_l\|\mathbf{l}\|^2
$$

写成矩阵形式：

$$
J_{\text{lat}}(\mathbf{l}) = \frac{1}{2}
\mathbf{l}^T
(w_lI)
\mathbf{l}
$$

其中 $w_l$ 是横向误差权重。

它的作用有三个：

1. 抑制路径偏离原始参考线。
2. 在平滑和保形之间建立权衡。
3. 改善 Hessian 的条件数，让 QP 更稳定。

如果 $w_l$ 很大，优化结果会更接近原始路径，但平滑效果变弱。如果 $w_l$ 很小，路径更容易被拉顺，但也更可能偏离原始参考形状。

## 9. 最终 QP 模型

把平滑项和横向误差项相加，得到：

$$
J(\mathbf{l}) = \frac{1}{2}
\mathbf{l}^T
\left(
TRT^T+w_lI
\right)
\mathbf{l} + \left(
TR\mathbf{x}
\right)^T
\mathbf{l}
$$

由于：

$$
R=w_sP_0
$$

最终可以写成：

$$
\min_{\mathbf{l}\in\mathbb{R}^{N}}\quad \frac{1}{2}\mathbf{l}^T\left(w_sTP_0T^T+w_lI\right)\mathbf{l} + \left(w_sTP_0\mathbf{x}\right)^T\mathbf{l}
$$

这就是 Autoware Elastic Band 平滑的核心目标函数。

令：

$$
P=w_sTP_0T^T+w_lI
$$

$$
\mathbf{q}=w_sTP_0\mathbf{x}
$$

则标准 QP 形式为：

$$
\min_{\mathbf{l}}\quad \frac{1}{2}\mathbf{l}^TP\mathbf{l} + \mathbf{q}^T\mathbf{l}
$$

接下来只需要加上每个横向偏移量的上下界约束。

## 10. 横向约束：每个点只能在一条短线段上移动

Autoware Elastic Band 不允许路径点在平面中任意移动。第 $i$ 个点只能在法向线段上运动：

$$
\mathbf{p}_i(l_i)=\mathbf{r}_i+l_i\mathbf{n}_i
$$

并满足：

$$
-c_i\le l_i\le c_i
$$

其中 $c_i$ 是第 $i$ 个点允许的最大横向偏移距离。

从几何上看，第 $i$ 个点的可行区间端点是：

$$
\mathbf{p}_i^{u} = \mathbf{r}_i+c_i\mathbf{n}_i
$$

$$
\mathbf{p}_i^{l} = \mathbf{r}_i-c_i\mathbf{n}_i
$$

也就是：

$$
\mathbf{p}_i(l_i)\in\left[
\mathbf{r}_i-c_i\mathbf{n}_i,\,
\mathbf{r}_i+c_i\mathbf{n}_i
\right]
$$

把全部约束写成矩阵形式：

$$
\mathbf{l}_{\min}\le A\mathbf{l}\le \mathbf{l}_{\max}
$$

在这个问题里：

$$
A=I
$$

因此约束就是最简单的 box constraint：

$$
\begin{bmatrix}-c_0 \\ -c_1 \\ \vdots \\ -c_{N-1}\end{bmatrix}\le \begin{bmatrix}l_0 \\ l_1 \\ \vdots \\ l_{N-1}\end{bmatrix}\le \begin{bmatrix}c_0 \\ c_1 \\ \vdots \\ c_{N-1}\end{bmatrix}
$$

如果某个 $c_i=0$，则：

$$
l_i=0
$$

这个点被硬固定，完全不能横向移动。

## 11. fix、joint、smooth 三类移动范围

不同位置的路径点不应该拥有同样的自由度。Autoware Elastic Band 把横向移动范围大致分成三类：

$$
c_i=\begin{cases}
c_{\text{fix}}, & \text{固定区域} \\
c_{\text{joint}}, & \text{连接过渡区域} \\
c_{\text{smooth}}, & \text{主要平滑区域}
\end{cases}
$$

### fix 区域

fix 区域用于强约束关键点。典型位置包括：

1. 路径起点。
2. 局部路径包含目标点时的终点附近。
3. 需要与上一周期结果保持连续的前端区域。

如果：

$$
c_{\text{fix}}=0
$$

那么该区域点满足：

$$
l_i=0
$$

也就是完全固定。

起点固定很好理解：自车附近的轨迹不能突然横向跳动，否则会造成转向突变。终点固定也很重要，特别是当局部平滑片段已经包含目标点时，不能让平滑器改变目标位置；只固定最后一个点有时还不足以稳定末端方向，因此终点前一个真实点也常被收紧，从而稳定末端姿态。

### joint 区域

joint 区域是固定段和平滑段之间的过渡段。

如果从“完全固定”直接切换到“较自由平滑”，边界处可能出现新的折角。joint 区域使用较小的横向可动范围，让自由度逐渐释放：

$$
0\le c_{\text{fix}}\le c_{\text{joint}}\le c_{\text{smooth}}
$$

这不是数学上必须的硬规则，但它符合常见调参直觉。

### smooth 区域

smooth 区域是主要被优化的部分。这里允许更大的横向微调，让二阶差分代价有足够空间下降：

$$
|l_i|\le c_{\text{smooth}}
$$

如果 $c_{\text{smooth}}$ 太小，路径几乎无法被改变，平滑效果有限。如果太大，路径可能偏离上游规划意图。

## 12. 完整问题的一句话形式

现在可以把整个问题写成最简洁的形式：

$$
\mathbf{p}_i = \mathbf{r}_i+l_i\mathbf{n}_i
$$

$$
\min_{\mathbf{l}}\quad
w_s\sum_{k=1}^{N-2}
\left\|
\mathbf{p}_{k+1}
-2\mathbf{p}_k + \mathbf{p}_{k-1}
\right\|^2 + w_l\sum_{i=0}^{N-1}l_i^2
$$

subject to：

$$
-c_i\le l_i\le c_i,
\qquad
i=0,\dots,N-1
$$

再写成 QP：

$$
\min_{\mathbf{l}}\quad \frac{1}{2}\mathbf{l}^TP\mathbf{l} + \mathbf{q}^T\mathbf{l}
$$

subject to：

$$
\mathbf{l}_{\min}\le \mathbf{l}\le \mathbf{l}_{\max}
$$

其中：

$$
P=w_sTP_0T^T+w_lI
$$

$$
\mathbf{q}=w_sTP_0\mathbf{x}
$$

这就是 Autoware Elastic Band 平滑器最核心的数学模型。

## 13. 为什么它一定是 QP

QP 的基本形式是：

$$
\min_{\mathbf{v}}\quad \frac{1}{2}\mathbf{v}^TH\mathbf{v} + \mathbf{f}^T\mathbf{v}
$$

subject to：

$$
\mathbf{b}_{\text{lower}}\le C\mathbf{v}\le \mathbf{b}_{\text{upper}}
$$

Autoware Elastic Band 满足这个结构：

1. 变量 $\mathbf{l}$ 是连续向量。
2. 优化后的点 $\mathbf{p}_i=\mathbf{r}_i+l_i\mathbf{n}_i$ 对 $\mathbf{l}$ 是线性的。
3. 二阶差分对 $\mathbf{l}$ 仍然是线性的。
4. 线性表达的平方和就是二次目标。
5. 横向移动范围是线性不等式。
6. 固定点可以看成上下界相同的 box constraint。

所以这个问题不是非线性规划，而是标准二次规划。

此外，$P_0=D^TD$ 是半正定的，$w_sTP_0T^T$ 也是半正定的。如果 $w_l>0$，那么 $w_lI$ 会让 Hessian 更稳定：

$$
P=w_sTP_0T^T+w_lI
$$

通常会比纯平滑项更适合数值求解。

## 14. 为什么稀疏结构很适合在线规划

二阶差分只连接相邻三点，因此 $D$ 很稀疏。$D^TD$ 是五对角带状矩阵。这样的矩阵有两个工程优势：

1. 存储代价低。
2. 求解器可以利用稀疏结构快速迭代。

而且变量只有 $N$ 个横向偏移量，而不是 $2N$ 个全局坐标变量。对于在线规划来说，这个降维非常关键。

Elastic Band 因此可以在较高频率的规划循环中作为轻量几何平滑器使用。它不追求解决完整的动态避障和车辆动力学问题，而是先把参考路径变得更适合后续优化。

## 15. 在线运行时为什么要裁剪、重采样和补齐

虽然核心数学模型很短，但在线系统还需要一些工程步骤来保证问题稳定。

### 裁剪局部路径

平滑器通常只优化自车附近的一段路径。前向优化长度大致由点数和采样间隔决定：

$$
L_{\text{forward}} = N\Delta s
$$

其中 $N$ 是优化点数量，$\Delta s$ 是优化点弧长间隔。

自车后方也会保留一小段路径，用来保证输出轨迹和自车附近状态连续。这样做的目的不是改变历史路径，而是避免输出轨迹在自车附近突然断开。

### 插入固定点

在线规划是周期性运行的。如果每一帧都完全重新优化，轨迹前端可能在相邻周期之间轻微跳动。为了减少这种跳变，可以把上一周期已经优化过的前端轨迹作为当前周期的稳定锚点。

这相当于告诉优化器：

$$
\text{车前很近的轨迹不要随意改变。}
$$

这对方向盘稳定性尤其重要。

### 等弧长重采样

二阶差分默认相邻点间隔比较均匀。如果路径点间距忽大忽小，同样的二阶差分数值就不再代表同样的几何弯折程度。

因此 Elastic Band 会倾向于使用固定弧长间隔：

$$
\Delta s=\text{constant}
$$

这样二阶差分代价的物理含义更一致，QP 数值也更稳定。

### 补齐点数

在线优化器希望问题维度固定。若某次局部路径比较短，例如已经接近目标点，实际点数不足 $N$，可以用末尾点补齐到固定数量。

这样做的好处是：

1. 优化变量维度保持不变。
2. Hessian 和约束矩阵维度保持稳定。
3. 更容易复用上一轮求解状态，也就是 warm start。

补齐点不代表真实路径变长，它只是为了保持优化问题形状一致。最终输出时只使用真实路径对应的部分。

## 16. 求解后如何回到路径

QP 求出来的是：

$$
\mathbf{l}^{*} = \begin{bmatrix}
l_0^{*} & l_1^{*} & \cdots & l_{N-1}^{*}
\end{bmatrix}^{T}
$$

每个点的平滑后位置为：

$$
\mathbf{p}_i^{*} = \mathbf{r}_i+l_i^{*}\mathbf{n}_i
$$

也就是：

$$
x_i^{*}=x_i-\sin\theta_i\,l_i^{*}
$$

$$
y_i^{*}=y_i+\cos\theta_i\,l_i^{*}
$$

得到新的点列后，还需要根据几何形状重新计算路径朝向。因为点的位置已经改变，旧的航向角未必仍然与新路径切向一致。

Elastic Band 本身只处理几何形状。速度不是它优化的对象。输出轨迹的速度通常从输入路径继承或重新映射，因此可以理解为：

$$
\text{Elastic Band 改形状，不重新设计速度曲线。}
$$

## 17. 结果验证和失败回退

因为这是在线自动驾驶系统，平滑结果不能只看求解器是否返回一个数值，还需要判断结果是否可信。

常见验证方式是检查最大横向偏移：

$$
\max_i |l_i^{*}|\le l_{\text{valid}}
$$

其中 $l_{\text{valid}}$ 是允许的验证阈值。

如果优化失败、结果包含异常数值，或横向偏移超过验证阈值，就不应该发布这条新轨迹。更稳妥的方式是回退到上一轮成功结果；如果没有上一轮结果，再退回原始输入路径。

这体现了在线规划里的一个基本原则：

$$
\text{宁可输出保守且连续的轨迹，也不要输出突兀的新解。}
$$

## 18. 参数如何影响结果

Elastic Band 的参数可以按功能分成五组。

### 离散化参数

`num_points` 决定优化点数量 $N$。

`delta_arc_length` 决定优化点间隔 $\Delta s$。

前向优化长度近似为：

$$
L_{\text{forward}}=N\Delta s
$$

$N$ 太小，优化范围短，平滑效果有限；$N$ 太大，计算量增加。$\Delta s$ 太大，路径细节表达粗糙；$\Delta s$ 太小，变量更多，且二阶差分对噪声更敏感。

### 权重参数

`smooth_weight` 对应 $w_s$：

$$
w_s
\sum_{k=1}^{N-2}
\left\|
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
\right\|^2
$$

$w_s$ 越大，路径越倾向于降低折角和锯齿。

`lat_error_weight` 对应 $w_l$：

$$
w_l\sum_{i=0}^{N-1}l_i^2
$$

$w_l$ 越大，路径越贴近原始输入。

两者的关系可以理解为：

$$
\frac{w_s}{w_l}
\text{ 越大，越追求平滑；}
\qquad
\frac{w_l}{w_s}
\text{ 越大，越保守贴原路径。}
$$

### clearance 参数

`clearance_for_fix` 对应 $c_{\text{fix}}$。

`clearance_for_joint` 对应 $c_{\text{joint}}$。

`clearance_for_smooth` 对应 $c_{\text{smooth}}$。

它们本质上定义变量边界：

$$
-c_i\le l_i\le c_i
$$

如果 clearance 太小，优化器没有足够空间消除折角。如果 clearance 太大，结果可能偏离上游路径意图。一般来说，固定区最小，连接区适中，平滑区最大。

`num_joint_points` 决定固定区和平滑区之间有多少过渡点。过渡点越多，前端连接越柔和，但真正自由平滑的区域会相对变短。

### 求解器参数

`max_iteration` 控制 QP 最大迭代次数。

`eps_abs` 和 `eps_rel` 控制绝对和相对收敛精度。

精度越严格，结果可能更精细，但计算时间增加。在线规划里通常需要在稳定性和实时性之间折中。

### 在线稳定参数

`enable_warm_start` 表示是否使用上一轮优化结果辅助当前求解。由于相邻规划周期的问题通常很相似，warm start 可以减少迭代次数，并提高轨迹连续性。

`enable_optimization_validation` 和 `validation.max_error` 用于检查优化结果是否超过允许横向误差。它们不是改变目标函数，而是在输出前做安全过滤。

重规划相关参数用于判断是否需要重新执行平滑。如果输入路径、自车位置或目标变化很小，可以复用上一轮结果；如果变化超过阈值，就需要重新优化。

## 19. 它和传统 Elastic Band 的区别

经典机器人 Elastic Band 通常会把路径看成一条受力的弹性带：

$$
\mathbf{p}_i^{(t+1)} = \mathbf{p}_i^{(t)} + \alpha
\left(
\mathbf{f}^{\text{internal}}_i + \mathbf{f}^{\text{obstacle}}_i
\right)
$$

其中 $\mathbf{f}^{\text{internal}}$ 表示弹性内力，让路径更短更平滑；$\mathbf{f}^{\text{obstacle}}$ 表示障碍物斥力，让路径远离障碍物。

Autoware 的 Elastic Band 明显不同：

1. 它不是力迭代，而是 QP。
2. 它不显式构造障碍物斥力。
3. 它不负责碰撞检查。
4. 它限制点只能沿局部法向移动。
5. 它的主要目标是让后续基于 Frenet 的优化更稳定。

所以不要把 Autoware Elastic Band 理解成完整避障规划器。更准确的说法是：

$$
\text{它是路径优化前的几何平滑预处理器。}
$$

## 20. 为什么它不做碰撞检查

这点非常容易误解。

Elastic Band 的输出路径可能比输入路径更光顺，但它不保证一定在可行驶区域内，也不保证不碰撞障碍物。原因是它的约束只描述了每个点的横向可移动范围：

$$
-c_i\le l_i\le c_i
$$

这里没有障碍物距离约束，也没有车辆 footprint 与道路边界的碰撞约束。

这种设计是有意的。Autoware 中不同模块有不同职责：

1. Elastic Band 负责轻量几何平滑。
2. 后续轨迹优化或可行驶区域检查负责可行性和碰撞相关问题。
3. 速度规划负责速度、停车点、减速度和舒适性。

职责拆开后，Elastic Band 可以保持简单、稳定、快速。

## 21. 它为什么能让后续 Frenet / MPT 更稳定

Frenet 坐标系依赖参考路径。给定参考路径后，车辆状态会被描述成：

$$
\text{纵向位置 }s,\quad
\text{横向误差 }l,\quad
\text{航向误差 }e_{\theta}
$$

如果参考路径有局部锯齿，则切向和法向会跳变：

$$
\mathbf{t}_i=\begin{bmatrix}
\cos\theta_i \\
\sin\theta_i
\end{bmatrix},
\qquad
\mathbf{n}_i=\begin{bmatrix}
-\sin\theta_i \\
\cos\theta_i
\end{bmatrix}
$$

航向角 $\theta_i$ 一旦抖动，横向误差、航向误差和曲率估计都会跟着抖动。后续优化器会看到一个不稳定的参考系，求解出的转向角也可能变得不稳定。

Elastic Band 通过降低二阶差分：

$$
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
$$

让路径局部方向变化更平顺，从而让 Frenet frame 更稳定。它不是直接优化车辆动力学，但它改善了后续动力学优化的输入条件。

## 22. 一个从零理解的学习路线

如果你完全不了解 Autoware，可以按下面这条路线学习 Elastic Band。

第一步，理解路径是离散点列：

$$
\mathbf{r}_0,\mathbf{r}_1,\dots,\mathbf{r}_{N-1}
$$

第二步，理解平滑就是减少相邻线段方向变化：

$$
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
$$

第三步，理解二阶差分平方和可以写成矩阵二次型：

$$
\|D\mathbf{x}\|^2=\mathbf{x}^TD^TD\mathbf{x}
$$

第四步，理解 Autoware 不直接优化 $x,y$，而是优化横向偏移：

$$
\mathbf{p}_i=\mathbf{r}_i+l_i\mathbf{n}_i
$$

第五步，把全局平滑代价投影到横向偏移变量上：

$$
\mathbf{z}(\mathbf{l})=\mathbf{x}+T^T\mathbf{l}
$$

第六步，展开得到 QP：

$$
\min_{\mathbf{l}}\quad \frac{1}{2}\mathbf{l}^T\left(w_sTP_0T^T+w_lI\right)\mathbf{l} + \left(w_sTP_0\mathbf{x}\right)^T\mathbf{l}
$$

第七步，加上横向 box constraint：

$$
-c_i\le l_i\le c_i
$$

理解完这七步，就已经抓住了 Autoware Elastic Band 的主干。

## 23. 优点和局限

### 优点

它很稳定，因为目标是二次函数，约束是线性边界。

它很快，因为 Hessian 来自二阶差分，具有稀疏带状结构。

它很保守，因为点只能沿法向小幅移动，不会彻底改变路径拓扑。

它很适合在线系统，因为可以固定优化维度、使用 warm start，并在失败时回退到上一轮结果。

### 局限

它不负责避障。

它不保证输出路径一定 collision-free。

它不能处理大尺度拓扑变化。

它依赖上游参考路径质量。如果原始路径很差，或者横向可动范围很小，平滑效果会受限。

它不是动力学优化。车辆转向角、转向角速度、轮胎约束、道路边界和障碍物约束需要交给后续模块处理。

## 24. 最后记住三个公式

第一个，平滑目标：

$$
J_{\text{smooth}} = \sum_{k=1}^{N-2}
\left\|
\mathbf{p}_{k+1}-2\mathbf{p}_k+\mathbf{p}_{k-1}
\right\|^2
$$

第二个，Autoware 的横向偏移参数化：

$$
\mathbf{p}_i = \mathbf{r}_i+l_i\mathbf{n}_i
$$

第三个，最终 QP：

$$
\min_{\mathbf{l}}\quad \frac{1}{2}\mathbf{l}^T\left(w_sTP_0T^T+w_lI\right)\mathbf{l} + \left(w_sTP_0\mathbf{x}\right)^T\mathbf{l}
$$

subject to：

$$
-c_i\le l_i\le c_i
$$

如果只能用一句话总结：

$$
\text{Autoware Elastic Band 通过优化每个路径点的法向横向偏移，在固定横向走廊内最小化离散二阶差分，从而得到更平顺但不负责避障的参考路径。}
$$
