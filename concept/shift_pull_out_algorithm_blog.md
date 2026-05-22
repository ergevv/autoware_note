# 从路边平滑驶入车道中心：Shift Pull Out 启动规划算法教程

Shift Pull Out 是路边起步规划里最自然、最像人类驾驶的一类算法。它不把车辆从路边“硬拐”进车道，也不依赖两段急转圆弧，而是沿着目标车道方向向前走，同时把车辆相对车道中心线的横向偏移逐渐消掉。直观地说，它要解决的问题是：

$$
\text{从 } d_0 \neq 0 \text{ 的路边位置，平滑过渡到 } d=0 \text{ 的车道中心}
$$

这篇教程专门围绕 Shift Pull Out 展开。它会从道路坐标、起步点搜索、横向 jerk 曲线、纵向距离、曲率约束、路径平移、静态碰撞检查、动态安全监督一路讲下去。读完后，即使不了解 Autoware，也应该能独立理解这类“路边平滑驶出”算法怎样设计。

## 1. Shift Pull Out 到底在解决什么问题

设车辆当前停在路肩或道路边缘，车头方向大致与道路方向一致，但车身中心不在目标车道中心线上。目标是让车辆从静止或低速状态起步，逐渐合入车道中心线，并继续沿车道向前行驶。

如果把目标车道中心线看作一条参考曲线，车辆初始状态可以写成：

$$
P_0=(x_0,y_0,\psi_0)
$$

车辆相对车道中心线的横向偏移为：

$$
d_0
$$

Shift Pull Out 的目标不是重新设计一条完全自由的曲线，而是在道路中心线基础上构造一个横向偏移函数 $\ell(s)$：

$$
P_{\text{shift}}(s)
=
C(s)+\ell(s)n(s)
$$

其中 $C(s)$ 是道路中心线，$s$ 是沿中心线的弧长，$n(s)$ 是中心线法向量。为了从路边驶入中心线，横向偏移函数需要满足：

$$
\ell(s_{\text{start}})=d_0
$$

$$
\ell(s_{\text{end}})=0
$$

并且在过渡过程中足够平滑，不能出现横向速度、横向加速度或曲率的突变。

因此，Shift Pull Out 的核心可以浓缩成一句话：

$$
\text{沿目标车道向前推进，同时用受限 jerk 的横向偏移曲线把 } d_0 \text{ 平滑降到 } 0
$$

它适合的场景是：车辆前方有足够距离，车辆姿态大体顺着道路方向，靠一条连续平滑的横移路径就能完成起步。相比几何圆弧驶出，它更舒适；相比自由空间搜索，它更稳定、更符合车道语义。

## 2. 先把道路变成弧长坐标

要描述“车辆离车道中心线多远”，先需要一套道路坐标。给定道路中心线：

$$
C=\{c_0,c_1,\dots,c_N\}
$$

其中：

$$
c_i=(x_i,y_i)
$$

中心线的累计弧长为：

$$
S_0=0
$$

$$
S_i=\sum_{k=0}^{i-1}\|c_{k+1}-c_k\|
$$

对任意点 $p=(x,y)$，找到它在中心线某段 $[c_i,c_{i+1}]$ 上的最近投影。令：

$$
u_i=c_{i+1}-c_i
$$

投影比例为：

$$
\lambda^*
=
\operatorname{clip}
\left(
\frac{(p-c_i)\cdot u_i}{\|u_i\|^2},
0,
1
\right)
$$

投影点为：

$$
q_i=c_i+\lambda^*u_i
$$

沿中心线弧长坐标为：

$$
s=S_i+\lambda^*\|u_i\|
$$

横向偏移用带符号距离表示。若二维叉积：

$$
u_i\times(p-q_i)
=
u_{i,x}(p_y-q_{i,y})-u_{i,y}(p_x-q_{i,x})
$$

则：

$$
d=
\operatorname{sign}
\left(
u_i\times(p-q_i)
\right)
\|p-q_i\|
$$

这样，车辆初始位姿相对道路中心线就可以写成：

$$
(s_0,d_0,\psi_0)
$$

Shift Pull Out 只关心一个核心变化：

$$
d_0 \rightarrow 0
$$

但车辆真实路径仍然在二维平面上，因此每个 $s$ 处的实际位置由中心线点和横向偏移共同决定：

$$
p(s)=C(s)+\ell(s)n(s)
$$

若中心线切向为：

$$
t(s)=
\begin{bmatrix}
\cos\theta(s)\\
\sin\theta(s)
\end{bmatrix}
$$

则左法向量可写成：

$$
n(s)=
\begin{bmatrix}
-\sin\theta(s)\\
\cos\theta(s)
\end{bmatrix}
$$

当 $d_0>0$ 时，车辆在中心线左侧；当 $d_0<0$ 时，车辆在中心线右侧。距离计算通常使用 $L_y=|d_0|$，方向由 $d_0$ 的符号单独保留。

## 3. Shift Pull Out 在启动规划中的位置

路边起步不是只生成一条 Shift 路径就结束。完整启动规划会先判断是否需要启动规划，再搜索可能的起步位姿，最后才尝试 Shift Pull Out。

车辆通常只有在以下条件大体满足时才需要启动规划：

$$
|d_0| \ge d_{\text{middle}}
$$

也就是车辆不在车道中心附近；同时车辆要足够靠近原始起点：

$$
\|p_{\text{ego}}-p_{\text{start}}\|\le d_{\text{arrive}}
$$

车辆还要停稳：

$$
\|v_{\text{ego}}\|<v_{\text{stop}}
$$

并且目标点不能在同一路段的车辆后方：

$$
s_{\text{goal}}-s_{\text{ego}}\ge0
$$

这些条件保证 Shift Pull Out 处理的是“从路边向前驶入车道”的问题，而不是普通车道跟随、倒车到目标点或行驶中换道。

如果当前位置直接 Shift 不安全，启动规划还会向后搜索候选起步点。这样 Shift Pull Out 可以从当前位置开始，也可以从倒车后的某个更有利位置开始。

因此，Shift Pull Out 的输入可以抽象为：

$$
\left(
P_{\text{start}},
P_{\text{goal}},
\mathcal{L}_{\text{road}},
\mathcal{O},
\Theta
\right)
$$

其中 $P_{\text{start}}$ 是当前或倒车后的候选起步位姿，$P_{\text{goal}}$ 是路线目标，$\mathcal{L}_{\text{road}}$ 是目标道路车道集合，$\mathcal{O}$ 是感知到的障碍物集合，$\Theta$ 是车辆和规划参数。

输出是一条或多条候选 Shift 路径：

$$
\mathcal{P}_{\text{shift}}=
\{P_0,P_1,\dots,P_M\}
$$

对 Shift Pull Out 来说，通常只需要一个连续子路径。若前面先倒车，那么倒车路径属于启动规划外层状态机，Shift 自身仍然是一段向前横移路径。

## 4. 倒车候选点为什么会影响 Shift 路径

Shift Pull Out 需要前方有足够纵向距离。如果前方有静止车辆，或者目标车道弯曲太急，从当前位置横移可能无法满足碰撞和曲率约束。人类驾驶员会先倒一点再打方向，算法也一样。

设原始起点在路边参考线上的弧长为 $s_0$，最大允许倒车距离为 $D_{\text{back}}$，倒车搜索分辨率为 $\Delta b$。候选倒车距离为：

$$
b_k=k\Delta b
$$

候选起步点为：

$$
P_k=C_{\text{side}}(s_0-b_k)
$$

其中 $C_{\text{side}}$ 是与车辆当前横向位置对齐的路边参考线。它可以由车道中心线横向平移得到：

$$
C_{\text{side}}(s)=C(s)+d_0n(s)
$$

实际可倒车距离还受车道后端和车辆后悬限制。设车辆参考点到车尾距离为 $l_{\text{rear}}$，当前弧长为 $s_0$，则：

$$
D_{\text{allow}}
=
\operatorname{clamp}
\left(
s_0-l_{\text{rear}},
0,
D_{\text{back}}
\right)
$$

只搜索满足：

$$
0\le b_k\le D_{\text{allow}}
$$

的候选点。

候选集合里始终应该包含当前位置。即使允许倒车，也要先尝试“从当前位置直接 Shift”，因为这是最短、最自然的方案。可以把候选集合写成：

$$
\mathcal{C}_{\text{start}}
=
\{P_{\text{current}}\}
\cup
\{P_k\mid 0<b_k\le D_{\text{allow}}\}
$$

为了判断选中的候选是否真的需要倒车，还会把当前车辆位置投影到路边参考线上，得到精修起点 $P_{\text{refined}}$。若某个候选满足：

$$
\|P_k-P_{\text{refined}}\|<\epsilon_{\text{back}}
$$

则认为不需要倒车，可以直接执行 Shift Pull Out。否则先生成一条倒车路径到 $P_k$，停稳后再执行 Shift：

$$
\|P_k-P_{\text{refined}}\|\ge\epsilon_{\text{back}}
\quad\Rightarrow\quad
\text{backward first, then Shift}
$$

候选起点也不应在每个规划周期疯狂变化。为了避免“刚想倒车又重新选择起点”的抖动，候选更新通常带有时间间隔：

$$
t_{\text{now}}-t_{\text{last-update}}
\ge
T_{\text{update}}
$$

只有超过这个间隔，才重新搜索倒车起步点。

每个候选点还要检查静止障碍物。设静止障碍物集合为：

$$
\mathcal{O}_{\text{stop}}
=
\{o_i\mid \|v_i\|<v_{\text{moving}},\ o_i\in\mathcal{L}_{\text{pullout}}\}
$$

候选点还要离路边车道末端保持足够距离。设起步相关车道总长度为 $L_{\text{lane}}$，候选点弧长为 $s(P_k)$，则到车道末端的剩余距离为：

$$
D_{\text{lane-end}}
=
L_{\text{lane}}-s(P_k)
$$

如果：

$$
D_{\text{lane-end}}<D_{\text{ignore-end}}
$$

则候选点太靠近车道末端，不适合作为 Shift 起点，应跳过。

如果候选点处车辆足迹与前方静止物体距离小于前方安全余量：

$$
\operatorname{dist}
\left(
F(P_k),
\mathcal{P}_{\text{front}}
\right)
<
m_{\text{front}}
$$

则这个候选点不能作为 Shift 起点。若车辆向后搜索时已经与后方静止物体发生最小余量冲突：

$$
\operatorname{dist}
\left(
F(P_k),
\mathcal{P}_{\text{rear}}
\right)
<
m_{\min}
$$

则更远的候选通常也不可用，可以停止继续向后搜索。

这一步非常重要：Shift Pull Out 本身是向前横移算法，但它的可行性高度依赖起点位置。一个前方空间不足的当前位置，倒车两三米后可能立刻变成可行。

## 5. 候选搜索的优先级：先安全余量，再路径策略

Shift Pull Out 和其它驶出方式一起参与候选搜索。安全余量优先级通常高于路径类型优先级。

设静态碰撞余量集合为：

$$
\mathcal{M}=\{m_1,m_2,\dots,m_R\}
$$

并且：

$$
m_1>m_2>\dots>m_R
$$

搜索会先尝试最大余量 $m_1$。只有当没有候选路径能满足 $m_1$ 时，才退到 $m_2$。这相当于优先求更保守的静态安全：

$$
\max m
\quad
\text{s.t. path is feasible under margin }m
$$

在同一个余量下，有两种常见策略。

第一种是“优先高效路径”。如果 Shift 被认为更自然，就先对所有候选起点尝试 Shift：

$$
(\text{Shift},P_0),
(\text{Shift},P_1),
\dots,
(\text{Shift},P_N)
$$

如果都失败，再尝试其它路径类型。它的效果是：只要 Shift 能成功，就尽量选择 Shift，即使需要多倒一点车。

第二种是“优先少倒车”。它先固定候选起点，再尝试路径类型：

$$
(\text{Shift},P_0),
(\text{Other},P_0),
(\text{Shift},P_1),
(\text{Other},P_1),
\dots
$$

它的效果是：尽量从离当前位置最近的位置驶出。

对于 Shift Pull Out，可以把最终选择理解成一个词典序问题：

$$
\operatorname*{arg\,first}_{m,\pi}
\left[
\text{Shift path feasible}
\right]
$$

其中 $m$ 按从大到小排序，$\pi$ 是候选起点和路径类型的策略顺序。

## 6. 构造道路参考路径

Shift Pull Out 不是直接从车辆位置画曲线，而是先取一段道路中心线作为参考路径。设候选起点在道路中心线上的弧长为：

$$
s_{\text{start}}
$$

为了让路径在起点前后都有足够上下文，参考路径通常从起点后方一段距离开始：

$$
s_{\text{ref,start}}
=
\max(s_{\text{start}}-D_{\text{back-ref}},0)
$$

向前延伸到：

$$
s_{\text{ref,end}}
=
s_{\text{ref,start}}+D_{\text{forward}}
$$

如果路线目标点位于这个前向范围内，则参考路径终点可以提前收在目标点：

$$
s_{\text{ref,end}}=s_{\text{goal}}
$$

否则使用固定前向长度：

$$
s_{\text{ref,end}}=s_{\text{ref,start}}+D_{\text{forward}}
$$

然后对中心线进行重采样，使相邻路径点间隔约为：

$$
\Delta s_{\text{path}}
$$

重采样后的参考路径记为：

$$
C_{\text{ref}}=\{C_0,C_1,\dots,C_M\}
$$

每个点都携带道路语义，例如所在车道、速度限制或下游期望速度。Shift Pull Out 之后生成的路径会继承这条参考路径的道路方向和车道语义，只是在几何位置上横向偏移。

## 7. 横向偏移量：Shift Length

候选起点 $P_{\text{start}}$ 到道路中心线的横向距离就是 Shift Pull Out 需要消除的偏移：

$$
d_0=\operatorname{LatOffset}(C_{\text{ref}},P_{\text{start}})
$$

它的符号表示车辆在中心线哪一侧，绝对值表示需要横移的距离：

$$
L_y=|d_0|
$$

如果：

$$
L_y<\epsilon_d
$$

说明车辆已经几乎在中心线上。此时没有必要生成横移曲线，可以直接使用道路中心线作为非横移路径：

$$
\ell(s)=0
$$

这是一种特殊的“零 Shift”情况。它避免在横向距离极小时生成数值不稳定或没有意义的曲线。

正常情况下，Shift 路径需要满足：

$$
\ell(0)=d_0
$$

$$
\ell(S)=0
$$

其中 $S$ 是横移完成所需的纵向距离。接下来的关键问题就是：$S$ 应该多长？

## 8. 横向 jerk 约束：为什么横移时间不是随便给的

车辆从路边平滑驶入车道中心，不能让横向加速度突然变化。横向运动通常使用 jerk 受限的 S 曲线。令横向位移为 $L_y$，横向 jerk 上限为 $j_y$，横向加速度上限为 $a_y$。

先考虑半段横移。半段由三部分组成：

1. 横向 jerk 为 $+j_y$，持续 $t_j$，横向加速度从 $0$ 增加到 $a_y$；
2. 横向 jerk 为 $0$，持续 $t_a$，横向加速度保持 $a_y$；
3. 横向 jerk 为 $-j_y$，持续 $t_j$，横向加速度降回 $0$。

由于：

$$
a_y=j_yt_j
$$

所以：

$$
t_j=\frac{a_y}{j_y}
$$

第一小段的横向位移为：

$$
y_1=\frac{1}{6}j_yt_j^3
$$

第二小段开始时横向速度为：

$$
v_{y,1}=\frac{1}{2}j_yt_j^2
$$

第二小段位移为：

$$
y_2=v_{y,1}t_a+\frac{1}{2}a_yt_a^2
=
\frac{1}{2}j_yt_j^2t_a
\frac{1}{2}j_yt_jt_a^2
$$

第三小段开始时横向速度为：

$$
v_{y,2}=v_{y,1}+a_yt_a
=
\frac{1}{2}j_yt_j^2+j_yt_jt_a
$$

第三小段位移为：

$$
y_3
=
v_{y,2}t_j
\frac{1}{2}a_yt_j^2
-\frac{1}{6}j_yt_j^3
$$

代入 $a_y=j_yt_j$ 得：

$$
y_3
=
\frac{5}{6}j_yt_j^3
j_yt_j^2t_a
$$

半段位移为：

$$
\frac{L_y}{2}
=
y_1+y_2+y_3
$$

整理得到：

$$
\frac{L_y}{2}
=
j_yt_j^3
\frac{3}{2}j_yt_j^2t_a
\frac{1}{2}j_yt_jt_a^2
$$

两边乘以 $2$：

$$
L_y
=
2j_yt_j^3
3j_yt_j^2t_a
j_yt_jt_a^2
$$

再代入 $t_j=a_y/j_y$：

$$
L_y
=
\frac{2a_y^3}{j_y^2}
\frac{3a_y^2}{j_y}t_a
a_yt_a^2
$$

这是关于 $t_a$ 的二次方程。解得：

$$
t_a
=
\frac{
\sqrt{
a_y^2+\frac{4j_y^2L_y}{a_y}
}
-3a_y
}{
2j_y
}
$$

总横移时间为：

$$
T_{\text{shift}}
=
4t_j+2t_a
$$

如果算出来 $t_a<0$，表示横移距离太短，横向加速度还没达到上限就需要反向 jerk。此时采用三角 jerk 曲线：

$$
t_a=0
$$

$$
t_j=
\left(
\frac{L_y}{2j_y}
\right)^{1/3}
$$

于是：

$$
T_{\text{shift}}=4t_j
$$

这个推导说明：横移时间由横移距离、横向 jerk 和横向加速度共同决定。横移越大，所需时间越长；允许的横向加速度越大，所需时间越短；横向 jerk 越大，曲线响应越快，但舒适性越差。

## 9. 从横移时间得到纵向加速度和终端速度

横移曲线需要 $T_{\text{shift}}$ 秒完成。车辆同时沿道路向前起步。设参考道路在起点附近的期望速度为：

$$
v_{\text{road}}
$$

如果车辆从低速或静止开始，用恒定纵向加速度近似，则希望在 $T_{\text{shift}}$ 内接近道路期望速度：

$$
a_x=
\frac{v_{\text{road}}}{T_{\text{shift}}}
$$

但纵向加速度不能无限大，因此做限幅：

$$
a_x=
\operatorname{clamp}
\left(
\frac{v_{\text{road}}}{T_{\text{shift}}},
0,
a_{x,\max}
\right)
$$

横移结束时的终端速度为：

$$
v_{\text{term}}=a_xT_{\text{shift}}
$$

纵向加速距离为：

$$
D_{\text{acc}}
=
\frac{1}{2}a_xT_{\text{shift}}^2
$$

这只是一个下界。它保证车辆在横移完成时间内的纵向运动与速度规划一致，但还没有考虑曲率约束和最小驶出距离。

## 10. 曲率约束：横移距离太短会导致路径太弯

如果车辆在很短的纵向距离内完成较大的横向移动，路径会很弯，方向变化很急，控制和乘坐体验都不好。因此 Shift Pull Out 会给路径曲率设置上限：

$$
|\kappa|\le\kappa_{\max}
$$

可以用双圆弧近似推导所需纵向距离。设横向位移为 $L_y$，纵向距离为 $D$。用两段对称圆弧完成从偏移位置到中心线的过渡时，近似几何关系为：

$$
R\approx
\frac{D^2+L_y^2}{4L_y}
$$

曲率为：

$$
\kappa=\frac{1}{R}
\approx
\frac{4L_y}{D^2+L_y^2}
$$

要求：

$$
\frac{4L_y}{D^2+L_y^2}
\le
\kappa_{\max}
$$

得到：

$$
D^2+L_y^2
\ge
\frac{4L_y}{\kappa_{\max}}
$$

因此：

$$
D
\ge
\sqrt{
\frac{4L_y}{\kappa_{\max}}-L_y^2
}
$$

记为：

$$
D_{\kappa}
=
\sqrt{
\max
\left(
\frac{4L_y}{\kappa_{\max}}-L_y^2,
0
\right)
}
$$

最终，Shift Pull Out 所需的驶出距离至少要满足三类约束：

$$
D_{\text{pullout}}
=
\max
\left(
D_{\text{acc}},
D_{\kappa},
D_{\min}
\right)
$$

其中 $D_{\min}$ 是人为设置的最小横移距离。这个公式非常重要，它把舒适性、可控性和工程下限统一到一个纵向距离中。

## 11. 为什么中心线距离和实际 Shift 距离不一样

上面算出的 $D_{\text{pullout}}$ 是实际横移路径上需要的长度。但路径生成时，终点通常在道路中心线上按弧长选取。若道路中心线是弯的，中心线弧长和横向偏移后的路径弧长并不相同。

对一条曲率为 $\kappa(s)$ 的曲线，其横向偏移 $d$ 后的平行曲线弧长微分近似为：

$$
ds_d\approx |1-\kappa(s)d|ds
$$

不同坐标约定下符号可能写成 $1+\kappa d$，但核心事实不变：弯道内侧和外侧的弧长不一样。

因此，需要把目标实际路径长度 $D_{\text{pullout}}$ 换算为中心线上的弧长距离。可以把中心线分成小段。第 $i$ 段中心线长度为：

$$
\Delta s_i
$$

曲率为：

$$
\kappa_i
$$

横向偏移为：

$$
d_0
$$

则偏移后路径长度可近似累计：

$$
\Delta s_{d,i}
\approx
g(\kappa_i,d_0)\Delta s_i
$$

其中 $g$ 是曲率修正因子。一种工程上常用的分段近似是：

$$
g(\kappa,d)
=
\begin{cases}
1-\kappa d, & \kappa<0\\
\frac{1}{1+\kappa d}, & \kappa\ge0
\end{cases}
$$

从起点开始累计：

$$
S_d(n)=\sum_{i=0}^{n}\Delta s_{d,i}
$$

找到第一个满足：

$$
S_d(n)\ge D_{\text{pullout}}
$$

的位置。若目标长度落在某一段内部，则做线性插值。剩余偏移长度为：

$$
r=D_{\text{pullout}}-S_d(n-1)
$$

对应中心线增量为：

$$
\Delta s=
\begin{cases}
\frac{r}{1-\kappa d}, & \kappa<0\\
r(1+\kappa d), & \kappa\ge0
\end{cases}
$$

最后得到中心线上的候选横移终点距离：

$$
D_{\text{center}}
$$

为了避免近似误差让横移距离过短，实际会取：

$$
D_{\text{center}}
\leftarrow
\max
\left(
D_{\text{center}},
D_{\text{pullout}}
\right)
$$

这一步保证在弯道上也不会因为弧长换算过于乐观而生成太短、太急的 Shift 路径。

## 12. 横移终点为什么要避开高曲率位置

如果横移刚结束的位置恰好在道路急弯处，车辆会在完成横向合流的同时面临较大的道路曲率。这会让路径末端曲率变大，控制不自然。

因此，算法不会简单把：

$$
s_{\text{end}}=s_{\text{start}}+D_{\text{center}}
$$

作为唯一终点，而是继续检查该点之后的道路曲率。设终点曲率阈值为：

$$
\kappa_{\text{end}}
$$

从 $D_{\text{center}}$ 之后开始向前搜索，找到第一个满足：

$$
|\kappa(s)|<\kappa_{\text{end}}
$$

的位置作为横移终点。

如果在前方范围内没有找到足够小曲率的点，则选择 $D_{\text{center}}$ 之后曲率最小的位置：

$$
s_{\text{end}}
=
\operatorname*{arg\,min}_{s>D_{\text{center}}}
|\kappa(s)|
$$

这样做的直觉是：宁可稍微延长横移，也尽量让车辆在更平顺的道路几何上完成合流。

## 13. 横向偏移函数怎样生成

现在有了起点、终点和需要消除的横向偏移：

$$
s=0\quad\Rightarrow\quad \ell=d_0
$$

$$
s=S\quad\Rightarrow\quad \ell=0
$$

其中：

$$
S=s_{\text{end}}-s_{\text{start}}
$$

理想的偏移函数应满足：

$$
\ell(0)=d_0
$$

$$
\ell(S)=0
$$

并且：

$$
\ell'(0)=\ell'(S)=0
$$

$$
\ell''(0)=\ell''(S)=0
$$

这样车辆在横移开始和结束时不会出现横向速度或横向加速度突变。

当不需要显式考虑横向加速度饱和时，可以用四段恒定 jerk 曲线。令横移总时间为 $T$，每段时间为：

$$
\tau=\frac{T}{4}
$$

jerk 符号序列为：

$$
(+j_y,\ -j_y,\ -j_y,\ +j_y)
$$

在这种对称曲线中，总横向位移满足：

$$
L_y=2j_y\tau^3
$$

第一段结束时的横向位移为：

$$
y_1=\frac{1}{6}j_y\tau^3
=
\frac{L_y}{12}
$$

第三段结束，也就是 $t=3T/4$ 时，横向位移为：

$$
y_3=\frac{11L_y}{12}
$$

所以可以用以下基准点构造横向偏移样条：

$$
(0,0)
$$

$$
\left(\frac{S}{4},\frac{L_y}{12}\right)
$$

$$
\left(\frac{3S}{4},\frac{11L_y}{12}\right)
$$

$$
(S,L_y)
$$

如果我们把路径理解为“从路边偏移 $d_0$ 回到中心线 $0$”，则偏移量方向要反过来：

$$
\ell(0)=d_0,\quad
\ell(S)=0
$$

可以用符号 $\sigma=\operatorname{sign}(d_0)$ 表示方向，实际横移距离用 $L_y=|d_0|$。一条概念上的偏移函数可写为：

$$
\ell(s)
=
d_0-\sigma y(s)
$$

其中 $y(s)$ 从 $0$ 平滑增加到 $L_y$。

## 14. 当横向加速度限制必须生效时

如果给定纵向距离 $S$、初始纵向速度 $v_0$ 和纵向加速度 $a_x$，车辆走完整个横移区间所需时间为：

$$
T=
\begin{cases}
\frac{-v_0+\sqrt{v_0^2+2a_xS}}{a_x}, & a_x>0\\
\frac{S}{v_0}, & a_x=0,\ v_0>0
\end{cases}
$$

若不考虑横向加速度上限，四段 jerk 曲线的峰值横向加速度为：

$$
a_{y,\text{peak}}
=
\frac{8L_y}{T^2}
$$

如果：

$$
a_{y,\text{peak}}<a_{y,\lim}
$$

说明当前横移足够温和，不需要显式压住横向加速度，使用四段基准点即可。

如果：

$$
a_{y,\text{peak}}\ge a_{y,\lim}
$$

则需要让横向加速度达到上限后保持一段时间，曲线进入带平台的 S 曲线。总时间关系仍然是：

$$
T=4t_j+2t_a
$$

横向距离关系为：

$$
L_y
=
2j_yt_j^3
3j_yt_j^2t_a
j_yt_jt_a^2
$$

并且：

$$
a_{y,\lim}=j_yt_j
$$

由这些关系可以解出：

$$
t_j
=
\frac{T}{2}
-
\frac{2L_y}{a_{y,\lim}T}
$$

$$
t_a
=
\frac{4L_y}{a_{y,\lim}T}
-
\frac{T}{2}
$$

$$
j_y
=
\frac{
2a_{y,\lim}^2T
}{
a_{y,\lim}T^2-4L_y
}
$$

如果出现：

$$
t_j<0
\quad\text{or}\quad
t_a<0
\quad\text{or}\quad
j_y<0
$$

说明当前距离、速度和加速度限制组合不可行或数值不合理，就退回不显式考虑加速度平台的平滑曲线。

带平台的曲线会产生更多基准点。设横移方向符号为：

$$
\sigma=\operatorname{sign}(d_0)
$$

横向位移关键点为：

$$
l_1=
\sigma\frac{1}{6}j_yt_j^3
$$

$$
l_2=
\sigma
\left(
\frac{1}{6}j_yt_j^3
+\frac{1}{2}j_yt_at_j^2
+\frac{1}{2}j_yt_a^2t_j
\right)
$$

$$
l_3=
\sigma
\left(
j_yt_j^3
+\frac{3}{2}j_yt_at_j^2
+\frac{1}{2}j_yt_a^2t_j
\right)
$$

这里 $l_3$ 正好是半程横移：

$$
l_3=\sigma\frac{L_y}{2}
$$

后半段对称，关键点为：

$$
l_5=
\sigma
\left(
\frac{11}{6}j_yt_j^3
+\frac{5}{2}j_yt_at_j^2
+\frac{1}{2}j_yt_a^2t_j
\right)
$$

$$
l_6=
\sigma
\left(
\frac{11}{6}j_yt_j^3
+3j_yt_at_j^2
+j_yt_a^2t_j
\right)
$$

$$
l_7=
\sigma
\left(
2j_yt_j^3
+3j_yt_at_j^2
+j_yt_a^2t_j
\right)
=
\sigma L_y
$$

纵向关键点由车辆纵向运动积分得到。若当前纵向速度为 $v_0$，纵向加速度为 $a_x$，则第一段结束弧长为：

$$
s_1=v_0t_j+\frac{1}{2}a_xt_j^2
$$

第二段结束：

$$
s_2=s_1+v_1t_a+\frac{1}{2}a_xt_a^2
$$

其中：

$$
v_1=v_0+a_xt_j
$$

第三段结束：

$$
s_3=s_2+v_2t_j+\frac{1}{2}a_xt_j^2
$$

其中：

$$
v_2=v_1+a_xt_a
$$

后半段同理得到 $s_5,s_6,s_7$。这些点形成：

$$
\{(s_0,l_0),(s_1,l_1),(s_2,l_2),(s_3,l_3),(s_5,l_5),(s_6,l_6),(s_7,l_7)\}
$$

再用样条插值得到连续横向偏移函数 $\ell(s)$。

这就是 Shift Pull Out 平滑性的核心：不是直接做线性偏移，而是通过 jerk 和横向加速度约束构造一条接近 clothoid 体验的横移曲线。

## 15. 把横向偏移应用到道路中心线

有了 $\ell(s)$ 后，路径点可以由参考中心线平移得到：

$$
P(s)=C(s)+\ell(s)n(s)
$$

若中心线点姿态为：

$$
C_i=(x_i,y_i,\theta_i)
$$

则横向偏移 $\ell_i$ 后：

$$
x_i'=x_i-\sin\theta_i\ell_i
$$

$$
y_i'=y_i+\cos\theta_i\ell_i
$$

这正是“沿车辆左法向平移”的几何形式。

路径平移后，点与点之间可能出现重复点、方向不一致或局部姿态不准。因此需要重新计算路径点航向。对相邻点：

$$
\theta_i'=
\operatorname{atan2}
\left(
y_{i+1}'-y_i',
x_{i+1}'-x_i'
\right)
$$

对最后一个点，可以沿用前一个方向或通过邻域估计。重新计算航向后，车辆控制器才能正确理解这条路径的行驶方向。

## 16. 为什么还要迭代修正起点

理论上，若起点横向偏移为 $d_0$，把中心线平移 $d_0$ 就应该穿过车辆起点。但在实际曲线路径中，中心线重采样、曲率、局部法向和样条插值都会带来误差。生成后的 Shift 路径可能没有精确经过车辆起点。

因此可以把起点修正看成一个固定点迭代问题。第 $r$ 次生成路径为：

$$
\mathcal{P}^{(r)}
$$

计算车辆起点相对这条路径的横向误差：

$$
e^{(r)}
=
\operatorname{LatOffset}
\left(
\mathcal{P}^{(r)},
P_{\text{start}}
\right)
$$

下一次用这个误差重新生成横移：

$$
d^{(r+1)}=e^{(r)}
$$

直到：

$$
|d^{(r+1)}-d^{(r)}|<\epsilon
$$

或者迭代次数达到上限。这样能让最终 Shift 路径的起点与车辆当前位置更好对齐，避免控制器刚开始跟踪时出现明显横向误差。

## 17. 速度规划：横移段不能比终端速度更快

Shift Pull Out 不只生成几何路径，也要给路径点赋速度。前面已经得到横移结束时的终端速度：

$$
v_{\text{term}}=a_xT_{\text{shift}}
$$

对横移完成前的路径点，速度不应超过这个终端速度：

$$
v_i
\leftarrow
\min
\left(
v_i,
v_{\text{term}}
\right),
\qquad
s_i<s_{\text{end}}
$$

这样可以避免车辆在横移还没完成时速度过快。若参考路径自身速度更低，例如接近目标点或道路限速较低，则保留参考路径速度。

如果参考路径终点就是路线目标点，则最终速度应为：

$$
v_{\text{last}}=0
$$

这表示车辆在目标点停车，而不是继续沿路行驶。

Shift Pull Out 的速度规划相对简单，但它和动态安全预测关系很大。后续预测自车未来轨迹时，会使用：

$$
(v_{\text{term}},a_x)
$$

作为自车预测路径的速度和加速度特征。也就是说，横移路径的速度设计会直接影响 RSS 动态安全判断。

## 18. 静态安全检查第一层：不能离开车道

Shift 路径生成后，首先要检查车辆足迹是否离开可行驶区域。车辆不能被看成一个点，而应该使用车身足迹多边形。设车辆局部足迹为：

$$
F_{\text{local}}=\{f_1,f_2,\dots,f_m\}
$$

路径点位姿为：

$$
P_i=(x_i,y_i,\psi_i)
$$

对应变换矩阵为：

$$
T_i=
\begin{bmatrix}
\cos\psi_i & -\sin\psi_i & x_i\\
\sin\psi_i & \cos\psi_i & y_i\\
0&0&1
\end{bmatrix}
$$

全局足迹为：

$$
F_i=
\{T_i\tilde f_j\mid f_j\in F_{\text{local}}\}
$$

可行驶车道多边形集合为：

$$
\mathcal{P}_{\text{lane}}
=
\bigcup_{\ell\in\mathcal{L}_{\text{drivable}}}
\mathcal{P}_{\ell}
$$

车道内约束为：

$$
F_i\subseteq\mathcal{P}_{\text{lane}}
$$

若考虑膨胀余量 $m_{\text{lane}}$，可以把车辆足迹放大：

$$
\operatorname{buffer}(F_i,m_{\text{lane}})
\subseteq
\mathcal{P}_{\text{lane}}
$$

Shift Pull Out 通常只对从横移起点到横移终点的关键区间做严格车道偏离检查：

$$
s\in[s_{\text{shift,start}},s_{\text{shift,end}}]
$$

如果车辆一开始就不完全在车道内，有时可以允许关闭这类检查的覆盖逻辑。背后的含义是：车辆初始状态已经处于路边或边界附近，如果硬性要求每个初始足迹都完全在车道内，反而会让合法的起步场景无法规划。但这类放宽必须谨慎，因为它会削弱边界安全约束。

## 19. 静态安全检查第二层：裁剪与连续性

从路边起步时，参考路径可能包含一部分车辆后方或车道外的点。为了给下游模块一个干净的可行驶路径，需要裁掉起点附近位于车道外的点。

裁剪后路径记为：

$$
\mathcal{P}_{\text{crop}}
$$

如果裁剪后为空：

$$
|\mathcal{P}_{\text{crop}}|=0
$$

则路径不可用。

即使不为空，也要避免裁剪过度导致起点附近路径点跳变。设车辆起点投影到裁剪后路径的最近段为 $i$，起点到最近段和下一段的纵向偏差分别为：

$$
\Delta s_i
$$

$$
\Delta s_{i+1}
$$

若：

$$
|\Delta s_i-\Delta s_{i+1}|
\ge
D_{\text{long,max}}
$$

说明裁剪后路径开头几何不稳定，可能导致控制器跟踪困难，应拒绝该候选。

这一步看起来不像“路径规划”，但对低速起步很关键。路边起步的初始点经常处在狭窄区域，路径开头如果被裁得太激进，车辆会收到一条从身后或侧后方突然开始的路径。

## 20. 静态安全检查第三层：与停车障碍物保持余量

Shift 路径必须避开静止障碍物。先筛选出低速或静止目标：

$$
\mathcal{O}_{\text{stop}}
=
\{o_i\mid \|v_i\|<v_{\text{moving}}\}
$$

再保留位于起步相关车道内、且类别需要检查的物体：

$$
\mathcal{O}_{\text{check}}
\subseteq
\mathcal{O}_{\text{stop}}
$$

对 Shift 路径的碰撞检查区间：

$$
\mathcal{P}_{\text{check}}
=
\{P_i\mid s_i\in[s_{\text{shift,start}},s_{\text{shift,end}}+\Delta s_{\text{check}}]\}
$$

其中 $\Delta s_{\text{check}}$ 可以为负，表示检查到横移终点前一点；也可以为零或正值，表示检查到终点或更远处。

对每个路径点和每个静止障碍物，要求：

$$
\operatorname{dist}
\left(
F(P_i),
\mathcal{P}_{o_j}
\right)
\ge
m_{\text{collision}}
$$

其中 $m_{\text{collision}}$ 是当前搜索使用的静态碰撞余量。

如果存在：

$$
\operatorname{dist}
\left(
F(P_i),
\mathcal{P}_{o_j}
\right)
<
m_{\text{collision}}
$$

则该 Shift 候选路径被拒绝。

注意，静态碰撞余量不是固定一个值，而是从大到小搜索。较大的余量更安全但更容易失败，较小的余量更容易找到路径但更贴近障碍物。搜索顺序通常是：

$$
2.0\text{ m}
\rightarrow
1.0\text{ m}
\rightarrow
0.5\text{ m}
\rightarrow
0.1\text{ m}
$$

这代表一种“先保守，必要时再放宽”的策略。

## 21. Shift 候选为什么可能失败

现在可以总结 Shift Pull Out 候选失败的主要原因。

如果目标道路车道为空，无法构造参考中心线，失败。

如果横移距离太短，或者横移起终点在离散路径上几乎相邻，生成稳定样条失败。此时可能退化为非横移路径。

如果横移段车辆足迹离开车道：

$$
F_i\not\subseteq\mathcal{P}_{\text{lane}}
$$

失败。

如果路径裁剪后为空：

$$
|\mathcal{P}_{\text{crop}}|=0
$$

失败。

如果裁剪造成起点附近纵向跳变过大：

$$
|\Delta s_i-\Delta s_{i+1}|
\ge
D_{\text{long,max}}
$$

失败。

如果与静止障碍物距离不足：

$$
\operatorname{dist}(F_i,\mathcal{P}_o)
<
m_{\text{collision}}
$$

失败。

只有同时满足几何可生成、车道内、裁剪稳定、静态无碰撞的候选，才会成为可执行 Shift Pull Out 路径。

## 22. 与动态安全检查的关系

Shift Pull Out 的静态检查通过后，还不能立刻起步。路边起步最危险的动态对象通常是后方来车。车辆从路边横移进入车道时，后方车辆可能在几秒后接近并与自车路径冲突。

动态安全检查会生成自车预测路径。设当前速度为 $v_0$，Shift 路径的预测加速度为 $a_x$，最大预测速度为 $v_{\text{term}}$。沿路径的预测弧长为：

$$
s(t)=v_0t+\frac{1}{2}a_xt^2
$$

速度为：

$$
v(t)=\min(v_0+a_xt,\ v_{\text{term}})
$$

按时间分辨率 $\Delta t$ 采样：

$$
t_i=i\Delta t
$$

得到自车未来位姿：

$$
E_i=P_{\text{shift}}(s(t_i))
$$

对每个目标物体，也使用其预测路径得到：

$$
O_i=O(t_i)
$$

如果某时刻自车足迹与目标物体多边形直接相交：

$$
F(E_i)\cap\mathcal{P}(O_i)\neq\varnothing
$$

则不安全。

若没有直接相交，还要计算 RSS 纵向安全距离。设前车速度为 $v_f$，后车速度为 $v_r$，后车反应时间与安全裕度之和为：

$$
\rho=t_{\text{reaction}}+t_{\text{margin}}
$$

前车和后车制动减速度分别为 $a_f<0$、$a_r<0$。RSS 距离为：

$$
D_{\text{RSS}}
=
v_r\rho
+\frac{v_r^2}{2|a_r|}
-\frac{v_f^2}{2|a_f|}
$$

还会引入低速下的最小纵向距离：

$$
D_{\min}
=
\Delta t_v\max(|v_f|,|v_r|)
D_{\text{base}}
$$

最终纵向扩展为：

$$
D_{\text{lon}}
=
\max(D_{\text{RSS}},D_{\min})h
$$

横向扩展为：

$$
D_{\text{lat}}
=
D_{\text{lat,max}}h
$$

其中 $h$ 是滞回系数，用于避免安全状态频繁跳变。

若扩展后的后车安全多边形与前车多边形相交：

$$
\mathcal{P}_{\text{rear}}^{\text{extended}}
\cap
\mathcal{P}_{\text{front}}
\neq
\varnothing
$$

则 Shift 路径在当前动态环境下不安全。车辆会继续等待批准或输出停车路径，而不是贸然横移进入车道。

## 23. 后车被挡住时，为什么不能无限等待

路边起步有一个特殊问题：如果自车已经占据了车道边缘，后方车辆可能没有足够空间从自车和车道边界之间通过。此时一直等待后车“安全通过”可能导致死锁，因为后车根本过不去。

因此，启动规划会判断自车是否阻碍后车通行。设自车靠近某一侧车道边界的最小间隙为：

$$
g
$$

后方最近目标车辆宽度为：

$$
w_o
$$

再加额外安全宽度：

$$
m_w
$$

若：

$$
w_o+m_w>g
$$

说明后车无法从自车旁边通过。此时继续等待动态安全完全放行可能没有意义，算法可以减少对后车通过的等待依赖，让自车尽快完成 Shift 合流，释放阻塞区域。

这个判断还会考虑“如果现在制动，车辆会停在哪里”。设最小可行停车距离为：

$$
D_{\text{stop}}
$$

它由当前速度、当前加速度、最大减速度和最大 jerk 约束决定：

$$
\dot a(t)=j(t)
$$

$$
a(t)\ge-a_{\max}
$$

$$
|j(t)|\le j_{\max}
$$

并满足：

$$
v(t_{\text{stop}})=0
$$

停车位姿也要进行同样的后车通行间隙判断。这样可以避免车辆在执行 Shift 过程中突然停在一个更堵的位置。

## 24. 审批、停车和执行

Shift Pull Out 找到静态安全路径后，通常先进入等待状态。等待时输出的路径几何可以是候选 Shift 路径，但速度会被置为：

$$
v_i=0
$$

这表示“路径已准备好，但车辆暂不执行”。

当外部批准满足、周围移动物体检查通过、RSS 动态安全也通过后，车辆开始执行 Shift 路径。

如果车辆尚未真正移动，而动态检查发现风险，继续等待即可。如果车辆已经开始执行 Shift，之后动态风险突然变为不安全，则尝试在当前路径上插入停止点。设当前速度为 $v_0$，目标速度为 $0$，最小停车距离为：

$$
D_{\text{stop}}
$$

在路径上找到距离当前位姿 $D_{\text{stop}}$ 的点：

$$
P_{\text{stop}}
=
P(s_{\text{ego}}+D_{\text{stop}})
$$

将该点速度设为：

$$
v(P_{\text{stop}})=0
$$

如果停车点无法在路径上可行插入，则继续使用原路径并保持安全监督。若风险消失且车辆已经停稳，则可以删除临时停止点，恢复原来的 Shift 路径。

## 25. 转向灯和 Shift 起终点

Shift Pull Out 是横向移动行为，必须给出清晰的转向灯意图。转向灯方向由横向偏移变化决定。

设起点横向偏移为：

$$
d_{\text{start}}=d_0
$$

终点横向偏移为：

$$
d_{\text{end}}=0
$$

横向变化为：

$$
\Delta d=d_{\text{end}}-d_{\text{start}}
=
-d_0
$$

如果 $\Delta d>0$，车辆向左合流；如果 $\Delta d<0$，车辆向右合流。用符号表示：

$$
\operatorname{signal}
=
\begin{cases}
\text{left}, & \Delta d>0\\
\text{right}, & \Delta d<0\\
\text{none}, & |\Delta d|\approx0
\end{cases}
$$

转向灯通常从 Shift 起点前开始，至少覆盖到 Shift 终点附近。若设置了起步准备时间，车辆进入自动驾驶并准备向前行驶后，还会等待：

$$
T_{\text{prepare}}
$$

再真正出发：

$$
t_{\text{now}}-t_{\text{engage}}
\ge
T_{\text{prepare}}
$$

这能让周围交通参与者在车辆横移前看到稳定的意图信号。

## 26. 可行驶区域：Shift 路径之外还要给边界

下游轨迹优化和控制不仅需要中心路径，也需要知道车辆允许在哪个区域内行驶。Shift Pull Out 的可行驶区域通常由道路车道和路肩车道共同构成。

设路径经过的道路车道集合为：

$$
\mathcal{L}_{\text{road-on-path}}
$$

路肩车道集合为：

$$
\mathcal{L}_{\text{shoulder}}
$$

可行驶车道集合为：

$$
\mathcal{L}_{\text{drivable}}
=
\mathcal{L}_{\text{road-on-path}}
\cup
\mathcal{L}_{\text{shoulder}}
$$

对应可行驶区域为：

$$
\mathcal{A}_{\text{drivable}}
=
\bigcup_{\ell\in\mathcal{L}_{\text{drivable}}}
\mathcal{P}_{\ell}
$$

若需要左右扩展边界，设左扩展为 $m_L$，右扩展为 $m_R$：

$$
\mathcal{A}_{\text{expanded}}
=
\operatorname{expand}
\left(
\mathcal{A}_{\text{drivable}},
m_L,
m_R
\right)
$$

当车辆已经向前执行 Shift 时，这个区域还可以与上游模块给出的可行驶区域合并，保证行为模块之间边界连续。

## 27. Shift Pull Out 什么时候结束

Shift Pull Out 的结束不是看车辆是否“接近某个点”，而是看车辆是否已经沿车道方向通过横移终点。设车辆当前弧长为：

$$
s_{\text{ego}}
$$

横移终点弧长为：

$$
s_{\text{shift,end}}
$$

若：

$$
s_{\text{ego}}-s_{\text{shift,end}}
>
\epsilon_s
$$

则认为车辆已经完成 Shift Pull Out。这里 $\epsilon_s$ 是一个小正数，避免刚到终点附近就提前结束。

完成后，车辆已经基本位于道路中心线附近，后续普通车道跟随、避障或速度规划模块可以接管。

## 28. 用一条算法路线串起来

现在把 Shift Pull Out 的完整学习路线串成一个连续过程。

首先，把道路表示为中心线弧长坐标，计算车辆候选起点的横向偏移 $d_0$。如果 $|d_0|$ 很小，就使用非横移路径；否则进入 Shift 路径生成。

然后，根据当前或倒车后的候选起点，构造目标道路中心线参考路径。参考路径从起点后方一定距离开始，向前延伸到固定长度或路线目标点，并按固定间隔重采样。

接着，使用横向 jerk $j_y$ 和横向加速度采样值 $a_y$ 计算横移时间：

$$
T_{\text{shift}}=4t_j+2t_a
$$

并由道路期望速度得到纵向加速度和终端速度：

$$
a_x=
\operatorname{clamp}
\left(
\frac{v_{\text{road}}}{T_{\text{shift}}},
0,
a_{x,\max}
\right)
$$

$$
v_{\text{term}}=a_xT_{\text{shift}}
$$

再计算至少需要的纵向距离：

$$
D_{\text{pullout}}
=
\max
\left(
\frac{1}{2}a_xT_{\text{shift}}^2,
\sqrt{
\max
\left(
\frac{4|d_0|}{\kappa_{\max}}-d_0^2,
0
\right)
},
D_{\min}
\right)
$$

在弯道上，把实际横移路径长度换算成中心线弧长，并继续向前寻找曲率足够小的横移终点：

$$
|\kappa(s_{\text{end}})|<\kappa_{\text{end}}
$$

随后构造横向偏移函数 $\ell(s)$，让它从 $d_0$ 平滑过渡到 $0$：

$$
P(s)=C(s)+\ell(s)n(s)
$$

生成路径后，迭代修正起点误差，使路径真正贴合车辆起步位姿：

$$
|d^{(r+1)}-d^{(r)}|<\epsilon
$$

然后设置横移段速度：

$$
v_i\le v_{\text{term}}
$$

如果终点是路线目标，则路径末端速度为：

$$
v_{\text{last}}=0
$$

最后进行三类静态验证：车身足迹不离开车道、裁剪后路径连续、与静止障碍物保持余量：

$$
F_i\subseteq\mathcal{P}_{\text{lane}}
$$

$$
|\Delta s_i-\Delta s_{i+1}|<D_{\text{long,max}}
$$

$$
\operatorname{dist}(F_i,\mathcal{P}_o)\ge m_{\text{collision}}
$$

通过静态检查后，Shift 路径进入等待审批和动态安全监督。动态安全通过后执行；执行中如果出现风险，则尝试插入停止点；驶过横移终点后，Shift Pull Out 完成。

## 29. 参数如何影响行为

横向 jerk $j_y$ 越大，车辆横移越快，但横向加速度变化也越激烈，乘坐舒适性下降。$j_y$ 越小，路径更温和，但需要更长纵向距离。

横向加速度范围 $[a_{y,\min},a_{y,\max}]$ 决定候选曲线集合。较小的 $a_y$ 生成更长、更平顺的路径；较大的 $a_y$ 生成更短、更积极的路径。若搜索从小到大进行，通常会优先尝试更自然的路径。

最大曲率 $\kappa_{\max}$ 越小，路径越不允许急弯，所需纵向距离越长。它直接影响：

$$
D_{\kappa}
=
\sqrt{
\frac{4L_y}{\kappa_{\max}}-L_y^2
}
$$

横移终点曲率阈值 $\kappa_{\text{end}}$ 越小，算法越倾向于在直一点的道路上完成横移，路径可能更长，但末端更平顺。

最小驶出距离 $D_{\min}$ 用于防止横移距离在低速或小偏移场景下过短。它是工程安全底线。

碰撞余量 $m_{\text{collision}}$ 越大，路径离静止障碍物越远，但更容易找不到候选。多级余量搜索可以在安全和可行之间折中。

倒车最大距离 $D_{\text{back}}$ 和搜索分辨率 $\Delta b$ 决定 Shift 能否通过“先倒一点”获得更好的起步空间。$D_{\text{back}}$ 太小，可能找不到可行起点；太大，则可能生成不自然的倒车动作。$\Delta b$ 太大，可能跳过刚好可行的位置；太小，则计算量增加。

## 30. 最后记住这条主线

Shift Pull Out 的本质不是“画一条 S 形曲线”，而是把路边起步拆成一组可验证的约束。

它先用车道坐标找出车辆偏离中心线的距离 $d_0$，再用 jerk 和横向加速度推导横移时间，用纵向加速度和曲率约束推导所需距离，在道路中心线上寻找合适的横移终点，然后用平滑横向偏移函数生成路径。路径生成后，还要经过车道边界、路径连续性、静态障碍物、动态 RSS、审批和停车策略的层层筛选。

如果只用一个公式总结，可以写成：

$$
\text{Shift Pull Out}
=
\text{Centerline}
+\text{Smooth Lateral Offset}
+\text{Safety Validation}
+\text{Execution Supervision}
$$

更具体一点：

$$
P(s)
=
C(s)+\ell(s)n(s),
\qquad
\ell(s_{\text{start}})=d_0,
\qquad
\ell(s_{\text{end}})=0
$$

并且：

$$
\ell(s)
\text{ obeys jerk and acceleration limits}
$$

$$
F(P_i)
\text{ stays inside drivable lanes and away from obstacles}
$$

$$
\text{dynamic objects satisfy RSS safety before and during departure}
$$

这就是 Shift Pull Out 的完整思想：用一条可控、平滑、车道语义明确的横移路径，把车辆从路边安全带回正常行驶车道。
