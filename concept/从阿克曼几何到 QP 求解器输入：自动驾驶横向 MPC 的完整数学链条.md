参考：chatGPT

---

# 目录

1. 为什么要学这套链条
2. 理想阿克曼转向模型：所有后续公式的几何起点
3. 车辆运动学模型：从阿克曼几何到自行车模型
4. 为什么不用全局位置直接控，而要引入 Frenet 误差
5. Frenet 坐标系：每个量到底是什么意思
6. Frenet 误差模型的完整推导
7. 小误差近似：工程模型为什么会简化成那样
8. 转向执行器动态：$\tau$ 从哪里来，为什么公式要这样写
9. 非线性误差模型如何线性化成 $\dot x = A x + B u + w$
10. 连续时间模型如何离散化成 $x_{k+1}=A_d x_k+B_d u_k+W_d$
11. 单步模型为什么还不够：MPC 为什么必须做未来时域展开
12. 从单步离散模型到批量预测方程
13. 代价函数是怎么设计出来的，为什么是二次型
14. 把预测方程代入代价函数：如何变成只关于输入的优化问题
15. 输入约束和输入变化率约束如何写成 QP 形式
16. QP 求解
17. 整套链条的统一大图景
18. 常见误区与统一澄清

---

# 1. 为什么要学这套链条

自动驾驶横向控制的目标，说得最朴素一点，就是：

> 让车沿着一条给定的参考轨迹平稳地走。

如果只从这个目标出发，你很容易以为问题只是：

* 当前车的位置是多少
* 当前车头朝向是多少
* 方向盘该打多少

但真正做控制时，会发现单独看世界坐标下的位置和姿态是不够的。控制器真正关心的是：

* 车偏离参考轨迹多少
* 偏离的方向是什么
* 车头和参考轨迹方向差了多少
* 如果我现在和未来几步分别打多少方向，未来误差会怎样演化

于是就形成了一整条经典建模链：

```text
车辆几何
→ 车辆运动学
→ 相对轨迹误差建模
→ 执行器建模
→ 线性化
→ 离散化
→ 未来时域展开
→ 代价函数设计
→ QP 求解
```

这篇文章就是把这条链从头到尾讲清楚。

---

# 2. 理想阿克曼转向模型：所有后续公式的几何起点

---

## 2.1 阿克曼转向到底要解决什么

先想一个非常基础的问题：四轮车转弯时，为什么左右前轮角度不能一样？

如果假设车辆低速转弯、轮胎纯滚动且无侧滑，那么每个车轮都必须满足：

* 轮心速度方向与车轮平面方向一致
* 车轮不能横着滑

而刚体在平面中的瞬时运动，总可以看成绕某个点做旋转。这个点叫做：

* 瞬时旋转中心
* Instantaneous Center of Rotation, ICR

要让所有车轮都不侧滑，那么每个车轮都必须“服从同一个 ICR”。这就意味着：

> 每个车轮所在平面的法线，必须都通过同一个瞬时旋转中心。

如果左右前轮打同样的角度，这个条件一般无法同时满足，于是轮胎会产生侧滑。阿克曼转向几何的作用就是让左右前轮角不同，以实现共同 ICR。

---

## 2.2 最基础的单轨几何关系

在控制和建模中，我们通常把真实四轮车等效成一个“单轨模型”或“自行车模型”：

* 左右前轮合并成一个等效前轮
* 左右后轮合并成一个等效后轮

设：

* 轴距：$L$
* 车辆等效前轮转角：$\delta$
* 后轴中心到 ICR 的距离：$R$

那么几何上会形成一个直角三角形，最终得到：

$$
\tan\delta = \frac{L}{R}
$$

这是整套模型最核心的几何起点之一。

---

## 2.3 为什么是 $\tan\delta = \frac{L}{R}$，而不是 $\frac{R}{L}$


关键在于：$\delta$ 不是随便定义的三角形角，而是**车轮方向相对于车身纵向方向**的夹角。

在单轨模型里：

* 后轴中心到前轴中心的方向是车身纵向
* ICR 位于后轴延长线上
* 前轮方向必须与其所走圆轨迹的切线一致

由这个几何关系建立三角形时，最终得到的正切关系恰好是：

$$
\tan\delta = \frac{L}{R}
$$

可以用直觉检验：

* $R$ 越大，弯越缓，$\delta$ 应该越小
* $L$ 越大，想走同样半径的弯，需要更大的 $\delta$

这都与 $\tan\delta = \frac{L}{R}$ 完全一致。

---

## 2.4 曲率的引入

定义曲率：

$$
\kappa = \frac{1}{R}
$$

于是：

$$
\tan\delta = L\kappa
$$

从而：

$$
\delta = \arctan(L\kappa)
$$

这条公式在后面会反复出现，因为：

* 路径规划器通常更自然给出曲率 $\kappa$
* 车辆执行器需要的是转向角 $\delta$

所以这条公式本质上是在做“路径几何量 $\to$ 控制输入量”的映射。

---

## 2.5 更完整的左右前轮阿克曼关系

如果不做单轨近似，而是看真实左右前轮：

设：

* 轴距：$L$
* 轮距：$W$
* 后轴中心到 ICR 的距离：$R$
* 内侧前轮转角：$\delta_l$
* 外侧前轮转角：$\delta_r$

则有：

$$
\tan\delta_l = \frac{L}{R-\frac{W}{2}}
$$

$$
\tan\delta_r = \frac{L}{R+\frac{W}{2}}
$$

从这两式可以进一步推出经典阿克曼约束：

$$
\cot\delta_r - \cot\delta_l = \frac{W}{L}
$$

这意味着：

* 左右前轮转角不是独立量
* 内侧轮角更大，外侧轮角更小
* 这正是为了让四轮对应同一个瞬时旋转中心

---

## 2.6 那个“三角形”的三个点到底是谁


### 对单轨模型的三角形

点分别是：

1. 瞬时旋转中心 $C$
2. 后轴中心 $O_r$
3. 前轴中心 $O_f$

这时：

* $CO_r = R$
* $O_rO_f = L$

从而得到：

$$
\tan\delta = \frac{L}{R}
$$

### 对左前轮的三角形

点分别是：

1. 瞬时旋转中心 $C$
2. 左后轮轮心 $R_l$
3. 左前轮轮心 $F_l$

于是：

* $R_lF_l = L$
* $CR_l = R-\frac{W}{2}$

得到：

$$
\tan\delta_l = \frac{L}{R-\frac{W}{2}}
$$

### 对右前轮的三角形

点分别是：

1. 瞬时旋转中心 $C$
2. 右后轮轮心 $R_r$
3. 右前轮轮心 $F_r$

于是：

$$
\tan\delta_r = \frac{L}{R+\frac{W}{2}}
$$

---

# 3. 车辆运动学模型：从阿克曼几何到自行车模型

---

## 3.1 世界坐标中的状态定义

设车辆后轴中心在世界坐标下的位置为：$(x,y)$，车辆航向角为：$\psi$，车辆速度为：$v$，等效前轮转角为：$\delta$

---

## 3.2 世界坐标下的理想运动学模型

理想自行车模型写成：

$$
\dot x = v\cos\psi
$$

$$
\dot y = v\sin\psi
$$

$$
\dot\psi = \frac{v}{L}\tan\delta
$$

这三式分别表示：

* （1）（2）车在世界坐标中沿当前航向前进
* （3）航向变化率由速度、轴距、转角决定

---

## 3.3 航向变化率为什么是 $\frac{v}{L}\tan\delta$

由圆周运动关系：

$$
\omega = \frac{v}{R}
$$

这里 $\omega$ 就是航向变化率 $\dot\psi$。

而阿克曼几何给出：

$$
R = \frac{L}{\tan\delta}
$$

代入：

$$
\dot\psi = \frac{v}{R} = \frac{v}{L/\tan\delta} = \frac{v}{L}\tan\delta
$$

---

## 3.4 曲率写法

由于：

$$
\kappa = \frac{1}{R} = \frac{\tan\delta}{L}
$$

所以：

$$
\dot\psi = v\kappa
$$

这说明：

* 速度控制“前进快慢”
* 曲率控制“路径弯曲程度”

---

# 4. 为什么不用全局位置直接控，而要引入 Frenet 误差

如果你只在世界坐标下写：

* 我现在在 $(x,y)$
* 轨迹上的点在 $(x_r,y_r)$

看起来好像就能直接做控制，但这在轨迹跟踪里其实并不自然。

原因是：

* 轨迹本身可能是弯的
* 只比较欧氏位置差，无法直接体现“我相对轨迹偏到哪边”
* 车头方向和轨迹切线方向的关系也非常关键

所以我们希望定义一种“沿轨迹”和“垂直轨迹”的局部坐标描述。

这就是 Frenet 坐标系。

---

# 5. Frenet 坐标系：每个量到底是什么意思

---

## 5.1 参考轨迹的弧长参数化

用弧长 $s$ 参数化参考轨迹：

$$
\mathbf r(s)=
\begin{bmatrix}
x_r(s)\\
y_r(s)
\end{bmatrix}
$$

这里的意思是：

> 沿参考轨迹走了 $s$ 米时，对应点的世界坐标是多少。

用弧长参数化有一个重要性质：

$$
\left|\frac{d\mathbf r}{ds}\right| = 1
$$

也就是说，轨迹对弧长求导，天然就是单位切向量（回忆高数对弧长的求导）。

---

## 5.2 切向量

参考轨迹在 $s$ 处的切向方向角记为 $\psi_r(s)$，则切向单位向量为：

$$
\mathbf t(s)=
\begin{bmatrix}
\cos\psi_r\\
\sin\psi_r
\end{bmatrix}
$$

---

## 5.3 法向量

左法向单位向量为：

$$
\mathbf n(s)=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

你可以验证：

$$
\mathbf t^T\mathbf n = 0
$$

即两者正交。

---

## 5.4 横向误差 $e$ 的定义

设车辆当前位置为：

$$
\mathbf p=
\begin{bmatrix}
x\\
y
\end{bmatrix}
$$

定义 $e$ 为：从参考轨迹点 $\mathbf r(s)$ 出发，沿法向 $\mathbf n(s)$ 走 $e$ 的距离，刚好到达车辆位置 $\mathbf p$。

于是：

$$
\mathbf p = \mathbf r(s) + e \, \mathbf n(s)
$$

这不是某种“推导出来的复杂结论”，它本身就是横向误差的定义。

展开后：

$$
x = x_r - e\sin\psi_r
$$

$$
y = y_r + e\cos\psi_r
$$

---

## 5.5 航向误差 $\theta_e$

定义：

$$
\theta_e = \psi - \psi_r
$$

表示车辆自身朝向和参考轨迹切线方向的差。

因此：

$$
\psi = \psi_r + \theta_e
$$

---

## 5.6 参考曲率 $\kappa_r$

曲率定义为：

$$
\kappa_r = \frac{d\psi_r}{ds}
$$

表示“沿轨迹走单位弧长，轨迹方向变化了多少”。

对于圆：

$$
\kappa = \frac{1}{R}
$$

这与常见几何定义完全一致。

---

# 6. Frenet 误差模型的完整推导

这是整套横向控制建模中最关键的一步。

---

## 6.1 从位置定义开始

已经有：

$$
\mathbf p = \mathbf r(s) + e \,\mathbf n(s)
$$

由于：

* $s=s(t)$
* $e=e(t)$
* $\mathbf r,\mathbf n$ 都是 $s$ 的函数


---

## 6.2 对时间求导

$$
\dot{\mathbf p}=
\frac{d}{dt}\mathbf r(s)+
\frac{d}{dt}\big(e,\mathbf n(s)\big)
$$

第一项用链式法则：

$$
\frac{d}{dt}\mathbf r(s)=
\frac{d\mathbf r}{ds}\dot s
$$

第二项用乘法法则：

$$
\frac{d}{dt}(e,\mathbf n)=
\dot e,\mathbf n + e,\frac{d\mathbf n}{dt}
$$

而：

$$
\frac{d\mathbf n}{dt}=
\frac{d\mathbf n}{ds}\dot s
$$

所以：

$$
\dot{\mathbf p}=
\frac{d\mathbf r}{ds}\dot s+
\dot e \, \mathbf n+
e \, \frac{d\mathbf n}{ds}\dot s
$$

---

## 6.3 Frenet 公式

由于弧长参数化：

$$
\frac{d\mathbf r}{ds} = \mathbf t
$$

下面求 $\frac{d\mathbf n}{ds}$。

由：

$$
\mathbf n=
\begin{bmatrix}
-\sin\psi_r\\
\cos\psi_r
\end{bmatrix}
$$

对 $s$ 求导：

$$
\frac{d\mathbf n}{ds}=
\begin{bmatrix}
-\cos\psi_r \frac{d\psi_r}{ds}\\
-\sin\psi_r \frac{d\psi_r}{ds}
\end{bmatrix}
$$

而：

$$
\frac{d\psi_r}{ds} = \kappa_r
$$

所以：

$$
\frac{d\mathbf n}{ds}=
-\kappa_r
\begin{bmatrix}
\cos\psi_r\\
\sin\psi_r
\end{bmatrix}=
-\kappa_r \mathbf t
$$

---

## 6.4 代回位置导数

因此：

$$
\dot{\mathbf p}=
\mathbf t \dot s+
\dot e,\mathbf n+
e(-\kappa_r\mathbf t)\dot s
$$

整理：

$$
\dot{\mathbf p}=
(1-e\kappa_r)\dot s,\mathbf t+
\dot e,\mathbf n
$$

这就是 Frenet 基底下的速度表达。

---

## 6.5 车辆真实速度表达

另一方面，车辆真实速度沿自身航向方向：

$$
\dot{\mathbf p}=v
\begin{bmatrix}
\cos\psi\\
\sin\psi
\end{bmatrix}
$$

由于 $\theta_e = \psi - \psi_r$，把该向量在 Frenet 基底 $(\mathbf t,\mathbf n)$ 下分解得：

$$
\dot{\mathbf p}=
v\cos\theta_e,\mathbf t+
v\sin\theta_e,\mathbf n
$$

---

## 6.6 对比两种表达

我们有：

$$
(1-e\kappa_r)\dot s \,\mathbf t + \dot e \, \mathbf n=
v \cos\theta_e \, \mathbf t + v\sin\theta_e \, \mathbf n
$$

由于 $\mathbf t,\mathbf n$ 独立正交，对应系数相等。

### 法向方向：

$$
\dot e = v\sin\theta_e
$$

### 切向方向：

$$
(1-e\kappa_r)\dot s = v\cos\theta_e
$$

所以：

$$
\dot s = \frac{v\cos\theta_e}{1-e\kappa_r}
$$

---

## 6.7 航向误差动力学

由：

$$
\theta_e = \psi - \psi_r
$$

得：

$$
\dot\theta_e = \dot\psi - \dot\psi_r
$$

车辆真实航向变化率：

$$
\dot\psi = \frac{v}{L}\tan\delta
$$

参考轨迹航向变化率：

$$
\dot\psi_r = \frac{d\psi_r}{ds}\dot s = \kappa_r \dot s
$$

所以：

$$
\dot\theta_e=
\frac{v}{L}\tan\delta - \kappa_r\dot s
$$

代入 $\dot s$：

$$
\dot\theta_e=
\frac{v}{L}\tan\delta -
\kappa_r\frac{v\cos\theta_e}{1-e\kappa_r}
$$

---

## 6.8 得到完整非线性 Frenet 误差模型

因此：

$$
\dot e = v\sin\theta_e
$$

$$
\dot s = \frac{v\cos\theta_e}{1-e\kappa_r}
$$

$$
\dot\theta_e =
\frac{v}{L}\tan\delta-
\kappa_r\frac{v\cos\theta_e}{1-e\kappa_r}
$$

这就是严格的 Frenet 误差动力学。

---

# 7. 小误差近似：工程模型

完整非线性模型准确，但直接用于线性 MPC 太复杂，所以工程中通常做小误差近似。

---

## 7.1 假设

假设车辆跟踪还不错：

$$
e \approx 0,\qquad \theta_e \approx 0
$$

于是：

$$
\sin\theta_e \approx \theta_e
$$

$$
\cos\theta_e \approx 1
$$

$$
1-e\kappa_r \approx 1
$$

---

## 7.2 简化 $\dot e$

原式：

$$
\dot e = v\sin\theta_e
$$

变成：

$$
\dot e \approx v\theta_e
$$

---

## 7.3 简化 $\dot s$

$$
\dot s = \frac{v\cos\theta_e}{1-e\kappa_r}
\approx v
$$

---

## 7.4 简化 $\dot\theta_e$

原式：

$$
\dot\theta_e =
\frac{v}{L}\tan\delta - \kappa_r \dot s
$$

近似后：

$$
\dot\theta_e \approx \frac{v}{L}\tan\delta - v\kappa_r
$$

---

## 7.5 工程中常用的简化误差模型

因此得到：

$$
\dot e = v\theta_e
$$

$$
\dot\theta_e = \frac{v}{L}\tan\delta - v\kappa_r
$$

这就是后面线性化前的“简化非线性模型”。

---

# 8. 转向执行器动态：$\tau$

---

## 8.1 为什么不能直接写 $\delta=u$

如果写：

$$
\delta = u
$$

意思就是：

> 控制器命令一发出，轮子瞬间到位。

这对真实车辆几乎从不成立。真实转向机构有：

* 电机或液压动态
* 机械惯性
* 速率限制
* 控制延迟

所以需要一个更真实的执行器动态模型。

---

## 8.2 最简单的一阶跟踪模型

定义：

* $u$：控制器下发的目标转角
* $\delta$：实际轮角

希望实际轮角向目标靠拢。最简单的假设是：

> 实际转角变化速度与误差 $u-\delta$ 成正比。

即：

$$
\dot\delta = k(u-\delta)
$$

令：

$$
k=\frac{1}{\tau}
$$

得到：

$$
\dot\delta = \frac{1}{\tau}(u-\delta)
$$

展开，得到常用的控制公式：

$$
\dot\delta = -\frac{1}{\tau}\delta + \frac{1}{\tau}u
$$

---

## 8.3 $\tau$ 的物理意义

把它写成：

$$
\tau\dot\delta + \delta = u
$$

这是标准一阶系统。若 $u=u_0$ 为常值，则：

$$
\delta(t)=u_0+(\delta(0)-u_0)e^{-t/\tau}
$$

所以：

* $\tau$ 小，响应快
* $\tau$ 大，响应慢

它叫时间常数，不是阿克曼几何推出来的，而是执行器动态参数。

---

# 9. 非线性误差模型如何线性化成 $\dot x = A x + B u + w$

---

## 9.1 定义状态和输入

设：

$$x=
\begin{bmatrix}
e\\
\theta_e\\
\delta
\end{bmatrix},
\qquad u=
\begin{bmatrix} u
\end{bmatrix}
$$

其中：

* $e$：横向误差
* $\theta_e$：航向误差
* $\delta$：实际转角
* $u$：转向命令


那么根据之前的推导，系统为：

$$
\dot e = v\theta_e
$$

$$
\dot\theta_e = \frac{v}{L}\tan\delta - v\kappa_r
$$

$$
\dot\delta = -\frac{1}{\tau}\delta + \frac{1}{\tau}u
$$

这里唯一非线性项是：

$$
\tan\delta
$$

---

## 9.2 线性化点怎么选

对参考曲率 $\kappa_r$，理想前轮角是：

$$
\delta_r = \arctan(L\kappa_r)
$$

所以在这个工作点附近线性化最自然。

---

## 9.3 泰勒展开

设：

$$
f(\delta)=\tan\delta
$$

在 $\delta_r$ 附近一阶展开：

$$
\tan\delta
\approx
\tan\delta_r
+
\frac{d}{d\delta}\tan\delta\Big|_{\delta_r}(\delta-\delta_r)
$$

而：

$$
\frac{d}{d\delta}\tan\delta = \sec^2\delta = \frac{1}{\cos^2\delta}
$$

所以：

$$
\tan\delta
\approx
\tan\delta_r
+
\sec^2\delta_r(\delta-\delta_r)
$$

展开：

$$
\tan\delta
\approx
\tan\delta_r - \delta_r\sec^2\delta_r + \sec^2\delta_r \, \delta
$$

---

## 9.4 代回 $\dot\theta_e$

原式：

$$
\dot\theta_e = \frac{v}{L}\tan\delta - v\kappa_r
$$

代入线性化：

$$
\dot\theta_e=
\frac{v}{L}\sec^2\delta_r,\delta+
\left[
-v\kappa_r + \frac{v}{L}\left(\tan\delta_r-\delta_r\sec^2\delta_r\right)
\right]
$$

---

## 9.5 得到线性仿射系统

于是：

$$
\dot x = A x + B u + w
$$

其中：

$$
A=
\begin{bmatrix}
0 & v & 0\\
0 & 0 & \frac{v}{L}\sec^2\delta_r\\
0 & 0 & -\frac{1}{\tau}
\end{bmatrix}
$$

$$
B=
\begin{bmatrix}
0\\
0\\
\frac{1}{\tau}
\end{bmatrix}
$$

$$
w=
\begin{bmatrix}
0\\
-v\kappa_r+\frac{v}{L}\left(\tan\delta_r-\delta_r\sec^2\delta_r\right)\\
0
\end{bmatrix}
$$

这里的 $w$ 是线性化偏置项，用来表达常数项，不是噪声。

---

# 10. 连续时间模型如何离散化成 $x_{k+1}=A_d x_k+B_d u_k+W_d$

---

## 10.1 连续系统

从：

$$
\dot x = A x + B u + w
$$

出发。

---

## 10.2 为什么要离散化

MPC 是数字控制器，只在离散时刻工作：

$$
t_k = k\,dt
$$

所以需要把连续系统变成一步一步更新的离散系统。

---

## 10.3 最简单欧拉法（先作为参考）

用：

$$
\dot x(t_k)\approx \frac{x_{k+1}-x_k}{dt}
$$

可得：

$$
x_{k+1}\approx (I+dtA)x_k + dtB u_k + dtw
$$

但工程里常用更稳定的 Tustin / 双线性离散化。

---

## 10.4 Tustin 推导

从积分形式：

$$
x_{k+1}-x_k=
\int_{t_k}^{t_{k+1}} (Ax + Bu + w),dt
$$

用梯形法近似：

$$
\int f(t) \, dt \approx \frac{dt}{2}(f_k + f_{k+1})
$$

于是：

$$
x_{k+1}-x_k
\approx
\frac{dt}{2}
\Big[
(Ax_k+Bu_k+w) + (Ax_{k+1}+Bu_k+w)
\Big]
$$

整理：

$$
\left(I-\frac{dt}{2}A\right)x_{k+1}=
\left(I+\frac{dt}{2}A\right)x_k+
dtBu_k+
dtw
$$

左乘逆矩阵：

$$
x_{k+1}=
\left(I-\frac{dt}{2}A\right)^{-1}\left(I+\frac{dt}{2}A\right)x_k+
\left(I-\frac{dt}{2}A\right)^{-1}B \, dt \, u_k+
\left(I-\frac{dt}{2}A\right)^{-1}w \, dt
$$

定义：

$$
A_d=\left(I-\frac{dt}{2}A\right)^{-1}\left(I+\frac{dt}{2}A\right)
$$

$$
B_d=\left(I-\frac{dt}{2}A\right)^{-1}B \, dt
$$

$$
W_d=\left(I-\frac{dt}{2}A\right)^{-1}w \, dt
$$

所以：

$$
x_{k+1}=A_d x_k + B_d u_k + W_d
$$

---

# 11. 单步模型为什么还不够：MPC 为什么必须做未来时域展开

单步模型只告诉你：

> 如果现在给一个 $u_k$，下一步状态会怎样。

但 MPC 要做的是：

> 同时优化未来 $N$ 步的输入，让未来整个误差轨迹最好。

所以必须引入：

$$
U_{ex}=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_{N-1}
\end{bmatrix}
$$

并预测：

$$
X_{ex}=
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_N
\end{bmatrix}
$$

---

# 12. 从单步离散模型到批量预测方程

---

## 12.1 单步模型

对每一步：

$$
x_{k+1}=A_kx_k+B_ku_k+W_k
$$

注意这里是时变系统，所以每一步矩阵可能不同。

---

## 12.2 展开前几步

### 第一步：

$$
x_1 = A_0 x_0 + B_0 u_0 + W_0
$$

### 第二步：

$$
x_2 = A_1x_1 + B_1u_1 + W_1
$$

代入 $x_1$：

$$
x_2=A_1A_0x_0+
A_1B_0u_0+
B_1u_1+A_1W_0+W_1
$$

### 第三步：

$$
x_3 = A_2x_2 + B_2u_2 + W_2
$$

代入 $x_2$：

$$
x_3=A_2A_1A_0x_0+
A_2A_1B_0u_0+
A_2B_1u_1+
B_2u_2+
A_2A_1W_0 + A_2W_1 + W_2
$$

---

## 12.3 堆叠形式

把所有状态堆起来：

$$
X_{ex}=
\begin{bmatrix}
x_1\\
x_2\\
x_3\\
\vdots\\
x_N
\end{bmatrix}
$$

则可以写成：

$$
X_{ex}=A_{ex}x_0+B_{ex}U_{ex}+W_{ex}
$$

---

## 12.4 $A_{ex}$

$$
A_{ex}=
\begin{bmatrix}
A_0\\
A_1A_0\\
A_2A_1A_0\\
\vdots
\end{bmatrix}
$$

---

## 12.5 $B_{ex}$

$$
B_{ex}=
\begin{bmatrix}
B_0 & 0 & 0 & \cdots\\
A_1B_0 & B_1 & 0 & \cdots\\
A_2A_1B_0 & A_2B_1 & B_2 & \cdots\\
\vdots & \vdots & \vdots & \ddots
\end{bmatrix}
$$

这是一个块下三角矩阵。

---

## 12.6 $W_{ex}$

$$
W_{ex}=
\begin{bmatrix}
W_0\\
A_1W_0 + W_1\\
A_2A_1W_0 + A_2W_1 + W_2\\
\vdots
\end{bmatrix}
$$

---

## 12.7 输出展开

定义：

$$
Y_{ex} = C_{ex}X_{ex}
$$

$$
y_k=
\begin{bmatrix}
e_k\\
\theta_{e,k}
\end{bmatrix}
$$
这里的$Y_{ex}$用于后续做约束，求解最小值。

其中：
$$
C_k=
\begin{bmatrix}
1&0&0\\
0&1&0
\end{bmatrix}
$$
$$
C_{ex}=\operatorname{diag}(C_0,C_1,\dots,C_{N-1})
$$

---

# 13. 代价函数是怎么设计出来的，为什么是二次型

代价函数不是“物理定律推导出来的”，而是根据控制目标设计出来的。

---

## 13.1 目标 1：跟踪误差小

希望：

* 横向误差小
* 航向误差小

设每步输出为：

$$
y_k = C_k x_k
$$

单步惩罚：

$$
y_k^T Q_k y_k
$$

堆叠后：

$$
J_{\text{state}} = Y_{ex}^T Q_{ex}Y_{ex}
$$

其中：

$$
Q_{ex}=\operatorname{diag}(Q_0,Q_1,\dots,Q_{N-1})
$$

其中 $Q_k \succeq 0$ 是权重矩阵。

---

## 13.2 目标 2：输入不要过大，尽量接近轨迹角度

不希望控制器猛打方向，即输入不要太大。但更合理的是让输入靠近参考前馈输入：

$$
u_{ref,k}=\arctan(L\kappa_k)
$$

于是：

$$
J_{\text{input}}=
(U_{ex}-U_{ref,ex})^T R_{1ex}(U_{ex}-U_{ref,ex})
$$

---

## 13.3 目标 3：输入变化要平滑

为了避免转向抖动，希望：

$$
u_k-u_{k-1}
$$

尽量小，惩罚：

$$
(u_k-u_{k-1})^2
$$

堆叠后可写成：

$$
J_{\text{smooth}} = U_{ex}^T R_{2ex} U_{ex}
$$

其中 $R_{2ex}$ 编码了输入差分的二次项。

---

## 13.4 最终总代价函数

$$
J=
Y_{ex}^T Q_{ex}Y_{ex}+
(U_{ex}-U_{ref,ex})^T R_{1ex}(U_{ex}-U_{ref,ex})+
U_{ex}^T R_{2ex} U_{ex}
$$

再由于：

$$
Y_{ex}=C_{ex}X_{ex}
$$

也可写成：

$$
J=
X_{ex}^T C_{ex}^T Q_{ex} C_{ex} X_{ex}+
(U_{ex}-U_{ref,ex})^T R_{1ex}(U_{ex}-U_{ref,ex})+
U_{ex}^T R_{2ex} U_{ex}
$$

---

# 14. 把预测方程代入代价函数：如何变成只关于输入的优化问题

现在关键来了。

已经有：

$$
X_{ex}=A_{ex}x_0+B_{ex}U_{ex}+W_{ex}
$$

$$
Y_{ex}=C_{ex}X_{ex}
$$

所以：

$$
Y_{ex}=C_{ex}(A_{ex}x_0+B_{ex}U_{ex}+W_{ex})
$$

定义简写：

$$
A:=A_{ex},\quad
B:=B_{ex},\quad
C:=C_{ex},\quad
Q:=Q_{ex},\quad
R_1:=R_{1ex},\quad
R_2:=R_{2ex},\quad
U:=U_{ex},\quad
U_r:=U_{ref,ex},\quad
W:=W_{ex}
$$

则：

$$
X = Ax_0 + BU + W
$$

$$
Y = C(Ax_0 + BU + W)
$$

设：

$$
d := C(Ax_0 + W)
$$

则：

$$
Y = d + CBU
$$

于是状态项：

$$
J_{\text{state}}=
(d+CBU)^TQ(d+CBU)
$$

展开：

$$
J_{\text{state}}=
U^T B^T C^T Q C B U+
2d^TQCBU+
d^TQd
$$

输入项：

$$
(U-U_r)^TR_1(U-U_r)=
U^TR_1U - 2U^TR_1U_r + U_r^TR_1U_r
$$

平滑项：

$$
U^TR_2U
$$

合并后：

$$
J=
U^T(B^TC^TQCB+R_1+R_2)U+
2(d^TQCB-U_r^TR_1)U+
\text{const}
$$

其中常数项与 $U$ 无关，不影响优化，可以忽略。

---

# 15. 输入约束和输入变化率约束如何写成 QP 形式

前面我们已经把代价函数整理成了只关于未来输入序列 $U$ 的二次型。  
但只有目标函数还不够，因为真实车辆永远存在物理限制：

* 转角不能无限大
* 转角变化不能无限快

因此，MPC 不是一个“只看目标函数”的无约束优化，而是一个**带线性约束的二次优化问题**。  
也正因为如此，它最后自然落到 QP 这个数学形式上。

---

## 15.1 优化变量到底是什么

在这一层里，真正被优化的不是状态 $x$，而是未来一段时间内的控制输入序列。

把未来 $N$ 步输入堆叠起来：

$$
U =
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_{N-1}
\end{bmatrix}
\in \mathbb R^{N_u}
$$

这里：

* 若每一步只有一个转向输入，则 $N_u=N$
* 更一般地，若每一步输入维度为 $\mathrm{DIM}_U$，则 $N_u=N\cdot \mathrm{DIM}_U$

这一步一定要想清楚，因为 QP 求解器根本不直接“优化轨迹”或“优化误差”，它优化的是：

> 未来一串控制量 $U$

而未来状态 $X_{ex}$、未来输出 $Y_{ex}$ 都只是这个输入序列通过预测模型推出来的结果。

---

## 15.2 输入幅值约束为什么是 box 约束

如果转向命令存在物理上限 $\delta_{\max}$，那么每一步都必须满足：

$$
-\delta_{\max} \le u_k \le \delta_{\max}
$$

把全部时刻堆叠起来后，就得到最简单的一类约束：

$$
lb \le U \le ub
$$

其中：

$$
lb = -\delta_{\max}\mathbf 1,\qquad
ub = \delta_{\max}\mathbf 1
$$

这类约束叫 box 约束，本质上就是：

> 每个优化变量单独都有一个上下界

从几何上看，它在输入空间里对应一个超矩形区域。  
QP 求解器只能在这个可行区域内寻找最优 $U$。

---

## 15.3 输入变化率约束从哪里来

真实车辆除了“能打多大方向”之外，还有“能多快打方向”的限制。

若连续时间下存在转角速度上限：

$$
|\dot u(t)| \le \dot\delta_{\max}(t)
$$

那么离散化之后，相邻两步输入之间就不能差太多。  
在第 $k$ 步，可写成：

$$
-\dot\delta_{\max,k}\Delta t
\le
u_k-u_{k-1}
\le
\dot\delta_{\max,k}\Delta t
$$

这条式子的物理意义非常直接：

* 左边限制“不要往负方向变得太快”
* 右边限制“不要往正方向变得太快”

如果 $\dot\delta_{\max,k}$ 随速度、曲率或工况变化，那么它就不再是一个常数，而是一组逐步变化的上限，这也是工程实现里常见的做法。

---

## 15.4 为什么差分约束可以写成矩阵形式

QP 喜欢的约束形式是仿射不等式：

$$
lbA \le A_{\text{ineq}}U \le ubA
$$

所以关键问题变成：

> 如何把“相邻输入之差”写成矩阵乘法

定义差分矩阵：

$$
D =
\begin{bmatrix}
1 & 0 & 0 & \cdots\\
-1 & 1 & 0 & \cdots\\
0 & -1 & 1 & \cdots\\
\vdots & \vdots & \vdots & \ddots
\end{bmatrix}
$$

则：

$$
DU =
\begin{bmatrix}
u_0\\
u_1-u_0\\
u_2-u_1\\
\vdots
\end{bmatrix}
$$

这条式子就是整个输入变化率约束写成 QP 形式的核心。

它告诉我们：

* 第一行取出的是 $u_0$
* 第二行取出的是 $u_1-u_0$
* 第三行取出的是 $u_2-u_1$
* 以此类推

因此，原本逐项写的差分约束，现在可以统一改写为：

$$
lbA \le D U \le ubA
$$

这一步的本质是“结构重写”：

> 没有改变任何物理含义，只是把一串标量不等式改写成了一个矩阵不等式

---

## 15.5 为什么第一项要单独处理

对 $k\ge 1$ 的项，差分总是：

$$
u_k-u_{k-1}
$$

但对第一步来说，并不存在预测时域内部的 $u_{-1}$。  
第一个控制量真正要比较的对象，不是“预测时域中的前一个量”，而是：

> 上一个控制周期已经实际发出去的控制命令

记它为 $u_{\mathrm{prev}}$，控制周期为 $T_c$，则第一项约束应写成：

$$
u_{\mathrm{prev}}-\dot\delta_{\max,0}T_c
\le
u_0
\le
u_{\mathrm{prev}}+\dot\delta_{\max,0}T_c
$$

这非常重要，因为它把当前优化问题和上一拍真实执行过的控制量接起来了。  
没有这一步，优化器就可能在当前时刻给出一个相对上一拍跳变过大的新命令。

所以完整的差分约束实际是：

$$
D U =
\begin{bmatrix}
u_0\\
u_1-u_0\\
u_2-u_1\\
\vdots
\end{bmatrix}
$$

并配套边界：

$$
lbA =
\begin{bmatrix}
u_{\mathrm{prev}}-\dot\delta_{\max,0}T_c\\
-\dot\delta_{\max,1}\Delta t\\
-\dot\delta_{\max,2}\Delta t\\
\vdots
\end{bmatrix}
$$

$$
ubA =
\begin{bmatrix}
u_{\mathrm{prev}}+\dot\delta_{\max,0}T_c\\
\dot\delta_{\max,1}\Delta t\\
\dot\delta_{\max,2}\Delta t\\
\vdots
\end{bmatrix}
$$

这样写以后，第 0 行和后续各行就被统一装进了同一个矩阵不等式里。

---

## 15.6 最终约束的统一写法

因此，这个 MPC 对输入的约束可以概括成两组：

### 幅值约束

$$
lb \le U \le ub
$$

### 变化率约束

$$
lbA \le D U \le ubA
$$

如果愿意，也可以进一步把二者堆成一个更大的线性约束系统：

$$
\begin{bmatrix}
lb\\
lbA
\end{bmatrix}
\le
\begin{bmatrix}
I\\
D
\end{bmatrix}U
\le
\begin{bmatrix}
ub\\
ubA
\end{bmatrix}
$$

这说明一个很本质的事实：

> 只要模型是线性的、约束是对输入的线性不等式、代价函数是二次型，问题就天然是 QP

---

# 16. QP 求解器到底在解什么数学问题

从原理上说，MPC 在这一层解的是：

> 一个关于未来输入序列 $U$ 的凸二次优化问题

它之所以是“二次”的，来自二次代价函数；  
它之所以是“带约束”的，来自输入幅值和输入变化率限制。

---

## 16.1 为什么 MPC 最后会落到 QP

回顾前面的链条：

* 预测模型是线性的：
  $$
  X_{ex}=A_{ex}x_0+B_{ex}U_{ex}+W_{ex}
  $$
* 输出也是线性的：
  $$
  Y_{ex}=C_{ex}X_{ex}
  $$
* 代价函数是二次型：
  $$
  J=
  Y_{ex}^TQ_{ex}Y_{ex}
  +(U_{ex}-U_{ref,ex})^TR_{1ex}(U_{ex}-U_{ref,ex})
  +U_{ex}^TR_{2ex}U_{ex}
  $$
* 约束是线性不等式：
  $$
  lb \le U \le ub,\qquad
  lbA \le D U \le ubA
  $$

这四点一组合，就已经把问题的类型完全定死了：

> 线性预测模型 + 二次代价函数 + 线性约束  
> 就是标准的 Quadratic Programming

所以 QP 不是“额外选的求解工具”，而是这套建模自然导出的数学结果。

---

## 16.2 把输出写成关于 $U$ 的仿射函数

从预测方程出发：

$$
X = Ax_0 + BU + W
$$

$$
Y = CX = C(Ax_0 + BU + W)
$$

把与 $U$ 无关的部分记为：

$$
d := C(Ax_0 + W)
$$

把输入到输出的映射记为：

$$
M := CB
$$

于是：

$$
Y = d + MU
$$

这一步非常关键。它告诉我们：

* $d$ 描述“在当前状态 $x_0$、当前参考轨迹和偏置项 $W$ 已知时，不考虑优化变量也会自然产生的输出偏移”
* $MU$ 描述“未来输入序列 $U$ 会怎样改变整个预测输出”

因此，从优化角度看，问题已经被压缩成：

> 用输入序列 $U$ 去修正一个已有偏移 $d$

---

## 16.3 从总代价函数推到二次型

把：

$$
Y=d+MU
$$

代回总代价函数：

$$
J=
Y^TQY+(U-U_r)^TR_1(U-U_r)+U^TR_2U
$$

得到：

$$
J=
(d+MU)^TQ(d+MU)
+
(U-U_r)^TR_1(U-U_r)
+
U^TR_2U
$$

下面分三部分展开。

### 第一部分：状态跟踪项

$$
(d+MU)^TQ(d+MU)=
U^TM^TQMU
+
2d^TQMU
+
d^TQd
$$

这里：

* $U^TM^TQMU$ 是二次项，表示输入变化对跟踪误差带来的二阶影响
* $2d^TQMU$ 是一次项，表示当前偏差 $d$ 会把最优解往某个方向“推过去”
* $d^TQd$ 是常数项，不依赖于 $U$

### 第二部分：输入偏离前馈项

$$
(U-U_r)^TR_1(U-U_r)=
U^TR_1U
-2U_r^TR_1U
+U_r^TR_1U_r
$$

这里：

* $U^TR_1U$ 惩罚输入本身过大
* $-2U_r^TR_1U$ 体现“最优输入倾向于贴近参考前馈输入 $U_r$”
* $U_r^TR_1U_r$ 仍然是常数项

### 第三部分：输入平滑项

$$
U^TR_2U
$$

它本身已经是标准二次型，无需再展开。

---

## 16.4 合并得到标准二次目标函数

把三部分合并：

$$
J=U^T(M^TQM+R_1+R_2)U+
2(d^TQM-U_r^TR_1)U+
\text{const}
$$

记：

$$
G := M^TQM+R_1+R_2
$$

$$
c := M^TQd-R_1U_r
$$

则：

$$
J(U)=U^TGU+2c^TU+\text{const}
$$

由于常数项不影响最优解，可以省略，于是优化问题只剩下：

$$
\min_U\; U^TGU+2c^TU
$$

这已经是一个非常标准的二次优化问题了。

---

## 16.5 为什么教材里常写成 $\frac12 U^T H U + g^T U$

数学上，二次规划最常见的标准形式是：

$$
\min_U \frac12 U^T H U + g^T U
$$

它和上面的写法完全等价，只是记号不同。

若从：

$$
J(U)=U^TGU+2c^TU
$$

变成标准形式，只需令：

$$
H = 2G,\qquad g = 2c
$$

于是：

$$
\frac12 U^T H U + g^T U=
\frac12 U^T(2G)U + (2c)^TU=
U^TGU + 2c^TU
$$

所以这里的 $\frac12$ 没有任何新的物理意义，它只是一个为了让导数更简洁的记号约定：

$$
\nabla\!\left(\frac12 U^T H U\right)=HU
\qquad (\text{当 } H \text{ 对称时})
$$

也就是说：

> 标准 QP 形式中的 $\frac12$ 是“记号规范”，不是“新的控制假设”

---

## 16.6 Hessian 为什么一定是对称的

由定义：

$$
G = M^TQM + R_1 + R_2
$$

若：

$$
Q \succeq 0,\qquad R_1 \succeq 0,\qquad R_2 \succeq 0
$$

则：

$$
M^TQM \succeq 0
$$

因此：

$$
G \succeq 0
$$

从而标准形式中的：

$$
H = 2G
$$

也是对称半正定的。

这意味着：

* 目标函数是凸的
* 只要约束集合非空，优化问题就是凸 QP
* 若再进一步有 $R_1+R_2 \succ 0$，通常还能保证解更稳定、甚至唯一

这就是为什么在线性 MPC 里我们特别喜欢二次型代价函数：

> 它既能表达跟踪误差、输入大小、输入平滑性，又能保持问题的凸性

---

## 16.7 一次项 $g$ 的物理意义是什么

很多人第一次学 QP 时，只记住了 Hessian，却不太明白一次项为什么存在。

从上面的推导可知：

$$
c = M^TQd - R_1U_r
$$

所以一次项的来源有两部分：

### 来源 1：当前偏差 $d$

若当前状态已经偏离参考，那么即便未来输入全为零，输出也会带着偏移量：

$$
d = C(Ax_0 + W)
$$

这一项会把优化器“推向能消除当前误差的方向”。

### 来源 2：前馈输入 $U_r$

若参考轨迹本身是弯的，那么理想情况下系统本来就不该围绕零输入工作，而应围绕某个参考转向输入工作：

$$
U_r
$$

这一项会把优化器“推向靠近参考前馈输入的方向”。

所以可以把一次项理解成：

> 当前误差和参考曲率共同告诉优化器：最优解应当朝哪个方向偏移

而 Hessian 则告诉优化器：

> 往那个方向偏移时，会付出多大的二次代价

---

## 16.8 如果没有约束，最优解长什么样

为了建立直觉，先暂时去掉所有约束，只看：

$$
\min_U \frac12 U^T H U + g^T U
$$

若 $H$ 可逆，则一阶最优条件为：

$$
\nabla J = HU + g = 0
$$

因此：

$$
U^\star = -H^{-1}g
$$

这条式子很有启发性，因为它说明：

* 若没有物理限制，最优解只是一个线性代数问题
* 真正让问题变成“需要 QP 求解器”的，不是二次代价本身，而是线性约束的加入

一旦加入：

$$
lb \le U \le ub,\qquad
lbA \le D U \le ubA
$$

最优解就不再是简单的闭式解，而必须在可行域边界和目标函数之间做折中。

---

## 16.9 最终的完整 QP 数学形式

因此，基于前面的预测模型、代价函数和输入约束，最终得到的 QP 可以完整写成：

$$
\min_U \frac12 U^T H U + g^T U
$$

其中：

$$
H = 2(B_{ex}^TC_{ex}^TQ_{ex}C_{ex}B_{ex}+R_{1ex}+R_{2ex})
$$

$$
g =
2\Big(
B_{ex}^TC_{ex}^TQ_{ex}C_{ex}(A_{ex}x_0+W_{ex})
-R_{1ex}U_{ref,ex}
\Big)
$$

满足：

$$
lb \le U \le ub
$$

$$
lbA \le D U \le ubA
$$

其中：

$$
lb = -\delta_{\max}\mathbf 1,\qquad
ub = \delta_{\max}\mathbf 1
$$

并且：

$$
lbA =
\begin{bmatrix}
u_{\mathrm{prev}}-\dot\delta_{\max,0}T_c\\
-\dot\delta_{\max,1}\Delta t\\
\vdots
\end{bmatrix},
\qquad
ubA =
\begin{bmatrix}
u_{\mathrm{prev}}+\dot\delta_{\max,0}T_c\\
\dot\delta_{\max,1}\Delta t\\
\vdots
\end{bmatrix}
$$

这就是求解器真正面对的数学问题。

如果某个实现把目标函数写成：

$$
U^THU + 2f^TU
$$

那只是符号约定不同，和上面的标准形式完全等价。  
本质上，求解器解的始终是同一个问题：

> 在物理约束允许的范围内，寻找一串未来控制输入，使未来预测误差最小、输入不过大、输入变化又足够平滑

---

## 16.10 这个 QP 一般是怎么求解的

到这里为止，我们已经把 MPC 写成了一个标准凸二次规划：

$$
\min_U \frac12 U^T H U + g^T U
$$

满足：

$$
lb \le U \le ub,\qquad
lbA \le D U \le ubA
$$


---

### 先看没有约束时会怎样

如果暂时把所有约束都拿掉，那么问题退化为：

$$
\min_U \frac12 U^T H U + g^T U
$$

对 $U$ 求导并令梯度为零：

$$
\nabla J = HU + g = 0
$$

因此，若 $H$ 可逆，则：

$$
U^\star = -H^{-1}g
$$

这说明一件很重要的事：

* 二次目标函数本身并不难
* 真正让问题变成 QP 的，是约束

一旦加入转角幅值约束和转角变化率约束，最优解就不能再直接写成一个闭式公式，而必须在“目标最优”和“约束可行”之间寻找平衡。

---

### 统一写成单边不等式

为了更方便讨论求解原理，可以把双边约束统一改写成单边不等式：

$$
G U \le h
$$

例如可以写成：

$$
G =
\begin{bmatrix}
I\\
-I\\
D\\
-D
\end{bmatrix},
\qquad
h =
\begin{bmatrix}
ub\\
-lb\\
ubA\\
-lbA
\end{bmatrix}
$$

于是整个问题变成：

$$
\min_U \frac12 U^T H U + g^T U,\qquad
G U \le h
$$

这就是教科书里最常见的凸 QP 形式。

---

### 最优解满足什么条件：KKT 条件

对这个带不等式约束的凸 QP，最优解满足 KKT 条件。

引入拉格朗日乘子：

$$
\lambda \ge 0
$$

构造拉格朗日函数：

$$
\mathcal L(U,\lambda)=
\frac12 U^T H U + g^T U + \lambda^T(GU-h)
$$

最优解满足：

$$
HU + g + G^T\lambda = 0
$$

$$
GU - h \le 0
$$

$$
\lambda \ge 0
$$

$$
\lambda_i (GU-h)_i = 0,\qquad \forall i
$$

这四条分别表示：

* 驻点条件：目标函数梯度与约束反作用力平衡
* 原始可行性：解必须满足原约束
* 对偶可行性：拉格朗日乘子非负
* 互补松弛：只有真正“卡住”的约束才会对应非零乘子

所以从本质上说，所有 QP 求解器都在做同一件事：

> 找到一组满足 KKT 条件的 $(U,\lambda)$

---

### 常见求解方法 1：Active-Set

Active-Set 方法的核心想法是：

> 先猜哪些约束在最优点会正好卡住

把这些“活跃约束”记为：

$$
G_A U = h_A
$$

那么原问题暂时变成一个**等式约束二次规划**：

$$
\min_U \frac12 U^T H U + g^T U,\qquad
G_A U = h_A
$$

它对应的 KKT 线性系统为：

$$
\begin{bmatrix}
H & G_A^T\\
G_A & 0
\end{bmatrix}
\begin{bmatrix}
U\\
\lambda_A
\end{bmatrix}=
\begin{bmatrix}
-g\\
h_A
\end{bmatrix}
$$

解出之后，再检查：

* 有没有别的约束被违反
* 当前 active set 里的某些乘子是否变成负值

如果有，就调整活跃约束集合，再重复。

这个方法很适合 MPC，因为相邻时刻的问题往往很像，活跃约束集合也常常变化不大，所以 warm start 很有效。

---

### 常见求解方法 2：Interior-Point

Interior-Point 的思路不是去猜哪条约束活跃，而是始终待在可行域内部，然后逐渐逼近边界上的最优点。

做法通常是先引入松弛变量：

$$
GU + s = h,\qquad s>0
$$

再构造带对数障碍项的目标函数：

$$
\min_U \frac12 U^T H U + g^T U - \mu \sum_i \log s_i
$$

其中 $\mu>0$ 是 barrier 参数。

当 $\mu$ 较大时，解会远离约束边界；当 $\mu \to 0$ 时，解会逐渐逼近原始 QP 的最优点。

这类方法的特点是：

* 数值上通常很稳健
* 对中大型问题很强
* 每一步往往需要解较大的牛顿线性系统

本质上，它是在“可行域内部”沿着一条中心路径逐步逼近最优解。

---

### 常见求解方法 3：Operator Splitting / ADMM

还有一类方法不直接一次性求解完整 KKT 系统，而是把问题拆成几个更容易的子问题交替求解。

典型思想是：

* 一步解一个带正则项的二次问题
* 一步把结果投影回约束集合
* 一步更新乘子或对偶变量

这类方法的代表就是 ADMM 风格的 QP 求解器。

它的优点通常是：

* 算法结构简单
* 很适合稀疏大规模问题
* 很适合嵌入式和实时优化
* warm start 也比较自然

它的缺点是：

* 相比高精度 interior-point，往往需要更多迭代
* 但对 MPC 这类“每拍都要很快给出一个够好的答案”的问题，往往已经非常合适

---

### 对 MPC 来说一般怎么选

从控制应用的角度看，常见选择大致是：

* 若问题规模中小、相邻时刻变化不大、希望充分利用 warm start，常用 Active-Set
* 若问题规模更大、追求稳健和高精度，常用 Interior-Point
* 若问题稀疏、强调实时性和工程实现便利，常用 ADMM / OSQP 这一类方法


---

# 17. 整套链条的统一大图景

总链条：

```text
理想阿克曼几何
→ \tan\delta = L/R
→ 世界坐标自行车模型
→ Frenet坐标定义：e, θe
→ 精确非线性误差模型
→ 小误差近似
→ 执行器动态：τ
→ 连续时间线性仿射模型
→ Tustin离散化
→ 单步离散模型
→ N步批量预测方程
→ 代价函数设计（误差、输入、平滑）
→ 代入预测方程
→ QP标准形式
→ 输入与输入差分约束
→ QP求解器
→ 得到最优未来控制序列
→ 执行第一步输入并滚动优化
```

---

# 18. 常见误区与统一澄清

---

## 18.1 “代价函数是推导出来的吗？”

不是。
更准确地说：

* 动力学模型是推导出来的
* 代价函数是根据控制目标设计出来的

但一旦设计好，它就可以严格地代入模型并整理为 QP。

---

## 18.2 “$\tau$ 是阿克曼几何里推出来的吗？”

不是。
$\tau$ 是执行器动态参数，用来描述“转角不会瞬间到位”。

---

## 18.3 “为什么还要前馈输入 $U_{ref}$？”

因为沿参考曲率走本来就需要一个理想转角：

$$
u_{ref}=\arctan(L\kappa)
$$

MPC 只优化相对于这个前馈值的偏差，能让控制更自然，也减小误差负担。

---

## 18.4 “为什么平滑项会变成 $U^TR_2U$ 而不是显式写 $(u_i-u_{i-1})^2$”

因为所有差分平方项展开后，本质上都能写成一个整体二次型：

$$
U^TR_2U
$$

这样求解器更容易处理。

---

## 18.5 “为什么最后只执行第一个输入？”

因为 MPC 是滚动优化。未来总会变化，所以：

* 先优化整个未来
* 但只执行眼前最优那一步
* 下一时刻重新测量、重新预测、重新优化
