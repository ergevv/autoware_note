你可以把 `route sections` 理解成：

**Mission Planner 给下游的一串“道路切片”。**

不是一条线，也不是单个车道，而是：

```text
整条路线 = section 0 + section 1 + section 2 + ...
每个 section = 这一小段纵向位置上，允许使用哪些 lanelet
```

源码消息结构非常直接：

```text
LaneletRoute
  start_pose
  goal_pose
  segments[]   <- 这里就是 route sections

LaneletSegment
  preferred_primitive
  primitives[]
```

对应文件：

- [LaneletRoute.msg](/home/erge/work/autoware_note/autoware_devel/src/core/autoware_msgs/autoware_planning_msgs/msg/LaneletRoute.msg:4)
- [LaneletSegment.msg](/home/erge/work/autoware_note/autoware_devel/src/core/autoware_msgs/autoware_planning_msgs/msg/LaneletSegment.msg:1)
- [default_planner.cpp](/home/erge/work/autoware_note/autoware_devel/src/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp:355)

**先说 section 是什么**

假设道路有三条同向车道，地图把每条车道沿前进方向切成很多小块 lanelet：

```text
左车道:   L1 --- L2 --- L3 --- L4
中车道:   M1 --- M2 --- M3 --- M4
右车道:   R1 --- R2 --- R3 --- R4
```

Mission Planner 算出来的主路线可能是：

```text
M1 -> M2 -> M3 -> M4
```

如果只把这条主线发给下游，下游就会很尴尬：  
Behavior Path Planner 想换道、避障、靠边、绕一点点，都不知道旁边哪些车道属于“本次任务允许使用的范围”。

所以 Autoware 不只输出：

```text
M1 -> M2 -> M3 -> M4
```

而是输出一串 section：

```text
section 0: preferred = M1, primitives = [L1, M1, R1]
section 1: preferred = M2, primitives = [L2, M2, R2]
section 2: preferred = M3, primitives = [L3, M3, R3]
section 3: preferred = M4, primitives = [L4, M4, R4]
```

这里每个 `section` 就像沿道路前进方向切下来的“一片横截面/纵向小段”。它告诉下游：在这一段路上，主路线推荐走哪条 lanelet，旁边哪些 lanelet 也属于可用走廊。

**preferred_primitive 是什么**

`preferred_primitive` 就是这个 section 里“主路线推荐走的那个 lanelet”。

比如：

```text
section 1:
  preferred_primitive = M2
```

意思是：在这段路上，Mission Planner 的主路径认为车应该优先沿着 `M2` 走。

它通常来自 Lanelet 图搜索得到的最短/推荐 lanelet 序列。源码里先调用：

```cpp
planPathLaneletsBetweenCheckpoints(...)
```

得到主路径 `path_lanelets`，然后再生成 route sections。

所以 `preferred_primitive` 的直觉是：

```text
导航主线
```

或者更口语一点：

```text
这段路默认跟着它走。
```

**primitives 是什么**

`primitives` 是这个 section 里“所有允许下游使用的 lanelet 集合”，通常包括：

```text
preferred_primitive 本身
+ 同方向相邻车道
+ 满足路线走廊规则的其他 lanelet
```

官方 README 的定义也很清楚：`primitives` 是同方向相邻车道集合，并且包含 preferred lane。

所以：

```text
section 1:
  preferred_primitive = M2
  primitives = [L2, M2, R2]
```

含义是：

```text
M2 是推荐主车道；
但这一段里 L2、M2、R2 都属于 route 允许范围。
```

这对下游特别重要。比如 Behavior Path Planner 要做换道：

```text
如果 primitives 只有 [M2]
  下游只能老老实实待在 M2

如果 primitives 是 [L2, M2, R2]
  下游知道左边 L2、右边 R2 也在本次路线走廊里
  才能考虑换道、避障、借一点空间
```

**为什么叫 primitive**

这里的 `primitive` 不要理解成“原始类型 int/string”那个 primitive。  
在 Autoware 消息里，`LaneletPrimitive` 很简单：

```text
int64 id
string primitive_type
```

也就是说它不是完整几何图形，只是一个“地图元素引用”。

例如：

```text
{id: 12345, primitive_type: "lane"}
```

它的意思是：

```text
去 LaneletMap 里找 id = 12345 的那个 lanelet / lane primitive
```

Mission Planner 的 route 消息不直接塞入 lanelet 的左右边界、中心线等几何数据，而是发 ID。下游拿到 ID 后，再结合本地 LaneletMap 查几何信息。

**一句话总括**

```text
Route sections = 一串道路切片
section = 某一小段路的车道级可行驶范围
preferred_primitive = 这个切片里推荐走的主 lanelet
primitives = 这个切片里允许下游使用的所有 lanelet，包含 preferred
```

最适合讲给新手的比喻是：

```text
preferred_primitive 像导航软件画出来的蓝色主线；
primitives 像这段路上导航允许你使用的车道范围；
route sections 则是把整条导航路线按前进方向切成一段一段，每段都标清主线和可用车道。
```

这样下游拿到的就不是“请去目标点”这么模糊的信息，而是：

```text
你要从 start 到 goal；
每一段推荐走哪条 lanelet；
每一段旁边哪些 lanelet 也属于本次任务允许使用的走廊。
```