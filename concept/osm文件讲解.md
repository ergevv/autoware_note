这个文件本质上是 **Lanelet2 的 OSM/XML 矢量高精地图**，不是图片地图。它描述的是“点、线、车道区域、交通规则”的结构化数据。

入口看这里：[lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:1)

**地图是怎么被 Autoware 使用的**

启动时 `lanelet2_map_loader` 读取这个 `.osm`，用 `map_projector_info.yaml` 里的 `MGRS 54SVE` 投影信息，把经纬度点投影到 Autoware 的 `map` 坐标系，然后发布成 `LaneletMapBin`。相关代码在 [lanelet2_map_loader_node.cpp](/home/erge/work/autoware_docker/autoware_devel/src/core/autoware_core/map/autoware_map_loader/src/lanelet2_map_loader/lanelet2_map_loader_node.cpp:137)。

这个文件头部写着 `generator='JOSM'`，说明它最后是由 JOSM 保存/编辑的：[lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:2)。但从 XML 本身看不出最原始来源，通常是通过点云/航测/人工标注/HD Map 工具生成车道边界，再导出成 Lanelet2 OSM。

**点是什么**

`node` 是地图最底层的几何点，带有：

- `lat/lon`：经纬度
- `ele`：高程
- `mgrs_code`：MGRS 网格

例如文件开头的点：[lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:3)

**车道线怎么来的**

车道线在文件里通常是 `way`，由一串 `node ref` 连接成折线。比如：

```xml
<way id='235'>
  <nd ref='1289' />
  ...
  <tag k='subtype' v='solid' />
  <tag k='type' v='line_thin' />
  <tag k='width' v='0.200' />
</way>
```

这表示一条实体细线，宽度 0.2m，位置由这些节点坐标决定。见 [lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:21299)。

所以“车道线”不是运行时识别出来的，而是制图时已经标注进 `.osm` 里的。这个文件里有：

- `line_thin`：195 条，表示车道标线
- `solid/dashed`：实线/虚线
- `stop_line`：95 条停止线
- `virtual`：173 条虚拟边界，常用于路口、连接段、没有真实画线的地方

**边界怎么来的**

Lanelet2 里的一个车道不是中心线，而是一个 **左右边界夹出来的有向区域**。

例如 relation 1：

```xml
<relation id='1'>
  <member type='way' ref='233' role='left' />
  <member type='way' ref='236' role='right' />
  <tag k='speed_limit' v='30' />
  <tag k='turn_direction' v='straight' />
  <tag k='type' v='lanelet' />
</relation>
```

见 [lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:26644)。

意思是：`way 233` 是这个 lanelet 的左边界，`way 236` 是右边界，两条线之间就是可行驶区域。这里的左右边界可以是真实车道线，也可以是 `virtual` 虚拟线，例如 [way 233](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:21292) 和 [way 236](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:21309)。

另外还有一种 `road_border`，它更像道路外缘/道路边界，不一定等同于某个 lanelet 的左右边界。例子在 [lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:23799)。

**中心线在哪里**

这个文件里没有显式搜索到 `centerline` 标签。Autoware 加载后会根据左右边界生成 lanelet 的中心线，配置里分辨率是 `5.0m`，见 [lanelet2_map_loader.param.yaml](/home/erge/work/autoware_docker/autoware_devel/src/core/autoware_core/map/autoware_map_loader/config/lanelet2_map_loader.param.yaml:4)。

生成逻辑在这里：[utilities.cpp](/home/erge/work/autoware_docker/autoware_devel/src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/utilities.cpp:505)。

**它描述了什么信息**

这个文件里大概有：

- 4499 个 `node`
- 625 条 `way`
- 241 个 `relation`
- 190 个 `lanelet`
- 183 个道路 lanelet
- 4 个人行横道 lanelet
- 2 个人行道 lanelet
- 51 个交通规则 relation

除了车道几何，它还描述：

- 限速：`speed_limit`
- 单向：`one_way`
- 转向：`turn_direction`
- 停止线：`stop_line`
- 交通灯：`traffic_light`
- 交通标志：`traffic_sign`
- 虚拟交通灯：`virtual_traffic_light`
- 让行/优先权：`right_of_way`
- 人行横道、停车区域、检测区域、禁停区域等

一句话总结：这个 `.osm` 文件就是 Autoware 的“道路语义地图”。车道由左右边界线组成，边界线由经纬度点串成，真实车道线来自制图标注，虚拟边界用于补足没有实体线但规划仍需要边界的地方。


这里的 `left` / `right` **默认指单个 lanelet，也就是单个车道单元的左右边界**，不是多车道整体取最远左右边界。

例如：

[lanelet2_map.osm](/home/erge/work/autoware_docker/autoware_map/sample-map-planning/lanelet2_map.osm:26644)

```xml
<relation id='1'>
  <member type='way' ref='233' role='left' />
  <member type='way' ref='236' role='right' />
  ...
  <tag k='type' v='lanelet' />
</relation>
```

这表示：`relation id=1` 这个 lanelet 的可行驶区域，由 `way 233` 和 `way 236` 两条边界夹出来。

关键点：

- `left/right` 是 **沿着该 lanelet 行驶方向看** 的左边和右边。
- 多车道道路会被拆成多个 lanelet，每个车道一个或多个 lanelet。
- 相邻车道之间的车道线，通常会作为一个 lanelet 的右边界、另一个 lanelet 的左边界。
- 最外侧的道路边缘可能是外侧 lanelet 的边界，也可能另有 `type='road_border'` 描述道路整体外缘。
- `road_border` 更像“道路边界/路缘/外轮廓”，不是每个 lanelet 的左右边界。

所以如果有三车道，大致是这样：

```text
道路最左边界
│
│  lanelet A
│  left = 道路左边/左车道线
│  right = A/B 中间车道线
│
│  lanelet B
│  left = A/B 中间车道线
│  right = B/C 中间车道线
│
│  lanelet C
│  left = B/C 中间车道线
│  right = 道路右边/右车道线
│
道路最右边界
```

只有当某些规划模块把多个 lanelet 合并成“可行驶区域”或“道路走廊”时，才可能取一组 lanelet 的最外侧边界。但那是运行时算法处理，不是 Lanelet2 文件里单个 `lanelet` 的定义。