# CyberDog Web Bridge API Handoff

这份文档面向后续接手前端展示层的人，重点是接口契约与坐标规则。

## 1. 架构边界

浏览器永远不要直接碰 ROS2 DDS。

正确链路是：

```text
Browser -> cyberdog_web_bridge -> ROS2 / cyberdog_inspection
```

其中：

- `cyberdog_nav2_lidar` 提供 `/map`、`/amcl_pose`、`/plan`、`/local_plan`
- `cyberdog_inspection` 提供巡检控制和巡检规划结果
- `cyberdog_web_bridge` 统一转成前端友好的 REST 和 WebSocket

## 2. 页面最低需要的数据

### 地图底图

来源：

- `GET /api/v1/maps`
- `GET /api/v1/maps/{scene_name}/metadata`
- `GET /api/v1/maps/{scene_name}/image`

说明：

- 页面不需要自己解析 ROS `OccupancyGrid`
- 桥接层已经把地图 yaml + pgm 转成了浏览器可直接显示的 png 图像

### 机器人位姿

来源：

- `GET /ws/live`
- `GET /api/v1/live`

字段：

- `robot_pose.x`
- `robot_pose.y`
- `robot_pose.yaw`

### 路线

来源：

- `planned_path`
  - 巡检规划路径，来自 `cyberdog_inspection`
- `global_path`
  - Nav2 全局路径
- `local_path`
  - Nav2 局部路径

## 3. 地图坐标规则

`/api/v1/maps/{scene_name}/metadata` 会给出：

- `resolution`
- `origin`
- `width`
- `height`
- `world_bounds`

地图原点规则和 ROS map yaml 一致：

- `origin[0]` 是世界坐标最小 `x`
- `origin[1]` 是世界坐标最小 `y`
- 图片像素坐标左上角为 `(0, 0)`
- 世界坐标原点在图片左下角对应位置

因此换算要做 Y 翻转。

### 世界坐标 -> 画布像素

```js
px = (x - origin[0]) / resolution
py = height - 1 - (y - origin[1]) / resolution
```

### 画布像素 -> 世界坐标

```js
x = origin[0] + px * resolution
y = origin[1] + (height - 1 - py) * resolution
```

当前极简页面已经按这个规则实现，参考：

- `cyberdog_web_bridge/static/app.js`

## 4. 设置 AMCL 初始位姿

接口：

- `POST /api/v1/localization/initial-pose`

请求体：

```json
{
  "x": 1.2,
  "y": -0.4,
  "yaw": 1.57,
  "covariance_xy": 0.25,
  "covariance_yaw": 0.1
}
```

语义：

- `x`、`y`、`yaw` 全是 `map` 坐标系
- 桥接层会发布 `PoseWithCovarianceStamped` 到 `/initialpose`

当前页面交互定义：

- 鼠标按下点是位置
- 拖拽方向定义朝向
- 松开时提交

## 5. 巡检控制接口

### 初始化巡检

- `POST /api/v1/inspection/initialize`

```json
{
  "scene_name": "map"
}
```

### 控制

- `POST /api/v1/inspection/start`
- `POST /api/v1/inspection/pause`
- `POST /api/v1/inspection/resume`
- `POST /api/v1/inspection/stop`

### 状态

- `GET /api/v1/inspection/status`
- `GET /api/v1/inspection/planned-path`
- `GET /ws/live`

## 6. `ws/live` 消息结构

当前推送结构如下：

```json
{
  "robot_pose": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "frame_id": "map",
    "stamp_sec": 0.0
  },
  "global_path": [
    { "x": 0.0, "y": 0.0, "yaw": 0.0 }
  ],
  "local_path": [
    { "x": 0.0, "y": 0.0, "yaw": 0.0 }
  ],
  "inspection_status": {
    "success": true,
    "inspection_status": 2,
    "inspection_status_name": "READY",
    "message": "Inspection path ready",
    "nav2_status": "",
    "active_scene_name": "map"
  },
  "planned_path": [
    [0.0, 0.0],
    [1.0, 0.0]
  ],
  "map_artifacts": {
    "success": true,
    "scene_name": "map",
    "latest_gif_path": "/abs/path/to/file.gif",
    "debug_map_paths": [
      "/abs/path/to/00_original_map.png"
    ]
  },
  "timestamp": 0.0
}
```

前端重做时，优先保持这个结构兼容。

## 7. 重做 UI 时建议保留的分层

建议继续沿用当前分层，不要把坐标换算和 DOM/样式混在一起：

- `api-client`
  - 负责 REST 和 WebSocket
- `map-transform`
  - 负责世界坐标与像素坐标互转
- `render-layer`
  - 负责地图底图和各种 overlay
- `interaction-layer`
  - 负责点击、拖拽、表单提交

## 8. 当前已知简化点

- 地图底图是运行时把 `pgm` 转成 `png` 响应，没有做缓存层
- 没有做多用户会话隔离，所有浏览器看到的是同一台机器人当前状态
- `/ws/live` 是定时推送，不是增量事件模型
- `map_artifacts` 主要用于调试和交接，不是正式 UI 主数据源

## 9. 建议的后续增强

- 给地图图片加缓存头和 ETag
- 给 WebSocket 增加事件类型字段
- 增加 inspection 进度百分比和当前 waypoint 下标
- 增加地图缩略图列表接口
- 增加认证或局域网访问控制
