# cyberdog_web_bridge

`cyberdog_web_bridge` 是运行在 Jetson 上的网页桥接层。

它不让浏览器直接接触 ROS2 DDS，而是把页面需要的数据和控制能力统一转换为 `HTTP + WebSocket`:

- 来自 `cyberdog_nav2_lidar` 的地图、AMCL 位姿、Nav2 全局/局部路径
- 来自 `cyberdog_inspection` 的巡检状态、地图列表、规划路线、开始/暂停/恢复/停止
- 面向网页的 AMCL 初始位姿设置接口

## 1. 设计目标

在你当前 DDS 依旧绑定狗子网线网卡的前提下，网页可视化走 Jetson 自己的 Web 服务。

这意味着只要浏览器能访问 Jetson 所在局域网 IP，就可以打开可视化页面。

- 可以是 Jetson 接入普通 Wi-Fi
- 可以是 Jetson 接到你自己的交换机/路由器
- 也可以是 Jetson 自己发热点

前提只有两个：

- 浏览器和 Jetson 网络互通
- Jetson 上的 `cyberdog_nav2_lidar`、`cyberdog_inspection`、`cyberdog_web_bridge` 已按顺序启动

## 2. 包内内容

- `cyberdog_web_bridge/bridge_node.py`
  - ROS2 + FastAPI 主桥接节点
- `launch/web_bridge.launch.py`
  - 启动入口
- `cyberdog_web_bridge/static/index.html`
  - 极简功能页
- `cyberdog_web_bridge/static/app.js`
  - 前端数据适配、地图坐标换算、Canvas 绘制
- `cyberdog_web_bridge/static/styles.css`
  - 极简样式
- `API_HANDOFF.md`
  - 给后续前端重做时使用的对接文档

## 3. 启动顺序

先启动 Nav2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_nav2_lidar bringup.launch.py
```

再启动巡检服务:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_inspection cyberdog_inspection.launch.py \
  config:=$PWD/src/cyberdog_inspection/config/inspection_config.yaml
```

最后启动网页桥接:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_web_bridge web_bridge.launch.py
```

默认监听:

- 页面: `http://<jetson-ip>:8090`
- WebSocket: `ws://<jetson-ip>:8090/ws/live`

## 4. 可配置参数

`launch/web_bridge.launch.py` 暴露了这些参数:

- `http_host`
  - 默认 `0.0.0.0`
- `http_port`
  - 默认 `8090`
- `inspection_base_url`
  - 默认 `http://127.0.0.1:8083`
- `inspection_poll_sec`
  - 默认 `0.5`
- `ws_interval_sec`
  - 默认 `0.5`

示例:

```bash
ros2 launch cyberdog_web_bridge web_bridge.launch.py \
  http_port:=8091 \
  inspection_poll_sec:=1.0 \
  ws_interval_sec:=0.2
```

## 5. 对外 API

### REST

- `GET /api/v1/health`
- `GET /api/v1/maps`
- `GET /api/v1/maps/{scene_name}/metadata`
- `GET /api/v1/maps/{scene_name}/image`
- `GET /api/v1/inspection/status`
- `GET /api/v1/inspection/planned-path`
- `GET /api/v1/live`
- `POST /api/v1/inspection/initialize`
- `POST /api/v1/inspection/start`
- `POST /api/v1/inspection/pause`
- `POST /api/v1/inspection/resume`
- `POST /api/v1/inspection/stop`
- `POST /api/v1/localization/initial-pose`

### WebSocket

- `GET /ws/live`
  - 周期推送当前机器人位姿、全局路径、局部路径、巡检状态、巡检规划路径

## 6. 现有极简页面支持的功能

- 地图场景列表和地图底图预览
- 机器人当前位置和朝向显示
- 巡检规划路径显示
- Nav2 全局/局部路径显示
- 地图拖拽设置 AMCL 初始位姿
- 开始/暂停/恢复/停止巡检

## 7. 注意事项

- 浏览器访问这层服务不依赖 DDS 发现
- 真正依赖 DDS 的仍然是 Jetson 与 CyberDog、Jetson 与 Nav2 运行栈之间
- 如果 `cyberdog_inspection` 没启动，桥接接口会返回 `503`
- 如果 ROS2 没提供 `/amcl_pose`、`/plan`、`/local_plan`，页面仍能打开，但对应覆盖层为空

 ## 8. 查看 Nav2 实时参数

运行时查看/修改规划器等参数：

```bash
# 列出 planner_server 所有参数
ros2 param list /planner_server

# 查看 allow_unknown（是否允许在未知区域规划）
ros2 param get /planner_server allow_unknown

# 查看其他常用参数
ros2 param get /planner_server use_final_approach_orientation
ros2 param get /controller_server controller_frequency
```

修改参数（需节点支持动态重配置）：

```bash
ros2 param set /planner_server allow_unknown true
```

## 9. 交接建议

后续把 UI 交给 Gemini 重做时，优先保留这些不动：

- REST 路径
- WebSocket 消息结构
- 地图坐标换算规则
- `POST /api/v1/localization/initial-pose` 的入参格式

建议直接参考:

- `API_HANDOFF.md`
- `cyberdog_web_bridge/bridge_node.py`
- `cyberdog_web_bridge/static/app.js`
