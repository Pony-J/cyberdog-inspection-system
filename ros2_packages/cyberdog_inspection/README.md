# cyberdog_inspection

`cyberdog_inspection` 是单机版巡检服务，保留 MSTC 巡检路径规划，使用本机 HTTP 接口触发流程，并通过 ROS 2 `NavigateToPose` 对接 `cyberdog_nav2_lidar` 的 Nav2 栈。

> 公开展示版本不包含来源和许可证尚待核对的 MSTC-Star Python 实现。仓库保留了本项目的 C++ 包装、gRPC 接口、Nav2 执行器和 mock 测试服务；运行完整路径规划前，请从已获授权的来源提供 MSTC 服务实现。

## 1. 配置

主配置文件：

`src/cyberdog_inspection/config/inspection_config.yaml`

关键参数：

- `ns`: 当前机器人的命名空间标识
- `maps_directory`: 默认找地图的目录
- `mstc.mode`: 走廊巡检使用 `"path"`
- `navigation_enabled`: 是否真的向 Nav2 发目标
- `visualization.enabled`: 调试开关
- `nav2.robot_frame`: 当前应与 `cyberdog_nav2_lidar` 的清洗后 TF 一致，使用 `base_footprint_fixed`

调试开关说明：

- `visualization.enabled: true`
  - 初始化后生成巡检路径 GIF
  - 保存地图处理中间图
- `visualization.enabled: false`
  - 不生成 GIF
  - 不保存中间图

当前调试输出目录：

- GIF: `src/cyberdog_inspection/test_output/gifs`
- 中间图: `src/cyberdog_inspection/test_output/maps`

## 2. 编译

在工作区根目录执行：

```bash
cd ~/ros2_ws
colcon build --packages-select cyberdog_inspection
source install/setup.bash
```

## 3. 真机完整启动

先启动你的 Nav2 栈：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_nav2_lidar bringup.launch.py
```

注意：

- 当前 `cyberdog_nav2_lidar` 已启用 Jetson 侧时间戳清洗层
- 巡检服务应读取清洗后的 TF 树
- 因此 `inspection_config.yaml` 中的 `nav2.robot_frame` 应保持为 `base_footprint_fixed`

再启动巡检服务：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_inspection cyberdog_inspection.launch.py \
  config:=$PWD/src/cyberdog_inspection/config/inspection_config.yaml
```

## 4. 本地测试启动

不依赖真实 Nav2 时，可用测试栈：

- 自动发 `map -> base_footprint` 原点 TF
- 自动发 `map -> base_footprint_fixed` 原点 TF
- 自动启动 mock `NavigateToPose` action server

启动命令：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch cyberdog_inspection cyberdog_inspection_test.launch.py \
  config:=$PWD/src/cyberdog_inspection/config/inspection_config.yaml
```

## 5. HTTP 使用流程

服务默认监听：

`http://127.0.0.1:8083`

说明：

- 这个服务仍然只建议本机访问
- 网页访问应通过 `cyberdog_web_bridge` 转发，不要让浏览器直接调这里

### 5.1 初始化

默认使用 `map` 场景：

```bash
curl -s -X POST http://127.0.0.1:8083/inspection/internal/start_initialization \
  -H 'Content-Type: application/json' \
  -d '{"scene_name":"map"}'
```

如果省略 `scene_name`，服务内部也会默认用 `map`。

### 5.2 查看状态

```bash
curl -s http://127.0.0.1:8083/inspection/internal/status
```

初始化成功后，期望看到：

- `inspection_status_name: READY`

### 5.2.1 查看地图列表

用于 Web bridge 构建场景下拉框和地图元数据：

```bash
curl -s http://127.0.0.1:8083/inspection/internal/maps
```

返回内容包含：

- `scene_name`
- `yaml_path`
- `pgm_path`
- `resolution`
- `origin`
- `width`
- `height`
- `available`

### 5.2.2 查看当前规划路径

```bash
curl -s http://127.0.0.1:8083/inspection/internal/planned_path
```

返回内容包含：

- `active_scene_name`
- `planned_path`

其中 `planned_path` 为世界坐标系下的二维点序列：

```json
[[x1, y1], [x2, y2], [x3, y3]]
```

### 5.2.3 查看当前调试产物

```bash
curl -s http://127.0.0.1:8083/inspection/internal/map_artifacts
```

返回内容包含：

- `scene_name`
- `latest_gif_path`
- `debug_map_paths`

### 5.3 开始巡检

```bash
curl -s -X POST http://127.0.0.1:8083/inspection/internal/start_inspection
```

### 5.4 暂停巡检

```bash
curl -s -X POST http://127.0.0.1:8083/inspection/internal/pause_inspection
```

说明：

- 当前“暂停”语义是取消当前 Nav2 goal
- 巡检进度会保留
- 后续可通过 `resume_inspection` 继续

### 5.5 恢复巡检

```bash
curl -s -X POST http://127.0.0.1:8083/inspection/internal/resume_inspection
```

### 5.6 停止/重置

```bash
curl -s -X POST http://127.0.0.1:8083/inspection/internal/stop_inspection
curl -s -X POST http://127.0.0.1:8083/inspection/internal/reset
```

## 6. 调试产物查看

当 `visualization.enabled: true` 时，初始化完成后会生成：

- 巡检路线 GIF
- `00_original_map.png`
- `01_filtered_map.png`
- `02_path_skeleton.png`
- `03_path_skeleton_with_robot.png`
- `04_path_graph.png`

对于狭窄走廊巡检，建议重点对比：

- `00_original_map.png`
- `01_filtered_map.png`

如果 `01_filtered_map.png` 里走廊明显被侵蚀变窄，优先检查：

- `mstc.robot_size`
- `mstc.min_obs_radius`

这些产物现在也会通过内部 HTTP 接口返回绝对路径，供 `cyberdog_web_bridge` 做调试展示和前端交接。

## 7. 与网页桥接层的关系

推荐部署方式：

```text
Browser -> cyberdog_web_bridge -> cyberdog_inspection
```

职责边界：

- `cyberdog_inspection`
  - 只负责巡检初始化、路径规划、巡检控制、状态查询
- `cyberdog_web_bridge`
  - 负责浏览器访问、地图图片输出、AMCL 初始位姿接口、WebSocket 实时推送

## 8. 当前推荐配置

当前场景推荐：

- `mstc.mode: "path"`

原因：

- 这是狭窄走廊巡检，更适合 skeleton/path 模式
- `coverage` 更适合开阔区域覆盖，不适合作为当前默认模式
