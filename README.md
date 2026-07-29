# CyberDog Inspection System

CyberDog + Jetson Orin + ROS 2 的巡检项目源码整理版。仓库保留 Web 控制、任务调度、Nav2 接入、巡检执行、视觉进程管理和环境传感器接入代码；模型、TensorRT Engine、厂商 SDK、地图和来源不明确的第三方源码不上传。

## 实机演示

<table>
  <tr>
    <td align="center"><b>自主到达预设点位并读取表盘读数</b><br><img src="media/demos/preset-point-meter-reading.gif" alt="自主到达预设点位并读取表盘读数" width="100%"></td>
    <td align="center"><b>SLAM 建图、前端一键初始化与巡检任务执行</b><br><img src="media/demos/slam-and-patrol-task.gif" alt="SLAM 建图、前端一键初始化与巡检任务执行" width="100%"></td>
  </tr>
  <tr>
    <td align="center" colspan="2"><b>实时避障功能演示</b><br><img src="media/demos/realtime-obstacle-avoidance.gif" alt="实时避障功能演示" width="50%"></td>
  </tr>
</table>

## Web 端识别与告警

<p align="center">
  <b>人员识别与巡检地图联动界面</b><br>
  <img src="media/screenshots/person-detection-map-linkage.png" alt="人员识别与巡检地图联动界面" width="100%">
</p>

<p align="center">
  <b>人员识别事件与前端告警记录</b><br>
  <img src="media/screenshots/person-alarm-record.jpg" alt="人员识别事件与前端告警记录" width="100%">
</p>

<table>
  <tr>
    <td align="center"><b>表盘检测与读数结果</b><br><img src="media/screenshots/meter-detection-result.jpg" alt="表盘检测与读数结果" width="100%"></td>
    <td align="center"><b>地图、视频与巡检控制界面</b><br><img src="media/screenshots/web-map-video-control.jpg" alt="地图、视频与巡检控制界面" width="100%"></td>
  </tr>
</table>

## 代码目录

```text
ros2_packages/
├─ cyberdog_web_bridge/       Web API、WebSocket、服务启停和机器人控制
├─ cyberdog_inspection/       巡检路径接口、Nav2 Action 执行、暂停/恢复
├─ cyberdog_nav2_lidar/       Cartographer、AMCL、Nav2 配置和速度桥接
└─ sensor_data/               串口环境传感器采集与 HTTP 回调

robot_vision/cyberdog_ai_pkg/
├─ scripts/                   视觉进程管理、结果聚合、WebSocket 网关
├─ launch/                    视觉服务启动文件
├─ msg/                       统一检测结果消息
└─ srv/                       动态启停检测器服务
```

## 主要代码入口

| 功能 | 文件 |
| --- | --- |
| Web 服务、REST/WS、任务与进程调度 | [`bridge_node.py`](ros2_packages/cyberdog_web_bridge/cyberdog_web_bridge/bridge_node.py) |
| 前端控制逻辑 | [`app.js`](ros2_packages/cyberdog_web_bridge/cyberdog_web_bridge/static/app.js) |
| Nav2/Cartographer/AMCL 启动 | [`bringup.launch.py`](ros2_packages/cyberdog_nav2_lidar/launch/bringup.launch.py) |
| Nav2 参数与动态避障配置 | [`nav2_params.yaml`](ros2_packages/cyberdog_nav2_lidar/config/nav2_params.yaml) |
| CyberDog 速度指令桥接 | [`cmd_vel_bridge.py`](ros2_packages/cyberdog_nav2_lidar/scripts/cmd_vel_bridge.py) |
| 时间戳与 TF 数据修正 | [`fix_cyberdog_timestamps.py`](ros2_packages/cyberdog_nav2_lidar/scripts/fix_cyberdog_timestamps.py) |
| 巡检任务实现 | [`local_inspection.cpp`](ros2_packages/cyberdog_inspection/src/local_inspection.cpp) |
| Nav2 Action 客户端 | [`nav2_client.cpp`](ros2_packages/cyberdog_inspection/src/nav2_client.cpp) |
| 视觉算法进程按需启停 | [`vision_runtime_manager.py`](robot_vision/cyberdog_ai_pkg/scripts/vision_runtime_manager.py) |
| 多检测器结果聚合与告警保存 | [`vision_manager_node.py`](robot_vision/cyberdog_ai_pkg/scripts/vision_manager_node.py) |
| 狗端视觉 WebSocket 网关 | [`vision_ws_gateway.py`](robot_vision/cyberdog_ai_pkg/scripts/vision_ws_gateway.py) |
| 环境传感器服务 | [`typec_sensor_server.py`](ros2_packages/sensor_data/typec_sensor_server.py) |

## 已包含的接口

Web bridge 提供的主要接口包括：

- 建图、地图保存、初始位姿和导航目标下发。
- Nav2、巡检服务和视觉检测器启动/停止。
- CyberDog 模式、步态、速度、急停和相机控制。
- 巡检初始化、开始、暂停、恢复和停止。
- 地图、规划路径、机器人状态和传感器数据实时推送。
- 仪表点位、读数历史、视觉告警和截图查询。

接口实现集中在 `bridge_node.py` 的 `_create_app()`。

## 视觉代码边界

现有 YOLOv8 检测底层基于 TensorRTx，并针对 CyberDog 的旧 TensorRT 7/AArch64 环境做过适配。但当前副本来自原公司环境，无法确认所有中间修改的权属，因此公开目录不包含 `thirdparty/trt_yolov8`。

仓库保留自己编写的 ROS 2 运行时管理、检测结果转换、告警聚合和 Web 接入代码。具体适配内容记录在 [`YOLOV8_TRT7_PORTING.md`](docs/YOLOV8_TRT7_PORTING.md)。

## 导航代码边界

当前机器狗巡检运行时使用 Cartographer/AMCL、Nav2、Smac Hybrid-A*、MPPI 和巡检包的路径生成接口。

[QIanKin/MOCHA](https://github.com/QIanKin/MOCHA) 是独立的轨迹优化/运动规划项目，不是本仓库的运行时依赖，不复制进来，也不使用 Git submodule。README 核对时对应提交为 [`e992abd`](https://github.com/QIanKin/MOCHA/tree/e992abded69f5fe68e679acaaf7cb73bb354f4c4)。

## 未上传内容

- `*.onnx`、`*.engine`、`*.pt`、`*.wts` 和 Paddle 模型。
- CyberDog、相机、雷达 SDK 及动态库。
- `build/`、`install/`、`log/`、ROS bag 和生成地图。
- RTSP 密码、签名 URL、真实设备 IP 和现场配置。
- 来源或许可证尚未确认的第三方实现。

第三方依赖和代码权属说明见 [`THIRD_PARTY.md`](docs/THIRD_PARTY.md) 与 [`SCOPE_AND_OWNERSHIP.md`](docs/SCOPE_AND_OWNERSHIP.md)。

## 检查

```powershell
powershell -ExecutionPolicy Bypass -File ./scripts/check_public_release.ps1
```

脚本检查模型/二进制产物、大文件、私钥、带密码的 RTSP 地址和常见令牌。当前公开版本是依赖真实 CyberDog/Jetson 环境的源码展示，不包含完整硬件运行镜像。
