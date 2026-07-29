# GuardDog-Q CyberDog Inspection System

GuardDog-Q 是一套基于 CyberDog、Jetson Orin 和 ROS 2 的多模态四足机器人自主巡检系统。系统通过 Web 端下发建图、导航、巡检和视觉检测任务，由 Orin 负责任务调度、机器人导航、算法启停、异常判断与巡检数据管理。

系统覆盖自主导航、多点巡检、端侧视觉识别、环境数据采集、本地告警和 Web 端事件追溯，可在走廊、实验室、设备区和仪表区等场景执行连续巡检任务。

## 核心功能

- **建图与定位：** Cartographer SLAM、地图保存、AMCL 定位和初始位姿设置。
- **自主导航：** Nav2 目标点导航、MSTC* 多点覆盖巡检、MOCHA 全局轨迹规划、MPPI 局部控制、动态避障及临时障碍重规划。
- **巡检调度：** 预设点位、任务启停、暂停恢复、到点检测和异常任务恢复。
- **视觉检测：** 火焰/烟雾、人员入侵、安全帽违规、人员聚集、人员摔倒、表盘检测与读数，支持 TensorRT 端侧加速并按巡检点位选择检测任务。
- **环境感知：** 烟雾/可燃气体、温湿度、光照和红外温度采集，并与视觉结果关联。
- **Web 控制：** 一键启动建图、导航、巡检和视觉服务，实时显示地图、路径、视频、机器人状态及检测结果。
- **告警追溯：** 本地声光提示、前端实时告警，以及截图、时间、点位、传感数据和处置结果记录查询。
- **硬件支撑：** STM32 多模态环境感知板、Type-C 通信与供电、OLED/蜂鸣器/状态灯及 Jetson Orin 升压供电模块。

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

## 项目结构

```text
ros2_packages/
├─ cyberdog_web_bridge/       Web 控制、任务下发与状态推送
├─ cyberdog_inspection/       巡检任务执行与 Nav2 接入
├─ cyberdog_nav2_lidar/       建图、定位、导航与速度桥接
└─ sensor_data/               环境传感器数据采集

robot_vision/cyberdog_ai_pkg/
├─ scripts/                   视觉服务管理、检测结果聚合与告警
├─ launch/                    ROS 2 启动文件
├─ msg/                       检测结果消息
└─ srv/                       检测器动态启停服务
```

## 未上传内容

- `*.onnx`、`*.engine`、`*.pt`、`*.wts` 和 Paddle 模型。
- CyberDog、相机、雷达 SDK 及动态库。

轨迹优化与运动规划项目：[QIanKin/MOCHA](https://github.com/QIanKin/MOCHA)
