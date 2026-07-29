# CyberDog 视觉集成代码

这里保留狗端视觉链路中可用于说明个人工程工作的部分：

- `vision_runtime_manager.py`：按前端请求启动/停止检测进程，检查 Engine 可用性并汇总运行状态。
- `vision_manager_node.py`：聚合人员、仪表和扩展检测话题，生成标注帧、结构化检测和告警证据。
- `vision_ws_gateway.py`：把 ROS 2 检测结果和图像转换为 WebSocket 消息。
- `official_vision_wake_controller.py`：对接 CyberDog 官方视觉/相机唤醒流程。
- `meter_*`：仪表点位、对准和读取任务的业务适配。
- `msg/`、`srv/`：统一检测结果和动态检测器控制接口。

## 为什么没有底层检测器

模型、TensorRT Engine、Paddle 推理产物、CyberDog SDK 和第三方检测器均不适合直接进入展示仓库。现有 YOLOv8 底层副本虽然标注来源于 TensorRTx，但它经过原公司环境流转，修改代码的权属尚未完全核清，因此本公开版本不包含 `thirdparty/trt_yolov8`。

检测进程只需遵守本目录的消息接口，并由运行时管理器启动。底层可以替换成 TensorRTx、ONNX Runtime、厂商 SDK 或其他后端。

## 发布版本的构建状态

这是经过筛选的集成代码，不是完整 ROS 2 包：低层检测实现、厂商消息包和 CMake 目标被有意移除。要在真实设备构建，需要在私有部署仓库中补齐：

- `cyberdog_ai_pkg` 完整 `CMakeLists.txt` 和 `package.xml`
- CyberDog 官方消息与视觉 SDK
- 已确认许可的检测后端
- 设备本地生成的模型和 Engine

公开仓库强调运行时管理、消息契约和 Web 接入，不声称脱离这些依赖可独立推理。
