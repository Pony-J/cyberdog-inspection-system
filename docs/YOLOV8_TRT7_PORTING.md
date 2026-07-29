# YOLOv8 在 CyberDog 旧版 Jetson 环境中的适配

## 背景

现有检测器以 TensorRTx/YOLOv8 的 TensorRT C++ 实现为基础。CyberDog 使用的旧系统镜像、TensorRT 7 API 和 AArch64 环境与较新的 TensorRT 8 示例存在差异，因此不能直接编译运行。

本公开版本不包含来自原公司环境的底层检测器副本，只记录可以公开说明的工程适配思路和接口边界。

## 适配内容

### TensorRT 版本兼容

- 隔离 TensorRT 7/8 的反序列化、绑定查询和执行上下文差异。
- 对显式/隐式 batch、输入输出 binding 和 workspace 配置进行版本分支处理。
- 将 Engine 的加载、失败回退和资源释放放入明确生命周期。

### Jetson/AArch64

- 使用 Jetson 对应 CUDA 架构和本机 TensorRT 头文件/库。
- 避免把 x86 构建产物、绝对库路径或其他设备生成的 Engine 写入仓库。
- Engine 在目标设备根据明确的模型版本生成，模型与 Engine 分开管理。

### ROS 2 图像链路

```text
CyberDog camera topic
  -> image conversion / resize / normalize
  -> TensorRT inference
  -> NMS and coordinate restore
  -> structured Detection messages
  -> runtime manager
  -> WebSocket / alarm evidence
```

- 统一输入图像编码和时间戳。
- 将推理线程与 ROS 2 回调解耦，避免阻塞相机订阅。
- 对检测框、置信度、类别、原图尺寸和帧时间进行结构化回传。
- 修复“推理成功但结果未传到上层回调/标注图像”的链路问题。

### 动态启停

- Web 端只提交检测器逻辑名称，不接触模型绝对路径。
- 运行时管理器维护检测器状态、可用性和最近错误。
- 模型不存在或 Engine 不兼容时返回 `unavailable`，而不是让整个巡检服务崩溃。

## 建议的公开代码结构

```text
vision_adapter/
├─ legacy_tensorrt_compat.hpp/.cpp   # 自有版本兼容层
├─ cyberdog_yolov8_node.cpp          # ROS 2 包装
├─ detection_result_adapter.cpp      # 结构化结果转换
├─ detector_backend.hpp              # 与底层实现解耦的接口
└─ config/detectors.example.yaml
```

TensorRTx 作为外部依赖或可替换后端存在；如果没有把握确认原公司副本的权属，就不要上传 `thirdparty/trt_yolov8`，也不要发布由该副本直接产生的大段补丁。

## 面试中的一句话版本

> 基于 TensorRTx/YOLOv8 推理实现，我完成了 CyberDog 旧版 TensorRT 7 与 AArch64 环境适配，并把推理结果接入 ROS 2 运行时管理和 Web 告警链路；重点工作是版本兼容、生命周期管理和端到端数据回传，而不是重新发明 YOLO 网络。
