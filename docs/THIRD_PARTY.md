# 第三方与外部依赖

本文件是公开展示版本的依赖清单，不代替正式的法律审查。原则是：能通过包管理器或官方仓库获得的第三方代码不复制；来源未确认的代码不公开。

| 组件 | 用途 | 公开仓库处理 | 状态 |
| --- | --- | --- | --- |
| ROS 2 / Nav2 | 通信、Action、导航 | 仅声明依赖 | 使用官方发行版 |
| Cartographer | 建图与定位 | 仅保留自定义配置 | 使用官方发行版 |
| TensorRT / CUDA | Jetson 推理 | 不包含 SDK、`.so` 或 Engine | NVIDIA 外部依赖 |
| [TensorRTx](https://github.com/wang-xinyu/tensorrtx) | YOLOv8 TensorRT 基础实现 | 不复制当前来源不明的修改副本 | 上游为 MIT，发布修改前仍需核对来源 |
| Ultralytics YOLOv8 | 模型训练/导出生态 | 不包含源码、权重和训练数据 | 使用者需自行核对对应版本许可 |
| PaddleOCR | 仪表 OCR | 不复制第三方源码和模型 | 使用官方依赖 |
| FastAPI / Uvicorn | Web API | Python 依赖声明 | 使用官方包 |
| OpenCV / Eigen / yaml-cpp | 图像、数学和配置 | 系统依赖声明 | 使用官方包 |
| CyberDog SDK / messages | 机器人控制和相机接口 | 不包含厂商 SDK；仅保留适配接口示例 | 发布前核对厂商许可 |
| Orbbec / YDLidar SDK | 深度相机和雷达 | 不包含 SDK 和二进制库 | 使用厂商安装方式 |
| MSTC-Star | 覆盖路径规划服务 | 当前不随公开版本分发 | 来源和许可证待确认 |
| gRPC / Protobuf 生成代码 | 巡检规划服务通信 | 仅保留 `.proto` 接口或本项目包装层 | 可重新生成 |
| `gif.h` by Charlie Tangora | 巡检路径调试 GIF | 保留单个小型头文件及原始声明 | 文件声明为 Public Domain |

## TensorRTx 处理方式

如果未来公开完整适配实现，建议按以下方式重新建立：

1. 记录 TensorRTx 上游 URL、基线提交和 MIT License。
2. 从上游干净副本开始，不以原公司目录作为发布源。
3. 将 CyberDog/ROS 2 适配放在独立包装层。
4. 对无法避免的上游修改保留版权和许可证，并在 `UPSTREAM.md` 中列出差异。
5. 不提交 `.pt`、`.wts`、`.onnx` 或 `.engine`。

## 仓库许可证

在自有代码的权属得到确认之前，不应给整个仓库直接套用 MIT/Apache 许可证。可以先以作品展示方式公开经过筛选的材料，并在仓库说明中声明“未明确授权的部分保留所有权利”。确认全部公开文件均为本人可授权内容后，再选择正式许可证。
