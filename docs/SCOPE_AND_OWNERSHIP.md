# 贡献与仓库边界

## 两个仓库的定位

| 仓库 | 展示重点 | 与本系统关系 |
| --- | --- | --- |
| `cyberdog-inspection-system` | Web—Orin—ROS 2—CyberDog 的系统集成、服务编排、导航接入、视觉接入和数据链路 | 机器狗巡检主项目 |
| [`QIanKin/MOCHA`](https://github.com/QIanKin/MOCHA) | 轨迹优化、同伦路径、倒车入库、动态障碍和多机器人实验 | 独立算法作品，不是主项目运行时依赖 |

不建议把 MOCHA 作为 Git submodule 接入主项目。直接在 README 中关联仓库和固定提交更清楚，也不会让面试官误以为 CyberDog 已经在线调用 MOCHA。

## 面试表述建议

可以明确表述：

- 设计并实现 Web 到 ROS 2 的任务控制和状态回传链路。
- 完成 CyberDog、Jetson Orin、Nav2、相机和视觉算法之间的接口适配。
- 完成建图/定位/导航服务的按需启动、巡检目标点执行、暂停恢复和结果展示。
- 基于开源推理框架完成 CyberDog 老版本 TensorRT 环境适配，并解决检测结果回调和运行时启停问题。
- 将视觉告警、传感器数据、机器人位姿和任务状态汇聚到 Web 端。

避免表述：

- “Nav2、Cartographer、YOLOv8、TensorRTx 均为自主研发”。
- “MOCHA 已经作为 CyberDog 在线规划器运行”——当前代码没有这条集成链路。
- “已经实现完整多源融合数据库”——当前实现主要是本地 JSON/JPG 留痕。
- “仓库可以无硬件一键运行”——模型、SDK 和现场设备依赖已经剥离。

## 代码公开分级

### 可以放入

- 自己确认编写的 Web bridge、任务编排和机器人适配代码。
- ROS 2 launch、消息/服务定义以及脱敏后的参数模板。
- Nav2 参数调整、TF/时间戳修正、速度桥接和巡检 Action 客户端。
- 自己确认编写的 TensorRT 7 兼容层、ROS 2 节点包装和结果转换代码。
- 架构图、接口说明、演示截图、测试流程和已知限制。

### 仅保留依赖说明

- 模型权重、ONNX、TensorRT Engine、Paddle 推理产物。
- CyberDog、相机、雷达等厂商 SDK 和二进制库。
- Nav2、Cartographer、TensorRTx、PaddleOCR 等可从官方来源获取的第三方实现。
- 许可证或来源尚未核清的 MSTC-Star、原公司视觉底层代码和生成代码。

### 不应公开

- 原公司私有仓库代码或无法确认作者/许可的修改版本。
- RTSP 用户名密码、签名 URL、内网地址、设备序列号和访问令牌。
- 训练数据、客户现场地图、告警截图或包含人员隐私的原始视频。
- `build/`、`install/`、`log/`、`.so`、SDK、Docker 导出包和缓存。

## YOLOv8Detector 归属

现有目录 README 说明其基础代码来自 `wang-xinyu/tensorrtx/yolov8`，上游采用 MIT License；本地提交记录中可以看到 TensorRT 7/8 兼容及回调修复工作。

但因为当前副本来自原公司环境，不能仅凭上游 MIT License 推定整个目录都可以公开。公开版本不包含该目录，只保留自有适配层和 [迁移说明](YOLOV8_TRT7_PORTING.md)。如果后续需要发布完整实现，应从明确的上游提交重新建立干净分支，再逐项重做或核对修改。

## MOCHA 发布前仍需补齐

MOCHA 当前仓库已经把 ROS、地图、结果和构建产物剥离，只保留算法核心与演示素材，这个方向是正确的；但正式作为面试仓库前还应补充：

- 根目录许可证或明确的版权声明。
- 可复现的 `CMakeLists.txt`、最小示例和至少一个测试。
- 算法输入/输出、代价函数、约束与实验指标说明。
- `third_party/lbfgs.hpp` 的来源、版本和许可证。
- 压缩或外链大型 GIF；当前最大素材约 24 MB，会明显拖慢克隆和 README 加载。
