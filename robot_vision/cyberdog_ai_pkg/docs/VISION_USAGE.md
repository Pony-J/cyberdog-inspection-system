# CyberDog 视觉使用说明

## 1. 目的

这份文档说明当前 `cyberdog_ai_pkg` 视觉链路的启动方式、默认行为、真开关语义、巡检并跑方式、报警记录位置，以及常用的调试命令。

## 2. 当前链路概览

狗侧一键入口：

```bash
ros2 run cyberdog_ai_pkg start_vision_stack
```

当前链路如下：

1. `start_vision_stack.sh`
2. 启动基础设施：`vision_runtime_manager.py`、`vision_manager_node.py`、`vision_ws_gateway.py`
3. `vision_runtime_manager.py` 负责真实启停 detector 进程
4. `vision_manager_node.py` 做聚合、报警保存、标注图生成
5. `vision_ws_gateway.py` 把检测结果、状态、运行时状态、报警、标注图通过 WebSocket 转出去
6. Orin 侧 `cyberdog_web_bridge` 代理 HTTP API，并把状态提供给前端

这套链路和巡检控制本身是解耦的，因此可以：

1. 先手动启动视觉
2. 再手动控狗
3. 或者再启动巡检

在这期间视觉检测和报警记录仍然会继续工作。

## 3. 当前代码默认配置

当前 `start_vision_stack.sh` 的默认环境变量如下：

```bash
ENABLE_CAMERA=1
ENABLE_BODY_DETECTOR=1
ENABLE_METER_READING=0
ENABLE_FACE_DETECTOR=0
ENABLE_FIRE_ALARM=1
ENABLE_FALL_ALARM=1
ENABLE_HAT_ALARM=0
ENABLE_SMOKE_ALARM=0
ENABLE_GATHER_ALARM=0
ANNOTATED_FPS=5.0
BODY_PROCESS_EVERY_N_FRAMES=3
STOP_OLD_PROCESSES=1
```

也就是说，当前默认启动视觉栈时会申请拉起：

1. `person`
2. `fire_alarm`
3. `fall_alarm`

同时固定启动：

1. `vision_runtime_manager`
2. `vision_manager`
3. `vision_ws_gateway`

默认不申请拉起的是：

1. `hat_alarm`
2. `gather_alarm`
3. `smoke_alarm`
4. `meter_reading`
5. `face_detector`

补充说明：

1. `body_detector` 现在默认三帧取一帧，即 `BODY_PROCESS_EVERY_N_FRAMES=3`。
2. 如果某个 runtime detector 对应的 `.engine` 文件不存在，运行时管理器会把它标记为不可用，不会偷偷构建 TensorRT engine。
3. `meter_reading`、`face_detector`、`smoke_alarm` 目前仍属于“启动时决定”的范围，不在 v1 真开关里。

## 4. 真开关语义

### 4.1 当前哪些算法支持真开关

当前 v1 真开关只覆盖这 5 路 detector：

1. `person`
2. `fire_alarm`
3. `fall_alarm`
4. `hat_alarm`
5. `gather_alarm`

前端按钮和 `/api/v1/vision/detectors` 现在对应的是“真实运行中的 detector 集合”，不是单纯修改聚合层展示。

### 4.2 真开关现在是怎么工作的

`POST /api/v1/vision/detectors`

请求体格式保持不变：

```json
{
  "detectors": ["person", "fire_alarm"]
}
```

但语义已经升级成：

1. 这是一组“当前会话希望真正运行”的 detector 全量集合
2. 狗侧 `vision_runtime_manager` 会按这组目标真实启动或停止 detector 进程
3. `vision_manager` 会始终订阅已知 detector topic，因此后来新增打开的 detector 无需重启视觉栈
4. API 返回的 `desired`、`running`、`failed` 才是最终真值

### 4.3 当前不在真开关里的内容

下面这些暂时不属于 v1 真开关范围：

1. `smoke_alarm`
2. `meter_reading`
3. `face_detector`

这几项如果要启停，仍然取决于 `start_vision_stack` 启动时带的环境变量。

## 5. 推荐运行方式

### 5.1 当前默认方式

狗侧：

```bash
source /etc/mi/mi_config
cd ~/cyberdog_ws
source install/setup.bash
ros2 run cyberdog_ai_pkg start_vision_stack
```

### 5.2 巡检并跑的轻量方式

如果目标是巡检期间保留核心视觉能力，建议优先使用轻量组合，例如只保留 `body + fire`：

```bash
source /etc/mi/mi_config
cd ~/cyberdog_ws
source install/setup.bash
ENABLE_BODY_DETECTOR=1 ENABLE_FIRE_ALARM=1 ENABLE_FALL_ALARM=0 ENABLE_HAT_ALARM=0 ENABLE_SMOKE_ALARM=0 ANNOTATED_FPS=2 ros2 run cyberdog_ai_pkg start_vision_stack
```

如果还需要进一步降载，可以继续调大抽帧：

```bash
BODY_PROCESS_EVERY_N_FRAMES=4 ros2 run cyberdog_ai_pkg start_vision_stack
```

或：

```bash
BODY_PROCESS_EVERY_N_FRAMES=5 ros2 run cyberdog_ai_pkg start_vision_stack
```

## 6. 常用启停命令

### 6.1 启动视觉

```bash
source /etc/mi/mi_config
cd ~/cyberdog_ws
source install/setup.bash
ros2 run cyberdog_ai_pkg start_vision_stack
```

### 6.2 运行中真开关算法

如果 Orin 侧 bridge 已启动，可以直接走 HTTP：

```bash
curl -X POST http://127.0.0.1:8090/api/v1/vision/detectors \
  -H 'Content-Type: application/json' \
  -d '{"detectors":["person","fire_alarm"]}'
```

全部关闭：

```bash
curl -X POST http://127.0.0.1:8090/api/v1/vision/detectors \
  -H 'Content-Type: application/json' \
  -d '{"detectors":[]}'
```

如果只想在狗侧直接调内部服务：

```bash
ros2 service call /vision/runtime/set_detectors cyberdog_ai_pkg/srv/RuntimeSetDetectors "{detectors: ['person', 'fire_alarm']}"
```

### 6.3 查看当前运行时状态

Orin 侧：

```bash
curl http://127.0.0.1:8090/api/v1/live
```

狗侧：

```bash
ros2 topic echo /vision/runtime_status
```

其中重点看：

1. `available`
2. `desired`
3. `running`
4. `unavailable`
5. `failed`

### 6.4 停止视觉

理论上前台运行时可以 `Ctrl+C`。但如果停不干净，建议直接执行下面这组命令：

```bash
pkill -INT -f 'ros2 run cyberdog_ai_pkg start_vision_stack' || true
sleep 1
pkill -f 'vision_runtime_manager.py|body_detector_node|fire_detector_node|gather_detector_node|smoke_detector_node|meter_reading_node|face_detector_node|vision_manager_node.py|vision_ws_gateway.py|vision_manager.launch.py|vision_ws_gateway.launch.py|fall_detector|hat_detector' || true
fuser -k 9091/tcp || true
```

### 6.5 Orin 侧启动 bridge / 前端链路

```bash
cd ~/ros2_ws
source ~/.bashrc
ros2 launch cyberdog_web_bridge web_bridge.launch.py
```

Orin 上的一键入口：

```bash
~/start_vision_console.sh
```

注意：这个入口是在 Orin 上运行，不是在狗上运行。

## 7. 报警与输出

### 7.1 主要 ROS / WS 输出

常用 topic：

```bash
/mi1035085/body
/mi1035085/fire_alarm
/mi1035085/fall_alarm
/mi1035085/hat_alarm
/mi1035085/smoke_alarm
/mi1035085/gather_alarm
/vision/detections
/vision/status
/vision/runtime_status
/vision/annotated/compressed
/vision/alarm_ws
```

WebSocket 默认地址：

```bash
ws://<dog-ip>:9091/vision
```

其中会转发：

1. `detections`
2. `status`
3. `runtime`
4. `annotated`
5. `alarm`

### 7.2 报警保存位置

狗侧 `vision_manager_node.py` 会把报警落到：

```bash
~/cyberdog_ws/alarm_logs
```

保存内容包括：

1. 报警截图 `alarm_*.jpg`
2. 报警结构化结果 `alarm_*.json`

只有 `is_alarm=true` 的检测结果才会被记为报警。`body/person` 本身不是报警，只是目标检测来源。

## 8. 巡检并跑说明

当前链路不要求“巡检服务自己拉起视觉”。更推荐的方式是：

1. 手动先起视觉
2. 再控狗或启动巡检
3. 视觉持续识别
4. 有报警时照常记录并往前端发送

这意味着视觉和巡检可以同时运行，但是否“跑得稳”主要取决于资源占用，而不是链路是否互斥。

## 9. 性能注意事项

当前视觉负载较高时，优先关注下面几项：

1. `body_detector` 是 CPU 大头，虽然已加入抽帧，但仍然是主要负载来源之一。
2. `annotated` 实时画面会带来额外开销，包括画框、JPEG 编码、base64 和 WS 转发。
3. 巡检和视觉并跑时，不建议一开始追求高帧率直播；应优先保证“报警事件能出来”。
4. 如果只是做火警巡检，通常建议先用 `body + fire` 轻量组合。

## 10. 当前已知限制

1. v1 真开关只覆盖 `person / fire_alarm / fall_alarm / hat_alarm / gather_alarm`。
2. `smoke_alarm`、`meter_reading`、`face_detector` 目前不在真开关范围内。
3. `Ctrl+C` 在部分场景下可能停不干净，因此保留手动清理命令。
4. `gather_alarm` 使用与 person 检测相同的 YOLOv5 模型，通过几何聚类后处理判定人员聚集，并生成合并报警框。
5. 当前默认仍包含 `fall_alarm`；如果后续决定关闭默认摔倒报警，请同步更新本文档。

## 11. 后续维护建议

如果后续做下面这些改动，建议第一时间同步改这份文档：

1. 默认 detector 组合改成 `body + fire`
2. `fall_alarm` 默认关闭
3. 从默认链路中移除 `smoke_alarm`
4. 扩展真开关范围到 `meter_reading` 或 `face_detector`
5. 给视觉栈增加更轻量的巡检模式
