# sensordata

Type-C 传感器数据服务，负责从串口接收如下格式的数据帧，解包后提供给本地主控，并可选地回调上传：

```text
[DATA:T=25.5,H=60.0,L=120,S=450,IR=33.2]\r\n
```

协议参数：

- 波特率：`115200`
- 数据位：`8`
- 停止位：`1`
- 帧头：`[DATA:`
- 帧尾：`]`

## 启动

```bash
cd /root/cyberdog_ws/sensordata
pip3 install -r requirements.txt
python3 typec_sensor_server.py
```

或：

```bash
cd /root/cyberdog_ws/sensordata
./start.sh
```

## 配置

编辑 `config.yaml`：

- `serial.port`：Type-C 枚举出来的串口设备，例如 `/dev/ttyACM0`
- `upload.callback_url`：主控接收地址，留空则只提供本地 HTTP / SSE 接口
- `upload.retry_interval_sec`：上传失败后的重试间隔
- `upload.queue_size`：本地上传队列长度

## 接口

- `GET /health`：健康状态
- `GET /status`：串口状态、接收计数、最近上传结果
- `GET /latest`：最近一帧解析结果
- `GET /stream`：SSE 实时推送

## 输出字段

每收到一帧，服务会：

- 始终通过 `GET /stream` 提供 SSE 事件流
- 如果配置了 `upload.callback_url`，则额外发起一次 HTTP POST(JSON)

```http
POST <callback_url>
Content-Type: application/json
```

示例帧：

```json
{
  "event": "sensor_frame",
  "seq": 1,
  "timestamp": "2026-03-27T12:00:00",
  "raw_frame": "[DATA:T=25.5,H=60.0,L=120,S=450,IR=33.2]",
  "data": {
    "temperature_c": 25.5,
    "humidity_pct": 60.0,
    "light_lux": 120,
    "sound_level": 450,
    "infrared_c": 33.2,
    "raw_fields": {
      "T": "25.5",
      "H": "60.0",
      "L": "120",
      "S": "450",
      "IR": "33.2"
    }
  }
}
```
