"""
Type-C 传感器数据服务

功能：
1. 从串口接收 Type-C 发送的数据帧
2. 按协议解包为结构化传感器数据
3. 通过 HTTP / SSE 提供给本机主控读取
4. 可选地按帧回调上传到上层主控
"""
import asyncio
import json
import logging
import queue
import threading
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import requests
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse

try:
    import serial
except ImportError:  # pragma: no cover - 依赖缺失时在运行期给出明确提示
    serial = None


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class ServerState:
    def __init__(self) -> None:
        self.config: Dict[str, Any] = {}
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.event_queue: asyncio.Queue = asyncio.Queue()
        self.latest_frame: Optional[Dict[str, Any]] = None
        self.frames_received = 0
        self.frames_dropped = 0
        self.last_frame_time: Optional[str] = None
        self.last_error: Optional[str] = None
        self.serial_connected = False
        self.last_upload_status: Optional[Dict[str, Any]] = None
        self.upload_configured = False
        self.upload_queue_size = 0
        self.reader: Optional["TypeCSerialReader"] = None


state = ServerState()


class TypeCSerialReader:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._upload_thread: Optional[threading.Thread] = None
        self._upload_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue(
            maxsize=int(config.get("upload", {}).get("queue_size", 100))
        )
        self._sequence = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._upload_thread = threading.Thread(
            target=self._upload_worker,
            name="typec-upload-worker",
            daemon=True,
        )
        self._thread = threading.Thread(target=self._run, name="typec-serial-reader", daemon=True)
        self._upload_thread.start()
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._upload_thread and self._upload_thread.is_alive():
            self._upload_thread.join(timeout=2.0)

    def _run(self) -> None:
        if serial is None:
            state.last_error = "pyserial 未安装，无法打开串口"
            logger.error(state.last_error)
            return

        serial_cfg = self.config["serial"]
        protocol_cfg = self.config["protocol"]
        reconnect_interval = float(serial_cfg.get("reconnect_interval_sec", 2.0))
        port = serial_cfg["port"]

        while not self._stop_event.is_set():
            ser = None
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=int(serial_cfg.get("baud_rate", 115200)),
                    bytesize=int(serial_cfg.get("data_bits", 8)),
                    stopbits=float(serial_cfg.get("stop_bits", 1)),
                    parity=serial.PARITY_NONE,
                    timeout=float(serial_cfg.get("read_timeout_sec", 1.0)),
                )
                state.serial_connected = True
                state.last_error = None
                logger.info("Serial port opened: %s", port)

                header = protocol_cfg.get("header", "[DATA:")
                tail = protocol_cfg.get("tail", "]")
                line_delimiter = protocol_cfg.get("line_delimiter", "\n")
                buffer = ""

                while not self._stop_event.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue

                    buffer += chunk.decode("utf-8", errors="ignore")
                    while line_delimiter in buffer:
                        raw_line, buffer = buffer.split(line_delimiter, 1)
                        raw_line = raw_line.rstrip("\r")
                        if not raw_line:
                            continue

                        parsed_frame = self._parse_frame(raw_line, header, tail)
                        if parsed_frame is None:
                            state.frames_dropped += 1
                            continue

                        self._sequence += 1
                        event = {
                            "event": "sensor_frame",
                            "seq": self._sequence,
                            "timestamp": datetime.now().isoformat(),
                            "raw_frame": raw_line,
                            "data": parsed_frame,
                        }
                        self._handle_frame(event)
            except Exception as exc:
                state.serial_connected = False
                state.last_error = str(exc)
                logger.error("Serial reader error: %s", exc)
                time.sleep(reconnect_interval)
            finally:
                state.serial_connected = False
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _parse_frame(self, frame: str, header: str, tail: str) -> Optional[Dict[str, Any]]:
        if not frame.startswith(header) or not frame.endswith(tail):
            logger.warning("Drop invalid frame: %s", frame)
            return None

        body = frame[len(header):]
        if tail:
            body = body[: -len(tail)]

        raw_fields: Dict[str, str] = {}
        parsed_fields: Dict[str, Any] = {}

        for item in body.split(","):
            part = item.strip()
            if not part:
                continue
            if "=" not in part:
                logger.warning("Drop malformed field: %s", part)
                return None

            key, value = part.split("=", 1)
            key = key.strip()
            value = value.strip()
            raw_fields[key] = value
            parsed_fields[self._map_field_name(key)] = self._convert_value(value)

        if not parsed_fields:
            logger.warning("Drop empty frame: %s", frame)
            return None

        return {
            "temperature_c": parsed_fields.get("temperature_c"),
            "humidity_pct": parsed_fields.get("humidity_pct"),
            "light_lux": parsed_fields.get("light_lux"),
            "sound_level": parsed_fields.get("sound_level"),
            "infrared_c": parsed_fields.get("infrared_c"),
            "raw_fields": raw_fields,
        }

    @staticmethod
    def _map_field_name(key: str) -> str:
        mapping = {
            "T": "temperature_c",
            "H": "humidity_pct",
            "L": "light_lux",
            "S": "sound_level",
            "IR": "infrared_c",
        }
        return mapping.get(key, key.lower())

    @staticmethod
    def _convert_value(value: str) -> Any:
        try:
            if "." in value:
                return float(value)
            return int(value)
        except ValueError:
            return value

    def _handle_frame(self, event: Dict[str, Any]) -> None:
        state.latest_frame = event
        state.frames_received += 1
        state.last_frame_time = event["timestamp"]

        if state.loop is not None:
            state.loop.call_soon_threadsafe(state.event_queue.put_nowait, event)

        try:
            self._upload_queue.put_nowait(event)
            state.upload_queue_size = self._upload_queue.qsize()
        except queue.Full:
            state.frames_dropped += 1
            state.last_upload_status = {
                "timestamp": datetime.now().isoformat(),
                "ok": False,
                "error": "upload queue full",
            }
            logger.error("Upload queue full, drop frame seq=%s", event.get("seq"))

    def _upload_worker(self) -> None:
        upload_cfg = state.config.get("upload", {})
        callback_url = upload_cfg.get("callback_url", "").strip()
        retry_interval = float(upload_cfg.get("retry_interval_sec", 1.0))

        state.upload_configured = bool(callback_url)

        while not self._stop_event.is_set():
            try:
                event = self._upload_queue.get(timeout=0.5)
            except queue.Empty:
                state.upload_queue_size = self._upload_queue.qsize()
                continue

            state.upload_queue_size = self._upload_queue.qsize()

            if not callback_url:
                continue

            self._upload_to_controller(callback_url, event)

            if not state.last_upload_status or not state.last_upload_status.get("ok", False):
                time.sleep(retry_interval)

    def _upload_to_controller(self, callback_url: str, event: Dict[str, Any]) -> None:
        upload_cfg = state.config.get("upload", {})
        timeout = float(upload_cfg.get("timeout_sec", 5))
        try:
            response = requests.post(callback_url, json=event, timeout=timeout)
            state.last_upload_status = {
                "timestamp": datetime.now().isoformat(),
                "ok": response.ok,
                "status_code": response.status_code,
                "callback_url": callback_url,
            }
            logger.info("Callback POST %s: %s", callback_url, response.status_code)
        except Exception as exc:
            state.last_upload_status = {
                "timestamp": datetime.now().isoformat(),
                "ok": False,
                "callback_url": callback_url,
                "error": str(exc),
            }
            logger.error("Callback failed: %s", exc)


@asynccontextmanager
async def lifespan(app: FastAPI):
    config_dir = Path(__file__).parent
    config_path = config_dir / "config.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        state.config = yaml.safe_load(f)

    state.config["_config_dir"] = str(config_dir)
    state.loop = asyncio.get_running_loop()
    state.upload_configured = bool(state.config.get("upload", {}).get("callback_url", "").strip())
    state.reader = TypeCSerialReader(state.config)
    state.reader.start()

    logger.info("Type-C sensor server started")
    yield

    if state.reader is not None:
        state.reader.stop()
    logger.info("Type-C sensor server stopped")


app = FastAPI(
    title="CyberDog Type-C Sensor Server",
    description="接收 Type-C 串口传感器数据并上传主控",
    version="1.0.0",
    lifespan=lifespan,
)


@app.get("/health")
async def health_check() -> Dict[str, Any]:
    return {
        "status": "ok" if state.last_error is None else "degraded",
        "serial_connected": state.serial_connected,
        "upload_configured": state.upload_configured,
        "last_error": state.last_error,
        "timestamp": datetime.now().isoformat(),
    }


@app.get("/status")
async def get_status() -> Dict[str, Any]:
    return {
        "serial_port": state.config.get("serial", {}).get("port"),
        "baud_rate": state.config.get("serial", {}).get("baud_rate"),
        "serial_connected": state.serial_connected,
        "upload_configured": state.upload_configured,
        "upload_queue_size": state.upload_queue_size,
        "frames_received": state.frames_received,
        "frames_dropped": state.frames_dropped,
        "last_frame_time": state.last_frame_time,
        "last_error": state.last_error,
        "last_upload_status": state.last_upload_status,
        "timestamp": datetime.now().isoformat(),
    }


@app.get("/latest")
async def latest_frame() -> Dict[str, Any]:
    if state.latest_frame is None:
        raise HTTPException(status_code=404, detail="No sensor frame received yet")
    return state.latest_frame


@app.get("/stream")
async def sensor_stream():
    async def event_generator():
        while True:
            try:
                event = await asyncio.wait_for(state.event_queue.get(), timeout=30)
                yield f"data: {json.dumps(event, ensure_ascii=False)}\n\n"
            except asyncio.TimeoutError:
                yield ": heartbeat\n\n"
            except Exception as exc:
                logger.error("SSE error: %s", exc)
                break

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
    )


def main() -> None:
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    server_cfg = config.get("server", {})
    uvicorn.run(
        app,
        host=server_cfg.get("host", "0.0.0.0"),
        port=int(server_cfg.get("port", 8091)),
        log_level=str(config.get("logging", {}).get("level", "info")).lower(),
    )


if __name__ == "__main__":
    main()
