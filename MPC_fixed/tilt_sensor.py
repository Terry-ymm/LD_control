# -*- coding: utf-8 -*-
"""倾角传感器读取模块。"""
from __future__ import annotations

import struct
import threading
import time
from datetime import datetime
from typing import Callable, Optional

import serial


class TiltSensorReader:
    def __init__(self, on_data: Optional[Callable[[dict], None]] = None, on_status: Optional[Callable[[str], None]] = None):
        self.port = None
        self.connected = False
        self.stop_receive = False
        self.receive_thread = None

        self.on_data = on_data
        self.on_status = on_status

        self.lock = threading.Lock()
        self.latest_angles = {"x_angle": 0.0, "y_angle": 0.0, "z_angle": 0.0}
        self.last_valid_angles = {"x_angle": 0.0, "y_angle": 0.0, "z_angle": 0.0}
        self.last_frame_time = 0.0

        self.rx_buffer = b""
        self.max_frames_per_cycle = 3
        self.backlog_drop_bytes = 2048
        self.local_buffer_keep = 64
        self.max_abs_x = 90.0
        self.max_abs_y = 90.0
        self.max_abs_z = 360.0
        self.jump_reject_deg = 1.0

        # 传感器安装方向修正：当前实验安装方向相反，解析后统一乘 -1。
        # 如果后续传感器按正常方向安装，把这里改成 1.0。
        self.angle_sign = -1.0

        self.data_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        self.fps = 0.0

    def _status(self, text: str):
        if self.on_status:
            self.on_status(text)

    def connect(self, port_name: str, baudrate: int):
        if not port_name:
            raise ValueError("倾角传感器端口为空")
        self.port = serial.Serial(port_name, int(baudrate), timeout=0.01)
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        self.rx_buffer = b""
        self.connected = True
        self.stop_receive = False
        self.data_count = 0
        self.last_fps_time = time.time()
        self.last_fps_count = 0
        self.fps = 0.0
        self.last_frame_time = 0.0
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

    def disconnect(self):
        self.stop_receive = True
        self.connected = False
        try:
            if self.port:
                self.port.close()
        finally:
            self.port = None

    def get_angles(self) -> dict:
        with self.lock:
            return dict(self.latest_angles)

    def _receive_loop(self):
        while not self.stop_receive and self.connected:
            try:
                if self.port and self.port.in_waiting:
                    waiting_now = self.port.in_waiting
                    if waiting_now > self.backlog_drop_bytes:
                        self.port.reset_input_buffer()
                        self.rx_buffer = b""
                        time.sleep(0.005)
                        continue

                    read_size = max(17, min(waiting_now, 256))
                    self.rx_buffer += self.port.read(read_size)
                    if len(self.rx_buffer) > self.local_buffer_keep:
                        self.rx_buffer = self.rx_buffer[-self.local_buffer_keep:]

                    frames_parsed = 0
                    last_valid = None
                    while len(self.rx_buffer) >= 17 and frames_parsed < self.max_frames_per_cycle:
                        idx = self.rx_buffer.find(b"\x50\x03\x0c")
                        if idx == -1:
                            self.rx_buffer = self.rx_buffer[-16:]
                            break
                        if idx > 0:
                            self.rx_buffer = self.rx_buffer[idx:]
                            continue

                        frame = self.rx_buffer[:17]
                        self.rx_buffer = self.rx_buffer[17:]
                        result, _ = self.parse_frame(frame)
                        if not result:
                            continue

                        if self.last_frame_time != 0.0:
                            dx = abs(result["x_angle"] - self.last_valid_angles["x_angle"])
                            dy = abs(result["y_angle"] - self.last_valid_angles["y_angle"])
                            dz = abs(result["z_angle"] - self.last_valid_angles["z_angle"])
                            if dx > self.jump_reject_deg or dy > self.jump_reject_deg or dz > self.jump_reject_deg:
                                continue

                        last_valid = result
                        frames_parsed += 1

                    if last_valid is not None:
                        self.data_count += 1
                        with self.lock:
                            self.latest_angles.update({
                                "x_angle": last_valid["x_angle"],
                                "y_angle": last_valid["y_angle"],
                                "z_angle": last_valid["z_angle"],
                            })
                        self.last_valid_angles = dict(self.latest_angles)
                        self.last_frame_time = time.time()
                        self._update_fps()
                        last_valid["fps"] = self.fps
                        last_valid["count"] = self.data_count
                        if self.on_data:
                            self.on_data(last_valid)
                else:
                    time.sleep(0.001)
            except Exception as exc:
                self.stop_receive = True
                self._status(f"传感器接收异常: {exc}")

    def _update_fps(self):
        now = time.time()
        dt = now - self.last_fps_time
        if dt >= 0.5:
            self.fps = (self.data_count - self.last_fps_count) / dt
            self.last_fps_time = now
            self.last_fps_count = self.data_count

    def parse_frame(self, data: bytes):
        if len(data) != 17 or data[0:3] != b"\x50\x03\x0c":
            return None, "HeadErr"

        def b2a(b):
            return struct.unpack(">i", b[2:4] + b[0:2])[0] / 1000.0

        try:
            x_angle = self.angle_sign * b2a(data[3:7])
            y_angle = self.angle_sign * b2a(data[7:11])
            z_angle = self.angle_sign * b2a(data[11:15])
        except Exception:
            return None, "DecodeErr"

        if abs(x_angle) > self.max_abs_x:
            return None, "RangeErrX"
        if abs(y_angle) > self.max_abs_y:
            return None, "RangeErrY"
        if abs(z_angle) > self.max_abs_z:
            return None, "RangeErrZ"

        return {
            "x_angle": x_angle,
            "y_angle": y_angle,
            "z_angle": z_angle,
            "timestamp": datetime.now(),
        }, "OK"
