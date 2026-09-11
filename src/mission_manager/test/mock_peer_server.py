#!/usr/bin/env python3
# --- How to run ---
# rostest mission_manager process_boundaries.test
"""Test-only ROS action/service and loopback WebSocket server process."""

from __future__ import annotations

import base64
import hashlib
import json
from json import JSONDecodeError
import socket
import threading
from typing import Dict, Mapping

import actionlib
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from mission_manager.msg import MissionAction, MissionResult


_WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class WebSocketProtocolError(Exception):
    """Raised when a test client violates the WebSocket wire contract."""


def _receive_exact(connection: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            raise WebSocketProtocolError("connection closed before frame completed")
        chunks.extend(chunk)
    return bytes(chunks)


def _receive_headers(connection: socket.socket) -> Mapping[str, str]:
    request = bytearray()
    while b"\r\n\r\n" not in request:
        chunk = connection.recv(1024)
        if not chunk or len(request) + len(chunk) > 8192:
            raise WebSocketProtocolError("invalid HTTP upgrade request")
        request.extend(chunk)
    lines = request.decode("ascii").split("\r\n")
    headers: Dict[str, str] = {}
    for line in lines[1:]:
        if ":" in line:
            name, value = line.split(":", 1)
            headers[name.strip().lower()] = value.strip()
    return headers


def _receive_text_frame(connection: socket.socket) -> str | None:
    first, second = _receive_exact(connection, 2)
    opcode = first & 0x0F
    if opcode == 0x08:
        return None
    if opcode != 0x01 or not second & 0x80:
        raise WebSocketProtocolError("expected a masked text frame")
    length = second & 0x7F
    if length == 126:
        length = int.from_bytes(_receive_exact(connection, 2), "big")
    elif length == 127:
        length = int.from_bytes(_receive_exact(connection, 8), "big")
    mask = _receive_exact(connection, 4)
    payload = _receive_exact(connection, length)
    return bytes(value ^ mask[index % 4] for index, value in enumerate(payload)).decode("utf-8")


def _send_text_frame(connection: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    length = len(payload)
    if length < 126:
        header = bytes((0x81, length))
    elif length < 65536:
        header = bytes((0x81, 126)) + length.to_bytes(2, "big")
    else:
        header = bytes((0x81, 127)) + length.to_bytes(8, "big")
    connection.sendall(header + payload)


class LoopbackWebSocketServer:
    """Mutable socket owner that accepts independent short-lived test clients."""

    def __init__(self, now_ns: int, freshness_limit_ns: int) -> None:
        self._now_ns = now_ns
        self._freshness_limit_ns = freshness_limit_ns
        self._stopped = threading.Event()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen()
        self._listener.settimeout(0.2)
        self.port = self._listener.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stopped.set()
        self._listener.close()
        self._thread.join(timeout=2.0)

    def _serve(self) -> None:
        while not self._stopped.is_set() and not rospy.is_shutdown():
            try:
                connection, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                connection.settimeout(2.0)
                try:
                    self._handle(connection)
                except (OSError, UnicodeDecodeError, WebSocketProtocolError):
                    continue

    def _handle(self, connection: socket.socket) -> None:
        headers = _receive_headers(connection)
        key = headers.get("sec-websocket-key")
        if not key:
            raise WebSocketProtocolError("missing Sec-WebSocket-Key")
        accept = base64.b64encode(hashlib.sha1((key + _WEBSOCKET_GUID).encode("ascii")).digest())
        connection.sendall(
            b"HTTP/1.1 101 Switching Protocols\r\n"
            b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
            + b"Sec-WebSocket-Accept: " + accept + b"\r\n\r\n"
        )
        text = _receive_text_frame(connection)
        if text is None:
            return
        try:
            request = json.loads(text)
        except JSONDecodeError:
            _send_text_frame(connection, '{"error":"malformed_input","ok":false}')
            return
        if not isinstance(request, dict):
            _send_text_frame(connection, '{"error":"malformed_input","ok":false}')
            return
        stamp_ns = request.get("stamp_ns")
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            _send_text_frame(connection, '{"error":"malformed_input","ok":false}')
            return
        age_ns = self._now_ns - stamp_ns
        if age_ns < 0 or age_ns > self._freshness_limit_ns:
            _send_text_frame(connection, '{"error":"stale_state","ok":false}')
            return
        _send_text_frame(connection, '{"ok":true,"response":"pong"}')


class RosMockPeers:
    """Owns the test-only ROS action and service endpoints."""

    def __init__(self) -> None:
        self._service = rospy.Service("/todo4/mock_change_map", Trigger, self._change_map)
        self._action = actionlib.SimpleActionServer(
            "/todo4/mock_mission",
            MissionAction,
            execute_cb=self._execute_mission,
            auto_start=False,
        )
        self._action.start()

    @staticmethod
    def _change_map(_request: Trigger.Request) -> TriggerResponse:
        return TriggerResponse(success=True, message="accepted")

    def _execute_mission(self, goal: MissionAction.Goal) -> None:
        result = MissionResult(
            result_code=MissionResult.OK,
            reason="",
            mission_id=goal.destination_id,
            artifact_path="",
        )
        self._action.set_succeeded(result)


def main() -> int:
    rospy.init_node("todo4_mock_peer_server")
    websocket_server = LoopbackWebSocketServer(
        now_ns=1_700_000_010_000_000_000,
        freshness_limit_ns=5_000_000_000,
    )
    websocket_server.start()
    rospy.on_shutdown(websocket_server.close)
    rospy.set_param("~websocket_port", websocket_server.port)
    RosMockPeers()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
