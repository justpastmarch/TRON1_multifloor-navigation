from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_robot_transport_fakes import (
    FakeClock,
    FakeWebSocket,
    make_transport,
    successful_mode_handler,
)


class RobotTransportTest(unittest.TestCase):
    def test_start_verifies_stand_then_walk_status_before_motion(self) -> None:
        # Given: a robot that acknowledges and reports each requested mode.
        clock = FakeClock()
        socket = FakeWebSocket(successful_mode_handler)
        transport, _ = make_transport(socket, clock)

        # When: the transport starts and receives a physical command.
        transport.start()
        transport.update_twist(linear_mps=0.25, angular_radps=-1.0)
        transport.send_current()

        # Then: STAND and WALK precede the bounded normalized twist.
        titles = [payload["title"] for payload in socket.sent]
        self.assertEqual(
            titles,
            [
                "request_twist",
                "request_twist",
                "request_stand_mode",
                "request_walk_mode",
                "request_twist",
            ],
        )
        self.assertEqual(socket.sent[-1]["data"], {"x": 0.5, "y": 0.0, "z": -0.5})
        self.assertEqual(socket.receive_timeouts, [0.01])

    def test_start_accepts_observed_firmware_frame_order(self) -> None:
        # Given: STAND notifies before its response; WALK responds before its status.
        clock = FakeClock()

        def observed_order(request, socket: FakeWebSocket) -> None:
            title = request["title"]
            if title == "request_twist":
                return
            response = {
                "accid": request["accid"],
                "title": title.replace("request_", "response_", 1),
                "timestamp": request["timestamp"],
                "guid": request["guid"],
                "data": {"result": "success"},
            }
            status = {
                "accid": request["accid"],
                "title": "notify_robot_info",
                "timestamp": request["timestamp"],
                "guid": "status-guid",
                "data": {"status": "WALK"},
            }
            if title == "request_stand_mode":
                socket.queue(status)
                socket.queue(response)
            else:
                socket.queue(response)
                socket.queue(status)

        transport, _ = make_transport(FakeWebSocket(observed_order), clock)

        # When/Then: the observed safe handshake reaches transport readiness.
        transport.start()

    def test_robot_clock_offset_uses_response_order_for_status_freshness(self) -> None:
        # Given: a robot clock behind the client that emits fractional milliseconds.
        clock = FakeClock()
        robot_timestamp = 1_787_286_632_000.25
        statuses = {
            "request_stand_mode": "STAND",
            "request_walk_mode": "WALK",
        }

        def offset_clock_handler(request, socket: FakeWebSocket) -> None:
            title = request["title"]
            if title == "request_twist":
                return
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": title.replace("request_", "response_", 1),
                    "timestamp": robot_timestamp,
                    "guid": request["guid"],
                    "data": {"result": "success"},
                }
            )
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": "notify_robot_info",
                    "timestamp": robot_timestamp + 1.0,
                    "guid": "status-guid",
                    "data": {"status": statuses[title]},
                }
            )

        transport, _ = make_transport(FakeWebSocket(offset_clock_handler), clock)

        # When/Then: same-clock response ordering permits verified startup.
        transport.start()

    def test_stand_acknowledgement_can_keep_reporting_walk(self) -> None:
        # Given: deployed firmware acknowledges stand mode but reports WALK throughout.
        clock = FakeClock()

        def walk_status_handler(request, socket: FakeWebSocket) -> None:
            title = request["title"]
            if title == "request_twist":
                return
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": title.replace("request_", "response_", 1),
                    "timestamp": request["timestamp"],
                    "guid": request["guid"],
                    "data": {"result": "success"},
                }
            )
            socket.queue(
                {
                    "accid": request["accid"],
                    "title": "notify_robot_info",
                    "timestamp": request["timestamp"],
                    "guid": "status-guid",
                    "data": {"status": "WALK"},
                }
            )

        transport, _ = make_transport(FakeWebSocket(walk_status_handler), clock)

        # When/Then: correlated stand acceptance still proceeds to verified WALK.
        transport.start()

    def test_watchdog_and_close_send_repeated_zero_commands(self) -> None:
        # Given: a ready transport with one command older than the watchdog.
        clock = FakeClock()
        socket = FakeWebSocket(successful_mode_handler)
        transport, _ = make_transport(socket, clock)
        transport.start()
        transport.update_twist(linear_mps=0.25, angular_radps=0.5)
        clock.sleep(0.3)

        # When: one stream tick runs and the transport closes.
        transport.send_current()
        transport.close()

        # Then: the watchdog tick and all close barriers are complete zeros.
        twist_data = [
            payload["data"]
            for payload in socket.sent
            if payload["title"] == "request_twist"
        ]
        self.assertEqual(twist_data[-5:], [{"x": 0.0, "y": 0.0, "z": 0.0}] * 5)
        self.assertTrue(socket.closed)

    def test_successful_frames_are_observed_byte_for_byte_after_send(self) -> None:
        # Given: a transport observer and a fake WebSocket that preserves raw frames.
        clock = FakeClock()
        socket = FakeWebSocket(successful_mode_handler)
        transport, _ = make_transport(socket, clock)
        observed = []
        transport.observe_sent_frames(observed.append)

        # When: startup, a watchdog tick, and normal close send only zero motion.
        transport.start()
        clock.sleep(0.3)
        transport.send_current()
        transport.close()

        # Then: every successful socket frame is observed without reserialization.
        self.assertEqual(observed, socket.sent_payloads)

    def test_nonfinite_update_emits_zero_over_mocked_websocket(self) -> None:
        # Given: a ready mocked session and a nonfinite physical command.
        clock = FakeClock()
        socket = FakeWebSocket(successful_mode_handler)
        transport, _ = make_transport(socket, clock)
        transport.start()

        # When: the unsafe input reaches a real transport stream tick.
        transport.update_twist(math.nan, 0.5)
        transport.send_current()

        # Then: the wire receives a complete zero rather than a partial command.
        self.assertEqual(socket.sent[-1]["data"], {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_documented_high_level_helpers_emit_typed_payloads(self) -> None:
        # Given: a ready transport using the documented high-level protocol.
        clock = FakeClock()
        socket = FakeWebSocket(successful_mode_handler)
        transport, _ = make_transport(socket, clock)
        transport.start()

        # When: stair, emergency, odometry, and IMU abstractions are requested.
        transport.request_stair_mode(True)
        transport.request_emergency_stop()
        transport.set_odometry_enabled(True)
        transport.set_imu_enabled(False)

        # Then: exact documented titles and payloads cross the wire.
        self.assertEqual(
            [(item["title"], item["data"]) for item in socket.sent[-4:]],
            [
                ("request_stair_mode", {"enable": True}),
                ("request_emgy_stop", {}),
                ("request_enable_odom", {"enable": True}),
                ("request_enable_imu", {"enable": False}),
            ],
        )


if __name__ == "__main__":
    unittest.main()
