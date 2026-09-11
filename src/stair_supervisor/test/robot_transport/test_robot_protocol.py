from __future__ import annotations

import json
import unittest

from stair_supervisor.robot_protocol import (
    ProtocolError,
    RequestEnvelope,
    RequestTitle,
    encode_request,
    parse_message,
)


class RobotProtocolTest(unittest.TestCase):
    def test_request_uses_documented_guid_json_envelope(self) -> None:
        # Given: a typed twist request.
        # When: it is serialized for the robot.
        encoded = encode_request(RequestEnvelope(
            accid="WF_TEST_001",
            title=RequestTitle.TWIST,
            timestamp_ms=1_700_000_000_123,
            guid="guid-1",
            data={"x": 0.25, "y": 0.0, "z": -0.5},
        ))

        # Then: exactly the documented five envelope fields are emitted.
        payload = json.loads(encoded)
        self.assertEqual(
            payload,
            {
                "accid": "WF_TEST_001",
                "title": "request_twist",
                "timestamp": 1_700_000_000_123,
                "guid": "guid-1",
                "data": {"x": 0.25, "y": 0.0, "z": -0.5},
            },
        )

    def test_malformed_or_incomplete_response_is_rejected(self) -> None:
        # Given: generated-looking JSON that omits a required correlation field.
        malformed = '{"accid":"WF_TEST_001","title":"response_walk_mode","data":{}}'

        # When/Then: boundary parsing rejects rather than guessing defaults.
        with self.assertRaises(ProtocolError):
            parse_message(malformed)

    def test_fractional_millisecond_robot_timestamp_is_normalized(self) -> None:
        # Given: the deployed TRON1 wire representation with sub-millisecond precision.
        frame = (
            '{"accid":"WF_TEST_001","title":"response_walk_mode",'
            '"timestamp":1787286632469.229248,"guid":"g",'
            '"data":{"result":"success"}}'
        )

        # When: the response crosses the strict protocol boundary.
        message = parse_message(frame)

        # Then: downstream receives comparable integer milliseconds.
        self.assertEqual(message.timestamp_ms, 1_787_286_632_469)

    def test_dirty_generated_response_fields_and_nonfinite_data_are_rejected(self) -> None:
        # Given: generated-looking envelopes carrying extra state or NaN telemetry.
        dirty = (
            '{"accid":"WF_TEST_001","title":"response_walk_mode",'
            '"timestamp":1700000000000,"guid":"g","data":{"result":"success"},'
            '"trusted":true}'
        )
        nonfinite = (
            '{"accid":"WF_TEST_001","title":"notify_imu",'
            '"timestamp":1700000000000,"guid":"g","data":{"value":NaN}}'
        )
        nonfinite_timestamp = (
            '{"accid":"WF_TEST_001","title":"notify_imu",'
            '"timestamp":NaN,"guid":"g","data":{}}'
        )

        # When/Then: neither payload crosses the strict protocol boundary.
        for payload in (dirty, nonfinite, nonfinite_timestamp):
            with self.subTest(payload=payload), self.assertRaises(ProtocolError):
                parse_message(payload)


if __name__ == "__main__":
    unittest.main()
