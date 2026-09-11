from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import NamedTuple


SCRIPT = Path(__file__).resolve().parents[1] / "sensor_joy_bridge.py"
SPEC = importlib.util.spec_from_file_location("sensor_joy_bridge", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)


class SdkSensorJoy(NamedTuple):
    stamp: int
    axes: tuple[float, ...]
    buttons: tuple[int, ...]


def test_parse_sensor_joy_preserves_raw_values_and_nanosecond_stamp() -> None:
    # Given: an SDK-shaped physical controller sample.
    sdk_sample = SdkSensorJoy(
        stamp=1_700_000_000_123_456_789,
        axes=(0.25, -0.5, 0.75),
        buttons=(0, 1, 0, 1),
    )

    # When: the callback payload is copied and its ROS timestamp fields are derived.
    sample = bridge.sample_sensor_joy(sdk_sample)
    seconds, nanoseconds = bridge.ros_stamp_parts(sample.stamp_ns)

    # Then: no axis/button conversion occurs and nanoseconds are losslessly split.
    assert sample.axes == (0.25, -0.5, 0.75)
    assert sample.buttons == (0, 1, 0, 1)
    assert (seconds, nanoseconds) == (1_700_000_000, 123_456_789)
