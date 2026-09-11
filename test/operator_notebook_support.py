from __future__ import annotations

import math
import sys
import types
from unittest import mock


class RosException(Exception):
    pass


class Stamp:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds

    def to_nsec(self) -> int:
        return self.nanoseconds


class FakeTime:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now


class LatchedAmcl:
    ROSException = RosException
    ServiceException = RosException

    def __init__(
        self,
        clock: FakeTime,
        *,
        advance_stamps: bool,
        messages_available: bool,
        service_available: bool,
        covariance: tuple[float, float, float],
    ) -> None:
        self.clock = clock
        self.advance_stamps = advance_stamps
        self.messages_available = messages_available
        self.service_available = service_available
        self.covariance = covariance
        self.stamp = 100
        self.service_calls = 0

    def wait_for_service(self, name: str, timeout: float) -> None:
        if name != "/request_nomotion_update" or timeout <= 0.0:
            raise AssertionError("unexpected service wait")
        if not self.service_available:
            raise RosException("service timeout")

    def ServiceProxy(self, name: str, service_type: object):
        if name != "/request_nomotion_update" or service_type is None:
            raise AssertionError("unexpected service proxy")
        return self.request_update

    def request_update(self) -> None:
        self.service_calls += 1
        if self.advance_stamps:
            self.stamp += 1

    def wait_for_message(self, topic: str, message_type: object, timeout: float):
        if topic != "/amcl_pose" or message_type is None or timeout <= 0.0:
            raise AssertionError("unexpected message wait")
        if not self.messages_available:
            self.clock.now += timeout
            raise RosException("message timeout")
        covariance = [0.0] * 36
        covariance[0], covariance[7], covariance[35] = self.covariance
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=Stamp(self.stamp)),
            pose=types.SimpleNamespace(
                covariance=covariance,
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=1.0, y=2.0),
                    orientation=types.SimpleNamespace(z=0.0, w=1.0),
                ),
            ),
        )

    def sleep(self, duration: float) -> None:
        self.clock.now += duration


def run_localization_gate(
    source: str,
    *,
    advance_stamps: bool = True,
    messages_available: bool = True,
    service_available: bool = True,
    covariance: tuple[float, float, float] = (0.04, 0.04, 0.09),
) -> tuple[int, list[int]]:
    policy = types.SimpleNamespace(
        required_pose_samples=3,
        max_covariance_x=0.05,
        max_covariance_y=0.05,
        max_covariance_yaw=0.10,
    )
    readiness = types.ModuleType("multifloor_manager.readiness")
    readiness.DEFAULT_READINESS_POLICY = policy
    package = types.ModuleType("multifloor_manager")
    package.__path__ = []
    clock = FakeTime()
    fake_rospy = LatchedAmcl(
        clock,
        advance_stamps=advance_stamps,
        messages_available=messages_available,
        service_available=service_available,
        covariance=covariance,
    )
    namespace = {
        "Empty": object,
        "INITIAL_X": 1.0,
        "INITIAL_Y": 2.0,
        "INITIAL_YAW": 0.0,
        "PoseWithCovarianceStamped": object,
        "math": math,
        "rospy": fake_rospy,
        "time": clock,
    }
    with mock.patch.dict(
        sys.modules,
        {"multifloor_manager": package, "multifloor_manager.readiness": readiness},
    ):
        exec(source, namespace)
    stamps = [sample.header.stamp.to_nsec() for sample in namespace["samples"]]
    return fake_rospy.service_calls, stamps
