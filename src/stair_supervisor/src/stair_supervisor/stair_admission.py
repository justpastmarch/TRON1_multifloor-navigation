"""Bounded ROS client for mission-owned, single-use stair admission."""

from __future__ import annotations

import queue
import threading

import rospy
from stair_supervisor.srv import ValidateStairAdmission

from .configuration import Direction
from .supervisor_types import AdmissionDecision, StairGoal


class RosStairAdmissionValidator:
    """Validate one action goal without allowing the mission peer to stall motion ownership."""

    def __init__(self, service_name: str, timeout_sec: float) -> None:
        self._service_name = service_name
        self._timeout_sec = timeout_sec
        self._proxy = rospy.ServiceProxy(service_name, ValidateStairAdmission)

    def validate(self, goal: StairGoal, ownership_epoch: int) -> AdmissionDecision:
        try:
            rospy.wait_for_service(self._service_name, timeout=self._timeout_sec)
        except rospy.ROSException:
            return AdmissionDecision(False, True, "stair admission service unavailable")
        responses = queue.Queue(maxsize=1)
        direction = 1 if goal.direction is Direction.UP else 2

        def invoke() -> None:
            try:
                response = self._proxy(
                    goal.admission_token,
                    goal.stair_id,
                    direction,
                    ownership_epoch,
                )
                responses.put((True, response))
            except rospy.ServiceException as error:
                responses.put((False, str(error)))

        threading.Thread(target=invoke, daemon=True).start()
        try:
            succeeded, value = responses.get(timeout=self._timeout_sec)
        except queue.Empty:
            return AdmissionDecision(False, True, "stair admission service response timeout")
        if not succeeded:
            return AdmissionDecision(False, True, str(value))
        return AdmissionDecision(bool(value.accepted), False, str(value.reason))


class AllowStairAdmissionValidator:
    """Explicit constructor-only authorizer for isolated replay and synthetic tests."""

    def validate(self, _goal: StairGoal, _ownership_epoch: int) -> AdmissionDecision:
        return AdmissionDecision(True, False, "isolated test admission")
