"""Bound ROS service availability and response time for fail-closed actions."""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading

import rospy


@dataclass(frozen=True)
class ServiceCallError(Exception):
    __slots__ = ("service", "detail")
    service: str
    detail: str

    def __str__(self) -> str:
        return "{}: {}".format(self.service, self.detail)


class BoundedServiceCaller:
    """Call a rospy service without allowing a peer to stall the action thread."""

    def __init__(self, timeout: float) -> None:
        self.timeout = timeout

    def call(self, name: str, proxy, *arguments):
        try:
            rospy.wait_for_service(name, timeout=self.timeout)
        except rospy.ROSException:
            raise ServiceCallError(name, "unavailable") from None
        responses = queue.Queue(maxsize=1)

        def invoke() -> None:
            try:
                responses.put((True, proxy(*arguments)))
            except rospy.ServiceException as error:
                responses.put((False, error))

        threading.Thread(target=invoke, daemon=True).start()
        try:
            succeeded, value = responses.get(timeout=self.timeout)
        except queue.Empty:
            raise ServiceCallError(name, "response timeout") from None
        if not succeeded:
            raise ServiceCallError(name, str(value))
        return value
