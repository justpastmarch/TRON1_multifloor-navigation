"""ROS session that drives the stair supervisor from recorded odometry."""

from __future__ import annotations

import json
from pathlib import Path
import re
import threading
import time
from typing import Final, Optional, Tuple

import actionlib
from actionlib_msgs.msg import GoalStatus
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import rosbag
import rospy
from sensor_msgs.msg import Image
from stair_supervisor.msg import (
    StairTraversalAction,
    StairTraversalFeedback,
    StairTraversalGoal,
    SupervisorState,
)
from std_msgs.msg import String

from .bag_replay import restamp_odometry
from .configuration import Direction, StairProfile, load_stair_configuration
from .stair_evidence import Phase
from .state_machine_dashboard import (
    PHASES,
    ReplayOutcome,
    StateMachineSnapshot,
    render_state_machine,
)


ODOM_TOPIC: Final = "/tron/wheel_odom_raw"
IMAGE_TOPIC: Final = "/replay/stair_state_machine/image"
STATUS_TOPIC: Final = "/replay/stair_state_machine/status"
PROGRESS_PATTERN: Final = re.compile(
    r"progress=(?P<progress>-?[0-9.]+) threshold=(?P<threshold>-?[0-9.]+)"
)


class ReplayInputError(RuntimeError):
    """Report an invalid bag, profile, or replay argument."""


def load_profile(config_dir: Path, profile_id: str) -> StairProfile:
    """Resolve exactly one configured production stair profile."""
    configuration = load_stair_configuration(config_dir)
    profile = next((item for item in configuration.profiles if item.id == profile_id), None)
    if profile is None:
        raise ReplayInputError(f"unknown stair profile: {profile_id}")
    return profile


class ReplaySession:
    """Own mutable ROS replay and visualization state for one recorded run."""

    def __init__(self, bag_path: Path, profile: StairProfile, output_dir: Path) -> None:
        self._bag_path = bag_path
        self._profile = profile
        self._output_dir = output_dir
        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._odom = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=20)
        self._image = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1, latch=True)
        self._status = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)
        self._state_subscriber = rospy.Subscriber(
            "/stair_supervisor/state", SupervisorState, self._accept_state, queue_size=10
        )
        self._client = actionlib.SimpleActionClient(
            "/stair_traversal", StairTraversalAction
        )
        self._supervisor_state = "DISARMED"
        self._active_phase = Phase.VERIFY_ENTRY
        self._completed: list[Phase] = []
        self._detail = "waiting for action server"
        self._progress_value = 0.0
        self._progress_threshold = 0.0
        self._outcome = ReplayOutcome.RUNNING
        self._elapsed_sec = 0.0
        self._sample_count = 0

    def _accept_state(self, message: SupervisorState) -> None:
        labels = {
            SupervisorState.DISARMED: "DISARMED",
            SupervisorState.NAV: "NAV",
            SupervisorState.STAIR: "STAIR",
            SupervisorState.FAULT: "FAULT",
        }
        with self._lock:
            self._supervisor_state = labels.get(message.state, f"UNKNOWN({message.state})")
            self._set_detail(message.detail)

    def _accept_feedback(self, feedback: StairTraversalFeedback) -> None:
        phase = Phase(feedback.phase)
        with self._lock:
            if phase is not self._active_phase:
                if self._active_phase not in self._completed:
                    self._completed.append(self._active_phase)
                self._active_phase = phase
            self._set_detail(feedback.detail)

    def _set_detail(self, detail: str) -> None:
        self._detail = detail
        match = PROGRESS_PATTERN.search(detail)
        if match is not None:
            self._progress_value = float(match.group("progress"))
            self._progress_threshold = float(match.group("threshold"))

    def _progress(self) -> Tuple[float, float]:
        return self._progress_value, self._progress_threshold

    def _snapshot(self) -> StateMachineSnapshot:
        with self._lock:
            progress, threshold = self._progress()
            return StateMachineSnapshot(
                self._supervisor_state,
                self._active_phase,
                tuple(self._completed),
                self._outcome,
                self._detail,
                self._bag_path.name,
                self._profile.id,
                self._elapsed_sec,
                self._sample_count,
                progress,
                threshold,
            )

    def publish_dashboard(self) -> None:
        snapshot = self._snapshot()
        image = render_state_machine(snapshot)
        message = self._bridge.cv2_to_imgmsg(image, "bgr8")
        message.header.stamp = rospy.Time.now()
        self._image.publish(message)
        self._status.publish(
            String(
                data=json.dumps(
                    {
                        "active_phase": snapshot.active_phase.value,
                        "bag": snapshot.bag_name,
                        "detail": snapshot.detail,
                        "elapsed_sec": round(snapshot.elapsed_sec, 3),
                        "outcome": snapshot.outcome.value,
                        "profile": snapshot.profile_id,
                        "sample_count": snapshot.sample_count,
                        "supervisor_state": snapshot.supervisor_state,
                    },
                    sort_keys=True,
                )
            )
        )

    def _finish(self, outcome: ReplayOutcome, detail: str) -> None:
        with self._lock:
            self._outcome = outcome
            self._set_detail(detail)
            if outcome is ReplayOutcome.SUCCEEDED:
                for phase in PHASES:
                    if phase not in self._completed:
                        self._completed.append(phase)
        self.publish_dashboard()
        image = render_state_machine(self._snapshot())
        self._output_dir.mkdir(parents=True, exist_ok=True)
        image_path = self._output_dir / "final-state.png"
        if not cv2.imwrite(str(image_path), image):
            raise ReplayInputError(f"failed to write {image_path}")
        snapshot = self._snapshot()
        (self._output_dir / "result.json").write_text(
            json.dumps(
                {
                    "active_phase": snapshot.active_phase.value,
                    "bag": str(self._bag_path),
                    "detail": snapshot.detail,
                    "elapsed_sec": snapshot.elapsed_sec,
                    "outcome": snapshot.outcome.value,
                    "profile": snapshot.profile_id,
                    "sample_count": snapshot.sample_count,
                    "supervisor_state": snapshot.supervisor_state,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"[REPLAY {outcome.value}] {detail}")
        print(f"[REPLAY ARTIFACT] {image_path}")

    def run(self, rate: float, start_sec: float) -> ReplayOutcome:
        """Send one action and preserve each recorded odometry interval."""
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            raise ReplayInputError("stair traversal action server did not start")
        deadline = time.monotonic() + 5.0
        while self._odom.get_num_connections() < 1 and time.monotonic() < deadline:
            rospy.sleep(0.02)
        if self._odom.get_num_connections() < 1:
            raise ReplayInputError("stair supervisor did not subscribe to replay odometry")
        direction = (
            StairTraversalGoal.UP
            if self._profile.direction is Direction.UP
            else StairTraversalGoal.DOWN
        )
        self._client.send_goal(
            StairTraversalGoal(stair_id=self._profile.id, direction=direction),
            feedback_cb=self._accept_feedback,
        )
        first_recorded: Optional[float] = None
        previous_recorded: Optional[float] = None
        last_preview = -1.0
        with rosbag.Bag(str(self._bag_path), "r") as bag:
            topic = bag.get_type_and_topic_info().topics.get(ODOM_TOPIC)
            if topic is None or topic.msg_type != "nav_msgs/Odometry":
                raise ReplayInputError(
                    f"{ODOM_TOPIC} must be recorded as nav_msgs/Odometry"
                )
            start_time = rospy.Time.from_sec(bag.get_start_time() + start_sec)
            for _topic, message, recorded_at in bag.read_messages(
                topics=[ODOM_TOPIC], start_time=start_time
            ):
                recorded_sec = recorded_at.to_sec()
                if first_recorded is None:
                    first_recorded = recorded_sec
                if previous_recorded is not None:
                    time.sleep(max(0.0, recorded_sec - previous_recorded) / rate)
                previous_recorded = recorded_sec
                self._odom.publish(restamp_odometry(message, rospy.Time.now()))
                with self._lock:
                    self._elapsed_sec = recorded_sec - first_recorded
                    self._sample_count += 1
                if self._elapsed_sec - last_preview >= 0.1:
                    self.publish_dashboard()
                    last_preview = self._elapsed_sec
                if self._client.get_state() in (
                    GoalStatus.SUCCEEDED,
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                ):
                    break
        state = self._client.get_state()
        result = self._client.get_result()
        if state == GoalStatus.SUCCEEDED:
            detail = result.reason if result is not None else "stair traversal succeeded"
            self._finish(ReplayOutcome.SUCCEEDED, detail)
        elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
            detail = result.reason if result is not None else "stair traversal faulted"
            self._finish(ReplayOutcome.FAULTED, detail)
        else:
            self._client.cancel_goal()
            self._client.wait_for_result(rospy.Duration(2.0))
            self._finish(
                ReplayOutcome.INCOMPLETE,
                "recorded odometry ended before the active phase completed",
            )
        return self._outcome
