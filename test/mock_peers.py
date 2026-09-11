"""In-memory non-node peers for deterministic ROS-adapter and WebSocket tests."""

from __future__ import annotations

from typing import Dict, List, Mapping, Tuple

from fixture_models import WebSocketFrame


class FakeClock:
    """Mutable test clock whose only purpose is controlled time advancement."""

    def __init__(self, now_ns: int) -> None:
        self._now_ns = now_ns

    def now_ns(self) -> int:
        return self._now_ns

    def advance_ns(self, delta_ns: int) -> None:
        if delta_ns < 0:
            raise ValueError("fake clock cannot move backwards")
        self._now_ns += delta_ns


class FakeActionPeer:
    """Records goals and returns queued terminal states without using ROS."""

    def __init__(self, terminal_states: Tuple[str, ...]) -> None:
        self._terminal_states = list(terminal_states)
        self.goals: List[str] = []

    def send_goal(self, goal: str) -> str:
        self.goals.append(goal)
        if not self._terminal_states:
            raise RuntimeError("no fake action terminal state queued")
        return self._terminal_states.pop(0)


class FakeServicePeer:
    """Returns named service responses and records request order."""

    def __init__(self, responses: Mapping[str, str]) -> None:
        self._responses: Dict[str, str] = dict(responses)
        self.requests: List[Tuple[str, str]] = []

    def call(self, service: str, request: str) -> str:
        self.requests.append((service, request))
        if service not in self._responses:
            raise KeyError(service)
        return self._responses[service]


class FakeWebSocketPeer:
    """Replays inbound frames and captures outbound payloads without sockets."""

    def __init__(self, frames: Tuple[WebSocketFrame, ...]) -> None:
        self._frames = list(frames)
        self.sent_payloads: List[str] = []

    def receive(self) -> WebSocketFrame:
        if not self._frames:
            raise EOFError("fake WebSocket transcript exhausted")
        return self._frames.pop(0)

    def send(self, payload_json: str) -> None:
        self.sent_payloads.append(payload_json)
