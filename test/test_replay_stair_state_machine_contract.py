"""Safety contract for the operator-facing stair bag replay wrapper."""

from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "replay_stair_state_machine.sh"


def test_replay_uses_an_isolated_master_and_in_memory_robot_transport() -> None:
    # Given/When: the executable replay surface is inspected.
    source = SCRIPT.read_text(encoding="utf-8")

    # Then: recorded inputs cannot reach the normal graph or physical robot socket.
    assert "http://127.0.0.1:11320" in source
    assert "export ROS_IP=127.0.0.1" in source
    assert "unset ROS_HOSTNAME" in source
    assert "synthetic_stair_supervisor_node.py" in source
    assert "ROBOT_HOST" not in source


def test_replay_wires_the_production_profile_and_visualization_driver() -> None:
    # Given/When: the wrapper's machine-consumed commands are inspected.
    source = SCRIPT.read_text(encoding="utf-8")

    # Then: one production profile drives the real ROS supervisor and image surface.
    assert "_config_dir:=" in source
    assert "replay_stair_state_machine.py" in source
    assert "/replay/stair_state_machine/image" in source
