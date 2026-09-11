from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]

EXPECTED_INTERFACES = {
    "src/mission_manager/action/Mission.action": (
        (
            "string destination_id",
            "string mission_type",
            "bool return_after_task",
        ),
        (
            "uint8 OK=0",
            "uint8 BUSY=1",
            "uint8 INVALID_GOAL=2",
            "uint8 CAPABILITY_DISABLED=3",
            "uint8 NAVIGATION_FAILED=4",
            "uint8 STAIR_FAILED=5",
            "uint8 LOCALIZATION_FAILED=6",
            "uint8 SCAN_FAILED=7",
            "uint8 COMMUNICATION_LOST=8",
            "uint8 MISSION_ABORT=9",
            "uint8 result_code",
            "string reason",
            "string mission_id",
            "string artifact_path",
        ),
        (
            "string mission_id",
            "string state",
            "string current_floor",
            "string segment_type",
            "uint32 segment_index",
            "uint32 segment_count",
            "string target_id",
        ),
    ),
    "src/multifloor_manager/action/FloorTransition.action": (
        ("string transition_id", "string target_floor", "uint64 stair_ownership_epoch"),
        (
            "uint8 OK=0",
            "uint8 BUSY=1",
            "uint8 INVALID_GOAL=2",
            "uint8 LOCALIZATION_FAILED=6",
            "uint8 result_code",
            "string floor_id",
            "uint64 map_generation",
            "string reason",
        ),
        ("string phase", "string detail"),
    ),
    "src/multifloor_manager/msg/FloorState.msg": (
        (
            "std_msgs/Header header",
            "uint8 UNKNOWN=0",
            "uint8 TRANSITIONING=1",
            "uint8 READY=2",
            "uint8 FAULT=3",
            "string floor_id",
            "uint64 map_generation",
            "uint8 state",
            "string detail",
        ),
    ),
    "src/stair_supervisor/action/StairTraversal.action": (
        (
            "string stair_id",
            "uint8 UP=1",
            "uint8 DOWN=2",
            "uint8 direction",
        ),
        (
            "uint8 OK=0",
            "uint8 BUSY=1",
            "uint8 INVALID_GOAL=2",
            "uint8 CAPABILITY_DISABLED=3",
            "uint8 STAIR_FAILED=5",
            "uint8 COMMUNICATION_LOST=8",
            "uint8 result_code",
            "string reason",
            "uint64 ownership_epoch",
        ),
        ("string phase", "string detail"),
    ),
    "src/stair_supervisor/msg/SupervisorState.msg": (
        (
            "std_msgs/Header header",
            "uint8 DISARMED=0",
            "uint8 NAV=1",
            "uint8 STAIR=2",
            "uint8 FAULT=3",
            "uint8 state",
            "bool connected",
            "string detail",
            "uint64 ownership_epoch",
        ),
    ),
}


class InterfaceContractTest(unittest.TestCase):
    def test_source_definitions_match_the_complete_contract(self) -> None:
        # Given: the five interfaces assigned to their owning packages.
        for relative_path, expected_sections in EXPECTED_INTERFACES.items():
            with self.subTest(interface=relative_path):
                # When: comments and blank lines are removed from each definition.
                source = (ROOT / relative_path).read_text(encoding="utf-8")
                sections = tuple(
                    tuple(
                        line.strip()
                        for line in section.splitlines()
                        if line.strip() and not line.lstrip().startswith("#")
                    )
                    for section in source.split("---")
                )

                # Then: every field and constant is present in exact wire order.
                self.assertEqual(sections, expected_sections)


if __name__ == "__main__":
    unittest.main()
