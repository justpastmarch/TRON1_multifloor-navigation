"""Tests for single-use, ownership-bound stair admission tokens."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "mission_manager" / "src"))

from mission_manager.stair_admission import (  # noqa: E402
    StairAdmissionBroker,
    StairAdmissionContext,
    StairAdmissionRequest,
)
from mission_manager.stair_entry_gate import StairEntryDecision  # noqa: E402


class StairAdmissionBrokerTest(unittest.TestCase):
    def test_valid_token_is_accepted_exactly_once(self) -> None:
        # Given: one fresh entry decision bound to a profile, direction, and epoch.
        broker = StairAdmissionBroker(clock=lambda: 10.0, lifetime_sec=1.0)
        context = StairAdmissionContext("profile_up", 1, 4)
        token = broker.issue(context, lambda: StairEntryDecision(True, "aligned"))
        request = StairAdmissionRequest(token, "profile_up", 1, 4)

        # When: the same action context consumes the grant twice.
        first = broker.validate(request)
        reused = broker.validate(request)

        # Then: only the first validation can authorize command ownership.
        self.assertTrue(first.accepted)
        self.assertFalse(reused.accepted)

    def test_context_mismatch_and_live_recheck_reject(self) -> None:
        cases = (
            (StairAdmissionRequest("TOKEN", "wrong", 1, 4), StairEntryDecision(True, "aligned")),
            (StairAdmissionRequest("TOKEN", "profile_up", 1, 4), StairEntryDecision(False, "pose drifted")),
        )
        for request, decision in cases:
            with self.subTest(request=request, decision=decision):
                # Given: a grant whose token is deterministic for this boundary test.
                broker = StairAdmissionBroker(clock=lambda: 10.0, lifetime_sec=1.0, token_factory=lambda: "TOKEN")
                broker.issue(StairAdmissionContext("profile_up", 1, 4), lambda: decision)

                # When: context or current entry evidence fails validation.
                result = broker.validate(request)

                # Then: the grant is rejected without becoming reusable.
                self.assertFalse(result.accepted)

    def test_expiry_during_bounded_recheck_cannot_admit(self):
        now=[10.]
        broker=StairAdmissionBroker(clock=lambda:now[0], lifetime_sec=1.)
        def recheck():
            now[0]=11.1
            return StairEntryDecision(True,'new observation')
        token=broker.issue(StairAdmissionContext('up',1,0),recheck)
        result=broker.validate(StairAdmissionRequest(token,'up',1,0))
        self.assertFalse(result.accepted)
        self.assertIn('during re-observation',result.reason)


if __name__ == "__main__":
    unittest.main()
