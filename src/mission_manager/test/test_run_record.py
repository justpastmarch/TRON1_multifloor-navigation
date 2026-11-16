"""Run summary written next to each console BAG (record only, never motion)."""
import json
import tempfile
import unittest
from pathlib import Path

from mission_manager.run_record import RunRecord


class Clock:
    def __init__(self): self.t = 1_800_000_000.0
    def __call__(self): return self.t


class RunRecordTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.dir = Path(temp.name); self.clock = Clock()
        self.events = []
        self.run = RunRecord(clock=self.clock, version=lambda: dict(commit='abc123', branch='fix/stair-safety', dirty=False))
        self.run.events_provider = lambda: list(self.events)
        self.bag = self.dir / 'ui_test_20261116_120000_deadbeef.bag'

    def test_full_run_is_summarised_next_to_the_bag(self):
        self.events.append(dict(time=self.clock.t - 60, kind='old', detail='before this BAG'))
        self.run.begin(self.bag, self.dir / 'ui_test_20261116_120000_deadbeef_raw.bag')
        self.run.mission(kind='navigate', destination='stair_4f_from_3f', start_floor='3F',
                         segments=[dict(type='STAIR', target_id='stair_4f_from_3f', profile_id='stair_3f_4f_up')])
        self.run.note('조건: 계단참 max_w 0.3')
        self.run.control(dict(config_sha256='f' * 64, route_id='stair_3f_4f_up',
                              first_fault=dict(at=12.5, phase='LANDING', tracking_state='SUPPORT_LIMIT', reason='clearance 0.019 m', clearance_m=0.019)))
        self.run.control(dict(first_fault=dict(at=12.5, phase='LANDING')))     # same fault again: kept once
        self.clock.t += 95.0
        self.events.append(dict(time=self.clock.t - 5, kind='operator_phase', detail='같은 단계 이어가기: LANDING'))
        self.run.result(4, 0, 'mission complete', 'mission_1')
        path = self.run.finish(saved=True)
        self.assertEqual(path, str(self.dir / 'ui_test_20261116_120000_deadbeef.run.json'))
        data = json.loads(Path(path).read_text())
        self.assertEqual(data['schema'], 'tron-run/1')
        self.assertEqual(data['code']['commit'], 'abc123')
        self.assertEqual(data['config_sha256'], 'f' * 64)
        self.assertEqual(data['missions'][0]['segments'][0]['profile_id'], 'stair_3f_4f_up')
        self.assertEqual(data['results'][0]['result_name'], 'OK')
        self.assertEqual(len(data['first_faults']), 1)
        self.assertEqual(data['first_faults'][0]['reason'], 'clearance 0.019 m')
        self.assertEqual([e['kind'] for e in data['console_events']], ['operator_phase'])
        self.assertEqual(data['notes'][0]['text'], '조건: 계단참 max_w 0.3')
        self.assertEqual((data['duration_s'], data['bag_saved']), (95.0, True))
        self.assertNotIn('started_unix', data)

    def test_note_after_save_is_appended_to_the_last_summary(self):
        self.run.begin(self.bag); self.run.finish(saved=False, error='index missing')
        result = self.run.note('  조종기로 계단참에서 회수  ')
        self.assertEqual(result['saved_to'], 'last')
        data = json.loads(self.bag.with_suffix('.run.json').read_text())
        self.assertEqual(data['notes'][-1]['text'], '조종기로 계단참에서 회수')
        self.assertEqual((data['bag_saved'], data['bag_error']), (False, 'index missing'))
        self.assertEqual(self.run.snapshot()['notes'][-1]['text'], '조종기로 계단참에서 회수')

    def test_nothing_is_recorded_outside_a_recording(self):
        self.run.mission(kind='navigate', destination='home_3f')
        self.run.result(3, 0, 'ok')
        self.run.control(dict(first_fault=dict(at=1)))
        self.assertIsNone(self.run.finish(saved=True))
        with self.assertRaisesRegex(ValueError, '녹화가 없습니다'):
            self.run.note('memo')
        with self.assertRaisesRegex(ValueError, '메모를 입력'):
            self.run.note('   ')
        self.assertEqual(list(self.dir.iterdir()), [])

    def test_a_failing_event_source_never_blocks_the_summary(self):
        self.run.events_provider = lambda: 1 / 0
        self.run.begin(self.bag)
        self.assertIsNotNone(self.run.finish(saved=True))
        self.assertEqual(json.loads(self.bag.with_suffix('.run.json').read_text())['console_events'], [])


if __name__ == '__main__':
    unittest.main()
