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


    def test_post_run_check_is_attached_to_the_summary(self):
        report = dict(config=dict(match=True), flights=[dict(route='stair_5f_rf_up', phase='FORWARD_SEGMENT_1', drift_slope_deg=0.83,
                      min_clearance_m=0.126)], turns=[dict(route='stair_5f_rf_up', phase='TURN_TO_NEXT_FLIGHT', duration_s=40.04,
                      forward_creep_mps=0.05, min_clearance_m=0.031)], first_faults=[], recovery_ticks={}, tx=dict(gap_p99_s=0.05, gap_max_s=0.1))
        self.run.runner = lambda bag: report
        self.run.begin(self.bag); self.run.finish(saved=True)
        self.run.check_async().join(5)
        check = json.loads(self.bag.with_suffix('.run.json').read_text())['field_check']
        self.assertEqual(check['state'], 'done')
        self.assertEqual((check['summary']['config_match'], check['summary']['min_clearance_m']), (True, 0.031))
        self.assertEqual(check['summary']['flights'][0]['slope_deg'], 0.8)
        self.assertEqual(self.run.snapshot()['field_check']['state'], 'done')

    def test_failed_check_is_reported_and_unsaved_bags_are_not_checked(self):
        self.run.runner = lambda bag: 1 / 0
        self.run.begin(self.bag); self.run.finish(saved=True)
        self.run.check_async().join(5)
        self.assertEqual(json.loads(self.bag.with_suffix('.run.json').read_text())['field_check']['state'], 'error')
        self.run.begin(self.bag); self.run.finish(saved=False)
        self.assertIsNone(self.run.check_async())

    def test_real_field_check_script_reads_a_console_bag(self):
        import rosbag
        from std_msgs.msg import String
        import rospy
        from mission_manager.run_record import FIELD_CHECK, RunRecord as Real
        import hashlib
        config = FIELD_CHECK.parents[1] / 'config' / 'stair_lidar_3f_4f_test.yaml'
        sha = hashlib.sha256(config.read_bytes()).hexdigest()
        with rosbag.Bag(str(self.bag), 'w') as bag:
            for i in range(20):
                t = rospy.Time.from_sec(1.8e9 + i * .1)
                bag.write('/stair_supervisor/tracking_status', String(json.dumps(dict(config_sha256=sha))), t)
                bag.write('/stair_supervisor/websocket_tx', String(json.dumps(dict(title='request_twist'))), t)
        summary = __import__('mission_manager.run_record', fromlist=['x']).summarize_field_check(Real._run_field_check(self.bag, timeout=120))
        self.assertTrue(summary['config_match'])
        self.assertEqual((summary['flights'], summary['first_faults']), ([], []))


if __name__ == '__main__':
    unittest.main()
