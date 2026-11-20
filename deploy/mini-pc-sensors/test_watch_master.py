import importlib.util
import signal
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('watch', Path(__file__).with_name('watch_master.py'))
watch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watch)


class WatchTest(unittest.TestCase):
    def run_case(self, identities, waits, poll=None, env=None):
        event = Mock()
        event.is_set.return_value = False
        event.wait.side_effect = waits
        child = Mock()
        child.poll.return_value = poll
        with patch.object(watch.threading, 'Event', return_value=event), \
             patch.object(watch.signal, 'signal'), \
             patch.object(watch, 'master_identity', side_effect=identities) as probe, \
             patch.object(watch.subprocess, 'Popen', return_value=child) as spawn, \
             patch.dict(watch.os.environ, env or dict(ROS_MASTER_URI='http://fixture:11311')):
            for key in ('ROS_MASTER_URI_ALT', 'ROS_IP_ALT'):
                if key not in (env or {}):
                    watch.os.environ.pop(key, None)
            result = watch.main()
        self.probes = [call.args[0] for call in probe.call_args_list]
        return result, child, spawn

    def test_wait_then_launch_and_stop(self):
        result, child, spawn = self.run_case([None, (1, 'a')], [False, True])
        self.assertEqual(result, 0)
        self.assertIn('--wait', spawn.call_args[0][0])
        child.send_signal.assert_called_once_with(signal.SIGINT)

    def test_master_replaced(self):
        result, child, _ = self.run_case([(1, 'a'), (1, 'b')], [False])
        self.assertEqual(result, 1)
        child.send_signal.assert_called_once_with(signal.SIGINT)

    def test_brief_disconnect_recovers(self):
        result, _, _ = self.run_case([(1, 'a'), None, (1, 'a')], [False, False, True])
        self.assertEqual(result, 0)

    def test_short_outage_keeps_sensors(self):
        # 20 s unreachable (hotspot drop) then back: no restart.
        result, child, _ = self.run_case([(1, 'a')] + [None] * 10 + [(1, 'a')], [False] * 11 + [True])
        self.assertEqual(result, 0)

    def test_master_lost(self):
        n = watch.UNREACHABLE_RESTART_SEC // 2
        result, _, _ = self.run_case([(1, 'a')] + [None] * n, [False] * n)
        self.assertEqual(result, 1)

    def test_master_replaced_after_outage(self):
        result, _, _ = self.run_case([(1, 'a'), None, None, None, (2, 'b')], [False] * 4)
        self.assertEqual(result, 1)

    def test_launch_exits(self):
        result, child, _ = self.run_case([(1, 'a')], [False], poll=1)
        self.assertEqual(result, 1)
        child.send_signal.assert_not_called()

    def test_connection_failure(self):
        with patch.object(watch.xmlrpc.client, 'ServerProxy', side_effect=OSError):
            self.assertIsNone(watch.master_identity('http://fixture:11311'))

    TWO_PATHS = dict(ROS_MASTER_URI='http://lan:11311', ROS_IP='192.0.2.56',
                     ROS_MASTER_URI_ALT='http://tail:11311', ROS_IP_ALT='100.64.0.56')

    def test_falls_back_to_tailscale_and_uses_its_address(self):
        # LAN master silent, Tailscale master answers: launch on the Tailscale pair
        # and keep monitoring that same master.
        result, _, spawn = self.run_case([None, (1, 'a'), (1, 'a')], [False, True], env=self.TWO_PATHS)
        self.assertEqual(result, 0)
        env = spawn.call_args.kwargs['env']
        self.assertEqual((env['ROS_MASTER_URI'], env['ROS_IP']), ('http://tail:11311', '100.64.0.56'))
        self.assertEqual(self.probes, ['http://lan:11311', 'http://tail:11311', 'http://tail:11311'])

    def test_prefers_lan_when_it_answers(self):
        result, _, spawn = self.run_case([(1, 'a')], [True], env=self.TWO_PATHS)
        self.assertEqual(result, 0)
        env = spawn.call_args.kwargs['env']
        self.assertEqual((env['ROS_MASTER_URI'], env['ROS_IP']), ('http://lan:11311', '192.0.2.56'))
        self.assertEqual(self.probes, ['http://lan:11311'])

    def test_keeps_scanning_both_until_one_answers(self):
        result, _, _ = self.run_case([None, None, None, (1, 'a')], [False, True], env=self.TWO_PATHS)
        self.assertEqual(result, 0)
        self.assertEqual(self.probes, ['http://lan:11311', 'http://tail:11311', 'http://lan:11311', 'http://tail:11311'])


if __name__ == '__main__':
    unittest.main()
