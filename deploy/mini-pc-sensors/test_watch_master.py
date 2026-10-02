import importlib.util
import signal
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('watch', Path(__file__).with_name('watch_master.py'))
watch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watch)


class WatchTest(unittest.TestCase):
    def run_case(self, identities, waits, poll=None):
        event = Mock()
        event.is_set.return_value = False
        event.wait.side_effect = waits
        child = Mock()
        child.poll.return_value = poll
        with patch.object(watch.threading, 'Event', return_value=event), \
             patch.object(watch.signal, 'signal'), \
             patch.object(watch, 'master_identity', side_effect=identities), \
             patch.object(watch.subprocess, 'Popen', return_value=child) as spawn, \
             patch.dict(watch.os.environ, ROS_MASTER_URI='http://fixture:11311'):
            result = watch.main()
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

    def test_master_lost(self):
        result, _, _ = self.run_case([(1, 'a'), None, None, None], [False] * 3)
        self.assertEqual(result, 1)

    def test_launch_exits(self):
        result, child, _ = self.run_case([(1, 'a')], [False], poll=1)
        self.assertEqual(result, 1)
        child.send_signal.assert_not_called()

    def test_connection_failure(self):
        with patch.dict(watch.os.environ, ROS_MASTER_URI='http://fixture:11311'), \
             patch.object(watch.xmlrpc.client, 'ServerProxy', side_effect=OSError):
            self.assertIsNone(watch.master_identity())


if __name__ == '__main__':
    unittest.main()
