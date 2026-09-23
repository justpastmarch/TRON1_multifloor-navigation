"""Exercise failure propagation in the existing shell launcher, with no SSH or ROS."""
from pathlib import Path
import os
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StartupShellFailuresTest(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / 'run.sh').read_text()

    def remote_text(self, marker):
        literal = next(line.strip() for line in self.source.splitlines() if marker in line)
        if literal.endswith('\")\"'):
            literal = literal[:-2]
        self.assertTrue(literal.startswith('"') and literal.endswith('"'))
        return subprocess.check_output(
            ['bash', '-c', 'source "$1"; printf "%s" ' + literal, '_', str(ROOT / 'config.env')],
            text=True)

    def test_term_exits_after_cleanup_instead_of_resuming_startup(self):
        traps = '\n'.join(line for line in self.source.splitlines() if line.startswith('trap '))
        script = 'cleanup() { echo CLEANUP; };\n' + traps + '\nkill -TERM "$BASHPID"\necho CONTINUED\n'
        result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 143)
        self.assertEqual(result.stdout.strip(), 'CLEANUP')

    def test_mapping_preflight_failure_cannot_be_hidden_by_camera_success(self):
        remote = self.remote_text('roslaunch --files sensor_integration wf_mapping.launch')
        stubs = 'source() { return 0; }; roslaunch() { echo "$*" >&2; [[ "$*" != *wf_mapping.launch* ]]; };\n'
        result = subprocess.run(['bash', '-c', stubs + remote], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('wf_mapping.launch', result.stderr)
        self.assertNotIn('d435f.launch', result.stderr)

    def test_failed_sensor_tmux_start_does_not_report_restart_requested(self):
        remote = self.remote_text('mapping_status=; sensor_ready=1')
        stubs = '''source() { return 0; }
timeout() { return 1; }
pkill() { return 0; }
sleep() { return 0; }
tmux() { return 7; }
'''
        result = subprocess.run(['bash', '-c', stubs + remote], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('restart requested', result.stdout)

    def test_manual_receiver_root_expands_on_remote_host(self):
        remote = self.remote_text('root=\\"\\${HOME}/.local/share/tron1-sensor-joy')
        assignment = remote.split(';', 1)[0]
        result = subprocess.check_output(['bash', '-c', assignment + '; printf "%s" "$root"'], text=True)
        self.assertEqual(result, str(Path.home() / '.local/share/tron1-sensor-joy'))


if __name__ == '__main__':
    unittest.main()
