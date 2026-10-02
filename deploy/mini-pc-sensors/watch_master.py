#!/usr/bin/env python3
"""Supervise the existing sensor launch under a systemd control group."""
import os
import signal
import subprocess
import threading
import xmlrpc.client


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = 2.0
        return conn


def master_identity():
    try:
        with xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'],
                                      transport=Transport()) as master:
            code, _, pid = master.getPid('/sensor_boot_watch')
            run_code, _, run_id = master.getParam('/sensor_boot_watch', '/run_id')
            if code == 1 and run_code == 1:
                return pid, run_id
    except (OSError, xmlrpc.client.Error):
        pass
    return None


def main():
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    print('Waiting for workstation ROS master: ' + os.environ['ROS_MASTER_URI'], flush=True)
    while not stopping.is_set():
        identity = master_identity()
        if identity is not None:
            break
        stopping.wait(2)
    if stopping.is_set():
        return 0
    print('Master ready; starting sensor_integration wf_mapping.launch', flush=True)
    child = subprocess.Popen(['/opt/ros/noetic/bin/roslaunch', '--wait', '--required',
                              'sensor_integration', 'wf_mapping.launch'])
    failures = 0
    try:
        while not stopping.wait(2):
            if child.poll() is not None:
                print('Sensor launch exited; systemd will retry', flush=True)
                return 1
            current = master_identity()
            failures = failures + 1 if current is None else 0
            if failures >= 3 or (current is not None and current != identity):
                print('Master lost or replaced; restarting sensor registration', flush=True)
                return 1
        return 0
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                # systemd KillMode=control-group owns remaining descendants.
                print('Launch cleanup timed out; systemd will clean the control group', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
