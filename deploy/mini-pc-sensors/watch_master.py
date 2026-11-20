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


def candidates():
    """(master URI, own ROS_IP) pairs in preference order: the robotnet LAN
    first, then the optional Tailscale path (ROS_MASTER_URI_ALT/ROS_IP_ALT)."""
    pairs = [(os.environ['ROS_MASTER_URI'], os.environ.get('ROS_IP'))]
    alt = os.environ.get('ROS_MASTER_URI_ALT')
    if alt:
        pairs.append((alt, os.environ.get('ROS_IP_ALT')))
    return pairs


def master_identity(uri):
    try:
        with xmlrpc.client.ServerProxy(uri,
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
    pairs = candidates()
    print('Waiting for workstation ROS master: ' + ' or '.join(uri for uri, _ in pairs), flush=True)
    identity = None
    while not stopping.is_set():
        for uri, ros_ip in pairs:
            identity = master_identity(uri)
            if identity is not None:
                break
        if identity is not None:
            break
        stopping.wait(2)
    if stopping.is_set():
        return 0
    env = dict(os.environ, ROS_MASTER_URI=uri)
    if ros_ip:
        env['ROS_IP'] = ros_ip
    print('Master ready at %s (ROS_IP %s); starting sensor_integration wf_mapping.launch'
          % (uri, env.get('ROS_IP')), flush=True)
    child = subprocess.Popen(['/opt/ros/noetic/bin/roslaunch', '--wait', '--required',
                              'sensor_integration', 'wf_mapping.launch'], env=env)
    failures = 0
    try:
        while not stopping.wait(2):
            if child.poll() is not None:
                print('Sensor launch exited; systemd will retry', flush=True)
                return 1
            current = master_identity(uri)
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
