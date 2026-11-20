#!/usr/bin/env python3
"""Supervise the existing sensor launch under a systemd control group."""
import os
import signal
import subprocess
import threading
import xmlrpc.client


UNREACHABLE_RESTART_SEC = 120


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = 2.0
        return conn


def candidates():
    """(master URI, own ROS_IP) pairs in preference order: the robotnet LAN
    first, then the optional Tailscale path (ROS_MASTER_URI_ALT/ROS_IP_ALT),
    then a master on this mini PC (ROS_MASTER_URI_LOCAL/ROS_IP_LOCAL), used
    when the mapping console runs here."""
    pairs = [(os.environ['ROS_MASTER_URI'], os.environ.get('ROS_IP'))]
    for suffix in ('_ALT', '_LOCAL'):
        uri = os.environ.get('ROS_MASTER_URI' + suffix)
        if uri:
            pairs.append((uri, os.environ.get('ROS_IP' + suffix)))
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
    # An unreachable master alone does not restart the sensors: on a phone
    # hotspot the link drops for seconds at a time, and every restart resets
    # wheel odometry. ROS nodes reconnect by themselves once the master
    # answers again. A replaced master (new PID/run_id) still restarts
    # registration, and so does a master gone for UNREACHABLE_RESTART_SEC.
    failures = 0
    try:
        while not stopping.wait(2):
            if child.poll() is not None:
                print('Sensor launch exited; systemd will retry', flush=True)
                return 1
            current = master_identity(uri)
            if current is None:
                failures += 1
                if failures == 3:
                    print('Master unreachable; keeping sensors running', flush=True)
            elif failures:
                print('Master reachable again after %d s' % (failures * 2), flush=True)
                failures = 0
            if failures * 2 >= UNREACHABLE_RESTART_SEC or (current is not None and current != identity):
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
