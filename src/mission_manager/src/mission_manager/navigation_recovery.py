"""Bounded recovery wait; no robot commands and no ROS dependency."""
import time


def wait_for_recovery(fresh, cancelled, clock=time.monotonic, sleep=time.sleep,
                      timeout=30.0, stable_sec=0.5):
    deadline = clock()+timeout
    since = None
    while not cancelled():
        now = clock()
        if now >= deadline:
            return False
        if fresh():
            since = now if since is None else since
            if now-since >= stable_sec:
                return not cancelled()
        else:
            since = None
        sleep(0.05)
    return False
