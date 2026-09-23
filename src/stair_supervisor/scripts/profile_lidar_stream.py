#!/usr/bin/env python3
"""One-timespeed bag/worker + 40 Hz memory-only command path load measurement.

No ROS node, RobotTransport, WebSocket or velocity publisher is constructed.
This is a component load test, not full-graph or closed-loop robot validation.
"""
import argparse
import csv
from scipy.spatial.transform import Rotation
import json
import multiprocessing
import queue
from pathlib import Path
import resource
import threading
import time

import numpy as np
import rosbag
import rospy

from stair_supervisor.configuration import load_stair_configuration
from stair_supervisor.lidar_tracking import TrackingSettings, load_imu_rotation
from stair_supervisor.lidar_process import ProcessTrackingWorker, raw_header_stamp
from stair_supervisor.robot_transport import SystemClock
from stair_supervisor.supervisor import StairSupervisor


class MemoryTransport:
    stream_period_sec = .025

    def __init__(self):
        self.sent = []

    def start(self):
        pass

    def update_twist(self, linear, angular):
        self.command = (linear, angular)

    def send_current(self):
        self.sent.append(time.monotonic())

    def close(self):
        pass


def bag_source(spec, packets, ready, running, stop):
    """The recorded sensor source is external to the command owner, as on robot."""
    packets.cancel_join_thread()
    with rosbag.Bag(spec["bag_path"], "r") as bag:
        first = bag.get_start_time() + spec["start_offset_s"]
        end = first + spec["duration_s"]
        bag_start = bag.get_start_time()  # preserve all arrived IMU history before selected first scan
        imu_topic, lidar_topic = spec.get("imu_topic", "/livox/imu"), spec.get("lidar_topic", "/livox/lidar")
        iterator = bag.read_messages(topics=[imu_topic, lidar_topic], start_time=rospy.Time.from_sec(bag_start),
                                    end_time=rospy.Time.from_sec(end + .6), raw=True)
        ready.set()
        running.wait(20.)
        wall_start = time.monotonic()
        for topic, raw, recorded in iterator:
            if stop.is_set():
                return
            delay = wall_start + recorded.to_sec() - bag_start - time.monotonic()
            if delay > 0:
                stop.wait(delay)
            stamp = raw_header_stamp(raw[1])
            if topic == imu_topic:
                message = raw[4]().deserialize(raw[1])
                w, a = message.angular_velocity, message.linear_acceleration
                packet = ("imu", (stamp, w.x, w.y, w.z, a.x, a.y, a.z))
            elif first <= recorded.to_sec() <= end:
                packet = ("lidar", (stamp, raw[1], time.monotonic(), wall_start + stamp - bag_start, recorded.to_sec()-stamp))
            else:
                continue
            packets.put(packet, timeout=2.)
        packets.put(("done", None), timeout=2.)


def profile(spec_path, output):
    spec = json.loads(Path(spec_path).read_text())
    rotation, checksum = load_imu_rotation(spec["imu_calibration_path"])
    samples, queue_lengths = [], []
    worker = ProcessTrackingWorker(TrackingSettings(), rotation, checksum, on_sample=samples.append)
    transport = MemoryTransport()
    config = load_stair_configuration(Path(__file__).resolve().parents[1] / "config")
    supervisor = StairSupervisor(config, transport, None, SystemClock(), lambda _r: None, None, .25)
    stop = threading.Event()

    def commands():
        deadline = time.monotonic()
        while not stop.is_set():
            supervisor.accept_navigation(0., 0.)
            supervisor.stream_navigation()
            worker.snapshot()
            queue_lengths.append(worker.diagnostics()["queue_length"])
            deadline += .025
            stop.wait(max(0., deadline - time.monotonic()))

    started = time.monotonic()
    usage = resource.getrusage(resource.RUSAGE_SELF)
    supervisor.start()
    worker.start()
    sender = threading.Thread(target=commands, name="memory-command-timer")
    context = multiprocessing.get_context("spawn")
    packets, ready, running, source_stop = context.Queue(256), context.Event(), context.Event(), context.Event()
    producer = context.Process(target=bag_source, args=(spec, packets, ready, running, source_stop), daemon=True)
    producer.start()
    if not ready.wait(20.):
        producer.terminate();producer.join();worker.shutdown()
        raise RuntimeError("recorded sensor source failed to initialize")
    stamp_offsets = []
    sender.start()
    running.set()
    try:
        while True:
            kind, payload = packets.get(timeout=5.)
            if kind == "done":
                break
            if kind == "imu":
                worker.push_imu(payload)
            else:
                stamp, buffer, received, measured, offset = payload
                stamp_offsets.append(offset)
                worker.push_raw_scan(stamp, buffer, received, measured)
        deadline = time.monotonic() + 3.
        while worker.diagnostics()["queue_length"] and time.monotonic() < deadline:
            stop.wait(.02)
    finally:
        stop.set()
        sender.join(2.)
        diagnostic = worker.diagnostics()
        worker.shutdown()
        source_stop.set()
        producer.join(3.)
        if producer.is_alive():
            producer.terminate();producer.join(3.)
        packets.cancel_join_thread();packets.close()
        command_stamps = transport.sent[:]
        supervisor.shutdown()
    elapsed = time.monotonic() - started
    after = resource.getrusage(resource.RUSAGE_SELF)
    geometry = [s for s in samples if s.geometry_valid]
    intervals = np.diff(command_stamps)
    metrics = dict(scope="component-only 1x recorded arrival + real worker + memory-only 40 Hz Supervisor NAV path",
        full_graph_test=False, external_recorded_source=True, robot_commands_sent=0, bag=spec["bag_path"], calibration_sha256=checksum,
        recorded_minus_header_p50_sec=float(np.percentile(stamp_offsets, 50)),
        recorded_minus_header_p99_sec=float(np.percentile(stamp_offsets, 99)),
        elapsed_sec=elapsed, processed_frames=len(samples), geometry_frames=len(geometry),
        bootstrap_frames=sum(s.state == "BOOTSTRAP" for s in samples),
        rejected_frames=sum(s.state == "DEGRADED" for s in samples),
        queue_max=max(queue_lengths), worker=diagnostic,
        compute_p99_sec=float(np.percentile([s.compute_sec for s in geometry], 99)),
        completion_age_p99_sec=float(np.percentile([s.completed_at-s.measured_at for s in geometry], 99)),
        command_count=len(command_stamps), command_interval_p99_sec=float(np.percentile(intervals, 99)),
        command_interval_max_sec=float(np.max(intervals)),
        cpu_seconds=(after.ru_utime+after.ru_stime)-(usage.ru_utime+usage.ru_stime), peak_rss_kib=after.ru_maxrss)
    reference_path = Path(spec_path).parent / "lidar_registration.csv"
    if reference_path.is_file():
        reference = list(csv.DictReader(reference_path.open()))
        rows = [s for s in samples if s.transform is not None]
        position_error, rotation_error, mismatches = [], [], abs(len(reference)-len(rows))
        for observed, expected in zip(rows, reference):
            valid = observed.geometry_valid or observed.state == "BOOTSTRAP"
            if (abs(observed.stamp-float(expected["stamp_ns"])*1e-9) > 1e-6 or
                    observed.state != expected["state"] or valid != (expected["accepted"].lower() in ("true", "1"))):
                mismatches += 1
            actual = np.asarray(observed.transform).reshape(4,4)
            target = np.array([float(expected[k]) for k in ("x_m", "y_m", "z_m")])
            quaternion = [float(expected[k]) for k in ("qx", "qy", "qz", "qw")]
            position_error.append(float(np.linalg.norm(actual[:3,3]-target)))
            rotation_error.append(float(np.degrees(Rotation.from_matrix(Rotation.from_quat(quaternion).as_matrix().T @ actual[:3,:3]).magnitude())))
        metrics["numerical_equivalence"] = dict(reference=str(reference_path), state_stamp_mismatches=mismatches,
            max_translation_m=max(position_error), max_rotation_deg=max(rotation_error),
            pass_criteria="<=1 mm / <=0.05 deg, identical frame/state acceptance",
            passed=mismatches==0 and max(position_error)<.001 and max(rotation_error)<.05)
    Path(output).with_suffix(".poses.json").write_text(json.dumps([dict(stamp=s.stamp, state=s.state,
        epoch=s.epoch, pose=s.transform) for s in samples]))
    metrics["component_timing_pass"] = (metrics["compute_p99_sec"] < .2 and
        metrics["command_interval_p99_sec"] <= .0333 and metrics["command_interval_max_sec"] <= .05 and
        metrics["worker"]["dropped_scans"] == 0 and not metrics["worker"]["worker_error"])
    Path(output).write_text(json.dumps(metrics, indent=2)+"\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec")
    parser.add_argument("output")
    arguments = parser.parse_args()
    profile(arguments.spec, arguments.output)
