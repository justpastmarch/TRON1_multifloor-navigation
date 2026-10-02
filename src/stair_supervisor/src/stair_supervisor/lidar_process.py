"""Bounded process adapter for the unchanged tracker; no ROS node or socket.

Large CustomMsg deserialization and Open3D calls stay out of the command owner's
Python interpreter. Only immutable samples and bounded diagnostics return.
"""
from concurrent.futures import Future
import multiprocessing
import queue
import signal
import struct
import threading
import time
import numpy as np

from .lidar_tracking import TrackingWorker


def raw_header_stamp(buffer):
    """Installed CustomMsg starts with Header: uint32 seq, secs, nsecs."""
    _sequence, seconds, nanos = struct.unpack_from("<III", buffer)
    if nanos >= 1000000000:
        raise ValueError("invalid ROS header nanoseconds")
    return seconds + nanos * 1e-9


def points_from_raw_livox(buffer, *, with_offsets=False):
    """Exact float32 XYZ values from the installed 19-byte CustomPoint layout.

    This avoids constructing thousands of Python message objects. No filtering,
    coordinates, timestamp offsets or tracking math change here.
    """
    frame_length = struct.unpack_from("<I", buffer, 12)[0]
    point_num = struct.unpack_from("<I", buffer, 24 + frame_length)[0]
    array_size = struct.unpack_from("<I", buffer, 32 + frame_length)[0]
    offset = 36 + frame_length
    if point_num != array_size or len(buffer) != offset + 19 * array_size:
        raise ValueError("CustomMsg point count or byte length mismatch")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [4, 8, 12], "itemsize": 19})
    points = np.frombuffer(buffer, dtype=dtype, count=array_size, offset=offset)
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(float)
    if with_offsets:
        times = np.ndarray((array_size,), dtype='<u4', buffer=buffer, offset=offset, strides=(19,))
        return np.column_stack((xyz, times.astype(float)*1e-9))
    return xyz


def _latest(queue_, value):
    try:
        queue_.put_nowait(value)
    except queue.Full:
        try:
            queue_.get_nowait()
        except queue.Empty:
            return
        try:
            queue_.put_nowait(value)
        except queue.Full:
            pass


def _process_main(settings, rotation, checksum, scans, imus, commands, states, replies, stopping, ready, initial_epoch):
    # roslaunch sends SIGINT to the process group. Parent owns child cleanup.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    worker = TrackingWorker(settings, rotation, checksum)
    while worker.epoch < initial_epoch:
        worker.reset("compute process restarted; new anchor required")
    publish_lock = threading.Lock()
    def publish():
        with publish_lock:
            _latest(states, (worker.snapshot(), worker.diagnostics()))
    worker.on_sample = lambda _sample: publish()
    def align_epoch(epoch):
        # Independent IPC queues may deliver new data before the reset command.
        # Parent-assigned epochs are authoritative; never drop new-generation
        # IMU merely because that control queue has not been drained yet.
        while worker.epoch < epoch:
            worker.reset("parent epoch advanced; new anchor required")
        return epoch == worker.epoch
    worker.start()
    ready.set()
    last_diagnostic = 0.
    try:
        while not stopping.is_set():
            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command is not None:
                kind, epoch, payload = command
                if kind == "reset":
                    while worker.epoch < epoch:
                        worker.reset(payload)
                elif kind == "entry" and epoch == worker.epoch:
                    job_id, operation = payload
                    try:
                        future = worker.entry_job(operation)
                        def reply(done, identifier=job_id, generation=epoch):
                            try:
                                result = (identifier, generation, done.result(), "")
                            except Exception as error:
                                result = (identifier, generation, None, str(error))
                            _latest(replies, result)
                        future.add_done_callback(reply)
                    except Exception as error:
                        _latest(replies, (job_id, epoch, None, str(error)))
            for _ in range(settings.imu_capacity):
                try:
                    epoch, row = imus.get_nowait()
                except queue.Empty:
                    break
                if align_epoch(epoch):
                    worker.push_imu(row)
            try:
                epoch, stamp, buffer, received, measured = scans.get(timeout=.003)
                if align_epoch(epoch):
                    worker.push_scan(stamp, points_from_raw_livox(buffer,
                        with_offsets=True), received, measured)
            except queue.Empty:
                pass
            now = time.monotonic()
            if now - last_diagnostic > .1:
                publish()
                last_diagnostic = now
    finally:
        worker.shutdown()
        # Parent owns shutdown; never hang child exit flushing obsolete telemetry.
        states.cancel_join_thread()
        replies.cancel_join_thread()


class ProcessTrackingWorker:
    def __init__(self, settings, rotation, calibration_hash="", clock=time.monotonic, on_sample=None):
        self.settings, self.rotation, self.calibration_hash = settings, rotation, calibration_hash
        self.clock, self.on_sample = clock, on_sample
        context = multiprocessing.get_context("spawn")
        self._scans = context.Queue(settings.queue_capacity)
        self._imus = context.Queue(settings.imu_capacity)
        self._commands, self._states, self._replies = context.Queue(8), context.Queue(4), context.Queue(4)
        self._stopping = context.Event()
        self._ready = context.Event()
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._closed = False
        self._epoch, self._sample, self._process, self._reader = 0, None, None, None
        self._diagnostic = dict(queue_length=0, dropped_scans=0, rejected_imu=0, worker_error="")
        self._ingress_drops = self._imu_drops = 0
        self._jobs, self._job_id = {}, 0
        self._last_stamp = None
        self._restarting = False

    @property
    def epoch(self):
        with self._lock:
            return self._epoch

    def snapshot(self):
        with self._lock:
            return self._sample

    def diagnostics(self):
        with self._lock:
            result = dict(self._diagnostic, epoch=self._epoch, backend="process")
            result["queue_length"] += 0 if self._restarting or self._closed else self._scans.qsize()
            result["dropped_scans"] += self._ingress_drops
            result["rejected_imu"] += self._imu_drops
            return result

    def reset(self, reason="explicit reset"):
        with self._lock:
            if self._closed or self._restarting:
                return
            self._epoch += 1
            self._sample = None
            self._last_stamp = None
            self._diagnostic["worker_error"] = reason
            for future in self._jobs.values():
                future.cancel()
            self._jobs.clear()
            try:
                self._commands.put_nowait(("reset", self._epoch, reason))
            except queue.Full:
                self._diagnostic["worker_error"] = "reset command queue full; restart observation"

    def push_imu(self, row):
        with self._lock:
            if self._restarting or self._closed:
                self._imu_drops += 1
                return False
            try:
                self._imus.put_nowait((self._epoch, tuple(row)))
                return True
            except queue.Full:
                self._imu_drops += 1
                return False

    def push_raw_scan(self, stamp, buffer, received_at=None, measured_at=None):
        now = self.clock() if received_at is None else received_at
        with self._lock:
            if self._restarting or self._closed:
                self._ingress_drops += 1
                return False
            if self._last_stamp is not None and stamp <= self._last_stamp:
                if stamp == self._last_stamp:
                    self._ingress_drops += 1
                    return False
                self.reset("LiDAR clock reversed; anchor invalidated")
            self._last_stamp = stamp
            try:
                self._scans.put_nowait((self._epoch, stamp, bytes(buffer), now, now if measured_at is None else measured_at))
                return True
            except queue.Full:
                self._ingress_drops += 1
                return False

    def entry_job(self, operation):
        with self._lock:
            if self._jobs or self._closed or self._restarting or self._stopping.is_set():
                raise ValueError("entry registration already pending or worker stopped")
            self._job_id += 1
            future = Future()
            self._jobs[self._job_id] = future
            try:
                self._commands.put_nowait(("entry", self._epoch, (self._job_id, operation)))
            except queue.Full:
                self._jobs.pop(self._job_id)
                raise ValueError("entry command queue full")
            return future

    def start(self):
        with self._lifecycle:
            if self._closed:
                raise RuntimeError("LiDAR worker has shut down")
            self._start()

    def _start(self):
        if self._process is not None:
            return
        context = multiprocessing.get_context("spawn")
        self._process = context.Process(target=_process_main, name="stair-lidar-worker",
            args=(self.settings, self.rotation, self.calibration_hash, self._scans, self._imus,
                  self._commands, self._states, self._replies, self._stopping, self._ready, self.epoch), daemon=True)
        self._process.start()
        if not self._ready.wait(20.):
            self.shutdown()
            raise RuntimeError("LiDAR worker failed to initialize")
        self._reader = threading.Thread(target=self._read, name="stair-lidar-snapshots", daemon=True)
        self._reader.start()

    def cancel_entry(self, future, reason="entry computation exceeded its deadline"):
        with self._lifecycle:
            if not self._closed:
                self._cancel_entry(future, reason)

    def _cancel_entry(self, future, reason):
        """Stop native work too, then reacquire in a new frame. Never reuse anchor.

        A timed-out native registration cannot be interrupted by Future.cancel.
        Restart only this socket-free child; parent NAV/command thread keeps running.
        """
        with self._lock:
            if future not in self._jobs.values() or self._restarting:
                return
            self._restarting = True
            self._epoch += 1
            self._sample, self._last_stamp = None, None
            self._diagnostic = dict(queue_length=0, dropped_scans=0, rejected_imu=0, worker_error=reason)
            old_process, old_reader = self._process, self._reader
            self._stopping.set()
            for job in self._jobs.values():
                job.cancel()
            self._jobs.clear()
        if old_reader is not None:
            old_reader.join(1.)
        if old_process is not None:
            old_process.terminate()
            old_process.join(2.)
            if old_process.is_alive():
                old_process.kill()
                old_process.join(2.)
        for channel in (self._scans, self._imus, self._commands, self._states, self._replies):
            channel.cancel_join_thread()
            channel.close()
        context = multiprocessing.get_context("spawn")
        with self._lock:
            self._scans, self._imus = context.Queue(self.settings.queue_capacity), context.Queue(self.settings.imu_capacity)
            self._commands, self._states, self._replies = context.Queue(8), context.Queue(4), context.Queue(4)
            self._stopping, self._ready = context.Event(), context.Event()
            self._process = self._reader = None
        try:
            self.start()
        finally:
            with self._lock:
                self._restarting = False

    def _read(self):
        last_key = None
        while not self._stopping.is_set():
            try:
                sample, diagnostic = self._states.get(timeout=.01)
                with self._lock:
                    current = diagnostic["epoch"] == self._epoch
                    if current:
                        self._diagnostic = diagnostic
                        if diagnostic["worker_error"]:
                            self._sample = None
                        elif sample is not None and sample.epoch != self._epoch:
                            self._sample = None
                            self._diagnostic["worker_error"] = "snapshot epoch mismatch"
                        elif self._sample is None or sample is not None and sample.sequence >= self._sample.sequence:
                            self._sample = sample
                key = None if sample is None else (sample.epoch, sample.sequence)
                newer = key is not None and (last_key is None or key > last_key)
                if newer and current and sample.epoch == self.epoch and not diagnostic["worker_error"] and self.on_sample:
                    self.on_sample(sample)
                if newer:
                    last_key = key
            except queue.Empty:
                pass
            try:
                identifier, epoch, result, error = self._replies.get_nowait()
                with self._lock:
                    future = self._jobs.pop(identifier, None)
                    if future is not None and not future.done():
                        if epoch != self._epoch or error:
                            future.set_exception(ValueError(error or "entry epoch invalidated"))
                        else:
                            future.set_result(result)
            except queue.Empty:
                pass
            if not self._process.is_alive():
                with self._lock:
                    self._diagnostic["worker_error"] = "LiDAR worker process exited"
                    self._sample = None
                    for future in self._jobs.values():
                        if not future.done():
                            future.set_exception(ValueError("LiDAR worker process exited"))
                    self._jobs.clear()
                return

    def shutdown(self, timeout=5.):
        with self._lifecycle:
            with self._lock:
                if self._closed:
                    return
                self._closed = True
            self._shutdown(timeout)

    def _shutdown(self, timeout):
        self._stopping.set()
        if self._reader is not None:
            self._reader.join(timeout)
        if self._process is not None:
            self._process.join(timeout)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout)
                self._diagnostic["worker_error"] = "worker exceeded shutdown budget and was terminated"
        for future in self._jobs.values():
            future.cancel()
        self._jobs.clear()
        for channel in (self._scans, self._imus, self._commands, self._states, self._replies):
            channel.cancel_join_thread()
            channel.close()
