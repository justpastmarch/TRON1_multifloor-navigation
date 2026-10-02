"""Optional sensor/diagnostic adapter owned by the existing Supervisor node."""
from __future__ import annotations

from dataclasses import asdict
from collections import deque
from concurrent.futures import TimeoutError as FutureTimeout, CancelledError
from functools import partial
import hashlib
import json
import math
import struct
import time
import threading
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from livox_ros_driver2.msg import CustomMsg

from .lidar_tracking import TrackingSettings, load_imu_rotation
from .lidar_process import ProcessTrackingWorker, raw_header_stamp, points_from_raw_livox
from .stair_feedback import StairFeedback, RouteAnchor, match_entry_landmarks, transform, se2, wrap
from .lidar_tracking_core import register_scan_to_map, voxel_downsample_points
from .configuration import load_stair_configuration
from .stair_evidence import Phase


def json_finite(value):
    # Completion checks contain NumPy scalars after numeric comparisons.
    # Normalize them before encoding status in the ROS timer callback.
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, np.ndarray):
        return json_finite(value.tolist())
    if isinstance(value, dict):
        return {k: json_finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


from .entry_registration import entry_level, refine_upright


def fit_entry_template(local_cloud, sample, route, target, reference, base_from_lidar, registration, deadline=math.inf, imu_context=None):
    start,end=np.asarray(route['flight_1'])
    initial_yaw=math.atan2(end[1]-start[1],end[0]-start[0])
    expected_base=se2(start[0],start[1],initial_yaw);expected_base[2,3]=start[2]
    local_lidar = transform(sample.transform)
    origin = np.eye(4); origin[:3,3] = local_lidar[:3,3]
    # The tracking origin may be metres away after flat-floor NAV. Register
    # around the current sensor, so rotation is not mistaken for a large step.
    centered_cloud = local_cloud - origin[:3,3]
    initial=expected_base@base_from_lidar@np.linalg.inv(local_lidar)@origin
    # Score the transform we actually install, not the unconstrained 6-DoF fit.
    source_voxels = voxel_downsample_points(centered_cloud, registration.voxel_size_m)
    target_voxels = voxel_downsample_points(target, registration.voxel_size_m)
    if not len(source_voxels) or not len(target_voxels):
        raise ValueError("entry point-cloud match missing or ambiguous")
    target_tree = cKDTree(target_voxels)
    candidates=[]
    diagnostics=[]
    level=np.eye(4)
    gravity_valid=False
    if imu_context is not None:
        try:
            level=entry_level(sample,imu_context)
            gravity_valid=True
        except ValueError as error:
            # Original admission remains available; never fabricate gravity.
            diagnostics.append(str(error))
    leveled_source=source_voxels@level[:3,:3].T
    for delta in reference['yaw_candidates']:
        if time.monotonic() >= deadline:
            raise ValueError("entry registration time budget expired")
        pivot=expected_base[:2,3]
        hypothesis=se2(*pivot,float(delta))@se2(*(-pivot),0)@initial
        result=register_scan_to_map(centered_cloud,target,hypothesis,registration)
        if time.monotonic() >= deadline:
            raise ValueError("entry registration time budget expired")
        label = 'yaw=%.1fdeg fit=%.3f rmse=%.3fm' % (math.degrees(delta), result.fitness, result.inlier_rmse_m)
        if not result.accepted or result.fitness<reference['min_fitness'] or result.inlier_rmse_m>reference['max_rmse_m']:
            diagnostics.append(label + ' rejected: registration/fit threshold')
            continue
        fitted=transform(result.transform)
        leveled_fit=fitted@np.linalg.inv(level)
        if gravity_valid and leveled_fit[2,2]<math.cos(math.radians(5.)):
            diagnostics.append(label+' rejected: IMU/geometric gravity disagreement')
            continue
        yaw=math.atan2(leveled_fit[1,0],leveled_fit[0,0])
        # Gravity remains +Z. Small solver tilt is not itself a rejection.
        if leveled_fit[2,2] <= 0.:
            diagnostics.append(label + " rejected: inverted gravity")
            continue
        planar=se2(fitted[0,3],fitted[1,3],yaw);planar[2,3]=fitted[2,3]
        if gravity_valid:
            planar=refine_upright(leveled_source,target_voxels,planar,registration,deadline)
        planar=planar@level
        correction=np.linalg.inv(hypothesis)@planar
        rotation=math.acos(float(np.clip((np.trace(correction[:3,:3])-1)/2,-1,1)))
        if (np.linalg.norm(correction[:3,3])>registration.max_translation_step_m or
                rotation>registration.max_rotation_step_rad):
            diagnostics.append(label+' rejected: installed transform innovation')
            continue
        projected = source_voxels @ planar[:3,:3].T + planar[:3,3]
        distances = target_tree.query(projected)[0]
        inliers = distances < registration.correspondence_distance_m
        fitness = float(inliers.mean())
        rmse = float(np.sqrt(np.mean(distances[inliers]**2))) if inliers.any() else math.inf
        if time.monotonic() >= deadline:
            raise ValueError("entry registration time budget expired")
        if fitness < reference['min_fitness'] or rmse > reference['max_rmse_m']:
            diagnostics.append(label + ' rejected: planar fit=%.3f rmse=%.3fm' % (fitness, rmse))
            continue
        score=fitness-rmse
        candidates.append((score,planar))
    candidates.sort(key=lambda x:x[0],reverse=True)
    distinct=[]
    for score, planar in candidates:
        yaw=math.atan2(planar[1,0],planar[0,0])
        if all(np.linalg.norm(planar[:3,3]-p[:3,3])>.03 or
               abs(wrap(yaw-math.atan2(p[1,0],p[0,0])))>.03 for _,p in distinct):
            distinct.append((score,planar))
    candidates=distinct
    if not candidates:
        raise ValueError("entry point-cloud match missing or ambiguous: no acceptable fit "
                         "(max_rmse=%.3fm); %s" % (reference['max_rmse_m'], '; '.join(diagnostics)))
    if len(candidates)>1 and candidates[0][0]-candidates[1][0]<reference['score_gap']:
        raise ValueError("entry point-cloud match missing or ambiguous: distinct pose score gap %.4f < %.4f" %
                         (candidates[0][0]-candidates[1][0], reference['score_gap']))
    # Independent entry/mount error bound, NOT GICP fit residual.
    uncertainty=reference['validated_anchor_error_m']
    if uncertainty>route['limits']['anchor_uncertainty_m']:
        raise ValueError("entry uncertainty exceeds route budget")
    return RouteAnchor(sample.epoch,sample.sequence,sample.measured_at,
        tuple((origin@np.linalg.inv(candidates[0][1])).flat),uncertainty,'surveyed cloud '+reference['sha256'])


def wait_for_entry_observation(worker, anchor, warn_sec, clock=time.monotonic,
                               sleep=time.sleep, timeout_sec=1.0):
    """Registration occupies the tracking worker; await a newer real sample.

    Never renew a measurement timestamp or reuse a sample across resets.
    This bounded wait sends no commands and leaves tracking free to catch up.
    """
    deadline = clock() + timeout_sec
    while True:
        sample = worker.snapshot()
        now = clock()
        if worker.epoch != anchor.epoch:
            raise ValueError("entry epoch changed while awaiting fresh observation")
        if now >= deadline:
            raise ValueError("entry fresh observation unavailable after registration (%.1fs wait)" % timeout_sec)
        if (sample is not None and sample.epoch == anchor.epoch and
                sample.sequence > anchor.sequence and sample.geometry_valid and
                0 <= now - sample.measured_at <= max(0., warn_sec - .1)):
            return sample
        sleep(min(.02, deadline - now))


class RosLidarInterface:
    def __init__(self, configuration, clock):
        self.configuration, self.clock = configuration, clock
        document = configuration.document
        calibration = Path(document["calibration"])
        if not calibration.is_absolute():
            calibration = configuration.root / calibration
        rotation, checksum = load_imu_rotation(calibration)
        self.odom = rospy.Publisher("~lidar_odom", Odometry, queue_size=2)
        self.status = rospy.Publisher("~tracking_status", String, queue_size=2)
        self.control_debug = rospy.Publisher("~control_debug", String, queue_size=2)
        self.markers = rospy.Publisher("~geometry_markers", MarkerArray, queue_size=1)
        self.cloud = rospy.Publisher("~aligned_cloud", PointCloud2, queue_size=1)
        self._raw_scans, self._raw_lock = deque(maxlen=32), threading.Lock()
        self.worker = ProcessTrackingWorker(TrackingSettings(**document["tracking"]), rotation, checksum, clock.monotonic)
        self.control = None
        if document["base_from_lidar"] is not None:
            self.control = StairFeedback(self.worker, document["base_from_lidar"], document["routes"],
                lidar_from_imu=rotation, imu_max_gap=self.worker.settings.imu_max_gap_s)
        if configuration.mode == "control" and self.control is None:
            raise ValueError("control needs surveyed base_from_lidar")
        self._published = None
        self._last_clock_offset = None
        self._closed = False
        self._observing = False
        self.worker.start()
        self._subscribers = [
            rospy.Subscriber(rospy.get_param("~lidar_topic", "/livox/lidar"), rospy.AnyMsg, self._scan, queue_size=2, buff_size=8*1024*1024),
            rospy.Subscriber(rospy.get_param("~imu_topic", "/livox/imu"), Imu, self._imu, queue_size=200),
        ]
        self._entry = rospy.Service("~capture_entry", Trigger, self._capture_entry)
        self._observe = rospy.Service("~start_observation", Trigger, self._start_observation)
        self._timer = rospy.Timer(rospy.Duration(.1), self._publish)

    def ensure_entry(self, route_id):
        """Automatically fit a surveyed entry template when one is commissioned."""
        if self.control is None or route_id not in self.control.routes:
            raise ValueError("no LiDAR route for requested profile")
        route=self.control.routes[route_id]
        if route.get('direction')=='DOWN' and 'entry_map' in route:
            return self._ensure_map_entry(route_id,route)
        reference=route.get("entry_reference")
        if reference is None:
            return  # first commissioning may supply the explicit capture service
        required={"path","sha256","yaw_candidates","min_fitness","max_rmse_m","score_gap","validated_anchor_error_m","timeout_sec","unique_geometry_verified"}
        if set(reference)!=required or reference['unique_geometry_verified'] is not True:
            raise ValueError("entry reference needs surveyed, verified unique geometry")
        for key in ('min_fitness','max_rmse_m','score_gap','validated_anchor_error_m','timeout_sec'):
            if type(reference[key]) not in (float,int) or not math.isfinite(reference[key]) or reference[key]<=0:
                raise ValueError("invalid entry reference limit: "+key)
        if reference['min_fitness']>1 or not reference['yaw_candidates'] or any(not math.isfinite(v) for v in reference['yaw_candidates']):
            raise ValueError("invalid entry hypotheses or fitness")
        path=self.configuration.root/reference['path']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=reference['sha256']:
            raise ValueError("entry template fingerprint changed")
        target=np.load(str(path),allow_pickle=False)
        if target.ndim!=2 or target.shape[1]!=3 or len(target)<3 or not np.isfinite(target).all():
            raise ValueError("entry template must be a finite surveyed Nx3 array")
        imu_context=self.control.entry_imu_context() if hasattr(self.control,'entry_imu_context') else None
        operation = partial(fit_entry_template, route=route, target=target, reference=reference,
                            imu_context=imu_context,
                            base_from_lidar=self.control.base_from_lidar,
                            registration=self.worker.settings.registration,
                            deadline=self.clock.monotonic()+reference['timeout_sec'])
        future=self.worker.entry_job(operation)
        try:
            anchor=future.result(reference['timeout_sec'])
        except FutureTimeout:
            self.worker.cancel_entry(future)
            raise ValueError("entry registration exceeded its time budget")
        except CancelledError:
            raise ValueError("entry registration invalidated by reset/shutdown")
        wait_for_entry_observation(self.worker, anchor, route['limits']['warn_sec'],
                                   clock=self.clock.monotonic)
        self.control.install_anchor(route_id,anchor)

    def _ensure_map_entry(self, route_id, route):
        import rospkg,yaml
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from multifloor_manager.msg import FloorState
        from std_srvs.srv import Empty
        from .map_entry import map_entry_anchor
        reference=route['entry_map']
        path=Path(rospkg.RosPack().get_path(reference['package']))/reference['locations']
        document=yaml.safe_load(path.read_text())
        location=next((x for x in document['locations'] if x['id']==reference['location_id']),None)
        if location is None:raise ValueError('RF map stair endpoint is missing')
        records=[]
        subscription=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,records.append,queue_size=1)
        try:
            # Existing service is bounded; no navigation or robot command.
            from multifloor_manager.ros_services import BoundedServiceCaller, ServiceCallError
            BoundedServiceCaller(2.).call('/request_nomotion_update',rospy.ServiceProxy('/request_nomotion_update',Empty))
            deadline=self.clock.monotonic()+2.
            while not records and self.clock.monotonic()<deadline:time.sleep(.02)
            if not records:raise ValueError('RF entry has no refreshed AMCL pose')
            floor=rospy.wait_for_message('/multifloor/floor_state',FloorState,timeout=2.)
            if floor.state!=FloorState.READY:raise ValueError('RF floor is not ready')
            message=records[-1];q=message.pose.pose.orientation;p=message.pose.pose.position
            if (message.header.frame_id.lstrip('/')!='map' or not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
                raise ValueError('RF entry pose frame/orientation invalid')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            sample=self.worker.snapshot()
            if sample is None:raise ValueError('RF entry has no LiDAR observation')
            anchor=map_entry_anchor(route,location,(p.x,p.y,yaw),
                [message.pose.covariance[i] for i in (0,7,35)],sample,self.control.base_from_lidar,
                floor.floor_id,self.clock.monotonic(),rospy.Time.now().to_sec(),message.header.stamp.to_sec())
            self.control.install_anchor(route_id,anchor)
        except (rospy.ROSException,StopIteration,ServiceCallError) as error:
            raise ValueError('RF map entry observation failed: '+str(error)) from error
        finally:subscription.unregister()

    def _start_observation(self, _request):
        """Replay measured phases and proposed commands without a transport."""
        try:
            if self.configuration.mode != "observe" or self.control is None:
                raise ValueError("observation requires observe mode and surveyed geometry")
            route_id = rospy.get_param("~observe_route_id")
            self.ensure_entry(route_id)
            root = Path(rospy.get_param("~config_dir", str(Path(__file__).resolve().parents[2] / "config")))
            profiles = load_stair_configuration(root).profiles
            profile = next((p for p in profiles if p.id == route_id and p.enabled), None)
            if profile is None:
                raise ValueError("observation profile unavailable")
            now = self.clock.monotonic()
            # Observation emits no robot commands and must be available before
            # commissioning. Start geometry/anchor validity still applies.
            self.control.arm(profile, now, phase_test=True, start_phase=Phase.VERIFY_ENTRY)
            self.control.begin_phase(Phase.VERIFY_ENTRY, now)
            self._observing = True
            return TriggerResponse(success=True, message="observing measured phases; no commands transmitted")
        except (ValueError, KeyError, OSError) as error:
            return TriggerResponse(success=False, message=str(error))

    def _imu(self, message):
        w, a = message.angular_velocity, message.linear_acceleration
        row = (message.header.stamp.to_sec(), w.x, w.y, w.z, a.x, a.y, a.z)
        self.worker.push_imu(row)
        if self.control is not None:
            self.control.push_imu(row)

    def _scan(self, message):
        header = getattr(message, "_connection_header", None)
        if header is not None and header.get("md5sum") != CustomMsg._md5sum:
            self.worker.reset("LiDAR message type does not match installed CustomMsg layout")
            return
        now = self.clock.monotonic()
        ros_now = rospy.Time.now().to_sec()
        offset = ros_now - now
        if self._last_clock_offset is not None and abs(offset - self._last_clock_offset) > .25:
            self.worker.reset("ROS/monotonic clock offset jumped")
        self._last_clock_offset = offset
        try:
            stamp = raw_header_stamp(message._buff)
        except (ValueError, TypeError, struct.error):
            self.worker.reset("invalid raw LiDAR header")
            return
        measured_at = now - (ros_now - stamp)
        if measured_at > now + .01:
            self.worker.reset("future-dated LiDAR input")
            return
        with self._raw_lock:
            self._raw_scans.append((self.worker.epoch, stamp, message._buff))
        self.worker.push_raw_scan(stamp, message._buff, now, measured_at)

    def _publish_cloud(self, sample):
        """Display only: exact matching scan/epoch transformed by its measured pose."""
        if not hasattr(self, '_raw_scans'):
            return
        with self._raw_lock:
            buffer = next((raw for epoch, stamp, raw in reversed(self._raw_scans)
                           if epoch == sample.epoch and stamp == sample.stamp), None)
        if buffer is None:
            return  # Never attach the newest cloud to an older pose.
        try:
            points = (np.asarray(sample.display_points).reshape(-1, 3)
                      if sample.display_points is not None else points_from_raw_livox(buffer))
        except (ValueError, TypeError, struct.error):
            return  # An optional display failure must not interrupt diagnostics.
        distance = np.linalg.norm(points, axis=1)
        points = points[np.isfinite(points).all(axis=1) & (distance > .25) & (distance < 20.)]
        points = points[::max(1, (len(points)+4999)//5000)]
        pose = transform(sample.transform)
        points = points @ pose[:3, :3].T + pose[:3, 3]
        header = Header(stamp=rospy.Time.from_sec(sample.stamp), frame_id='stair_local_%d' % sample.epoch)
        self.cloud.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _capture_entry(self, _request):
        try:
            if self.control is None:
                raise ValueError("body transform/routes not commissioned")
            evidence = rospy.get_param("~entry_evidence")
            sample = self.worker.snapshot()
            route_id = evidence["route_id"]
            limits = self.control.routes[route_id]["limits"]
            if sample is None or evidence["epoch"] != sample.epoch or abs(evidence["sensor_stamp"] - sample.stamp) > limits["warn_sec"]:
                raise ValueError("entry evidence does not correspond to current scan/epoch")
            profile_base = evidence.get("profile_from_base")
            if "profile_from_local" in evidence:
                if profile_base is not None:
                    raise ValueError("provide only one entry frame transform")
                # The reference frame is fixed within this epoch. Compose it
                # with the current immutable sample, even if a newer scan arrived
                # after the client observed the explicitly placed entry position.
                profile_local = transform(evidence["profile_from_local"])
                if not np.allclose(profile_local[2, :3], [0., 0., 1.], atol=1e-6):
                    raise ValueError("entry frame must preserve measured gravity")
                profile_base = profile_local @ transform(sample.transform) @ np.linalg.inv(self.control.base_from_lidar)
            elif profile_base is None:
                profile_local, residual = match_entry_landmarks(evidence["observed_local_xy"], evidence["profile_candidates"],
                    limits["anchor_uncertainty_m"], evidence["ambiguity_margin_m"])
                profile_base = profile_local @ transform(sample.transform) @ np.linalg.inv(self.control.base_from_lidar)
                if evidence["uncertainty_m"] < residual:
                    raise ValueError("declared uncertainty below landmark residual")
            anchor = self.control.capture_entry(route_id, profile_base, evidence["uncertainty_m"], evidence["source"], self.clock.monotonic(), sample=sample)
            return TriggerResponse(success=True, message="entry anchor captured epoch=%d sequence=%d" % (anchor.epoch, anchor.sequence))
        except (ValueError, KeyError, TypeError) as error:
            return TriggerResponse(success=False, message=str(error))

    def _publish(self, _event):
        sample = self.worker.snapshot()
        diagnostic = self.worker.diagnostics()
        diagnostic.update(mode=self.configuration.mode, observe_only=self.configuration.observe_only)
        diagnostic['phase_test_protocol'] = 'bound-ticket-v1'
        diagnostic.update(config_path=self.configuration.source_path,
                          config_sha256=self.configuration.source_sha256,
                          routes=[dict(id=r['id'], commissioned=r['commissioned'])
                                  for r in self.configuration.document.get('routes', [])])
        if sample is not None:
            diagnostic.update(asdict(sample))
            diagnostic.pop("transform", None)
            diagnostic.pop("display_points", None)
            diagnostic["geometry_age_sec"] = None if sample.last_geometry_at is None else self.clock.monotonic() - sample.last_geometry_at
            diagnostic["transport_delay_sec"] = sample.received_at - sample.measured_at
            diagnostic["completion_age_sec"] = sample.completed_at - sample.measured_at
            diagnostic['input_age_after_scan_end_sec'] = sample.received_at-sample.measured_at-sample.scan_span_sec
            diagnostic['scan_span_known'] = sample.scan_span_sec > 0.
            if sample.processing_started_at is not None:
                diagnostic['callback_to_processing_sec'] = sample.processing_started_at-sample.received_at
                diagnostic['processing_to_completion_sec'] = sample.completed_at-sample.processing_started_at
                diagnostic['completion_to_publish_sec'] = self.clock.monotonic()-sample.completed_at
            diagnostic['timestamp_reference'] = 'scan_start'
            key = (sample.epoch, sample.sequence)
            if sample.geometry_valid and key != self._published and sample.epoch == self.worker.epoch:
                matrix = transform(sample.transform)
                message = Odometry()
                message.header.stamp = rospy.Time.from_sec(sample.stamp)
                message.header.seq = sample.sequence
                message.header.frame_id = "stair_local_%d" % sample.epoch
                message.child_frame_id = "stair_lidar_sensor"
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = matrix[:3, 3]
                q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
                message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w = q
                # Display-only large positive placeholder, NOT a calibrated
                # uncertainty model. Never feed this diagnostic into fusion.
                message.pose.covariance = [1e6 if i % 7 == 0 else 0. for i in range(36)]
                self.odom.publish(message)
                self._publish_cloud(sample)
                self._published = key
        if self.control is not None:
            diagnostic['control_policy'] = self.control.policy_version
        self.status.publish(String(data=json.dumps(json_finite(diagnostic), allow_nan=False)))
        if self.control is not None:
            if self._observing:
                phase = self.control.phase
                report = self.control.evaluate(phase, self.clock.monotonic())
                if report.faulted:
                    self._observing = False
                elif report.complete:
                    phases = self.control.traversal_phases(self.control.profile)
                    index = phases.index(phase) + 1
                    if index == len(phases):
                        self._observing = False
                    else:
                        self.control.begin_phase(phases[index], self.clock.monotonic())
            debug = self.control.diagnostic_snapshot()
            with self.control._lock:
                debug['pending_entries'] = [dict(route_id=key, epoch=a.epoch,
                    sequence=a.sequence, source=a.source,
                    age_sec=self.clock.monotonic()-a.measured_at)
                    for key, a in self.control.anchors.items() if a.epoch == self.worker.epoch]
            debug.update(observation_only=self.configuration.mode == "observe")
            debug.update(config_path=self.configuration.source_path, config_sha256=self.configuration.source_sha256)
            self.control_debug.publish(String(data=json.dumps(json_finite(debug), allow_nan=False)))
            self._publish_geometry(sample)

    def _publish_geometry(self, sample):
        if sample is None or sample.transform is None:
            return
        control = self.control
        with control._lock:
            pending = [(k, a) for k, a in control.anchors.items() if a.epoch == sample.epoch]
            preview = bool(pending) and (control.anchor is None or control._interrupted)
            if preview:
                key, anchor = max(pending, key=lambda pair: pair[1].measured_at)
                route, phase = control.routes[key], Phase.VERIFY_ENTRY
            else:
                route, anchor, phase = control.route, control.anchor, control.phase
            if anchor is None or sample.epoch != anchor.epoch:
                return
            local_profile = transform(anchor.local_from_profile)
            body = transform(sample.transform) @ np.linalg.inv(control.base_from_lidar)
            fresh = sample.geometry_valid and self.clock.monotonic() - sample.measured_at <= route["limits"]["warn_sec"]
            array = MarkerArray()
            def line(name, points, matrix, color, closed=False):
                marker = Marker()
                marker.header.frame_id = "stair_local_%d" % sample.epoch
                marker.header.stamp = rospy.Time.from_sec(sample.stamp)
                marker.ns, marker.id, marker.type, marker.action = name, 0, Marker.LINE_STRIP, Marker.ADD
                marker.pose.orientation.w = 1.
                marker.scale.x = .025
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, .9)
                marker.lifetime = rospy.Duration(.5)
                rows = np.asarray(points, dtype=float)
                if rows.shape[1] == 2:
                    rows = np.column_stack((rows, np.zeros(len(rows))))
                rows = (matrix @ np.column_stack((rows, np.ones(len(rows)))).T).T[:, :3]
                if closed:
                    rows = np.vstack((rows, rows[:1]))
                marker.points = [Point(*map(float, row)) for row in rows]
                array.markers.append(marker)
            for key in ("flight_1", "flight_2"):
                line(key, route[key], local_profile, (0., .8, 1.))
            if not preview and "target" in control.debug:
                line("current_target", [control.debug["pose"], control.debug["target"][:3]],
                     local_profile, (1., 0., 1.))
            for key, height in (("entry_polygon", route["flight_1"][0][2]),
                                ("landing_polygon", route["flight_1"][1][2]),
                                ("exit_polygon", route["flight_2"][1][2])):
                points = [[*xy, height] for xy in route[key]]
                line(key, points, local_profile, (1., .7, 0.), True)
            line("body", route["footprint"], body, (0., 1., 0.) if fresh else (1., 0., 0.), True)
            text = Marker()
            text.header.frame_id = "stair_local_%d" % sample.epoch
            text.header.stamp = rospy.Time.from_sec(sample.stamp)
            text.ns, text.id, text.type, text.action = "supervisor_phase", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position = Point(*map(float, body[:3, 3] + [0, 0, .5]))
            text.pose.orientation.w = 1.
            text.scale.z, text.color.a = .16, 1.
            text.color.r, text.color.g = (0., 1.) if fresh else (1., 0.)
            debug = {} if preview else control.diagnostic_snapshot()
            sent = debug.get('last_sent_twist')
            fault = debug.get('first_fault')
            text.text = "%s | %s | epoch %d\nage=%.2fs margin=%.3fm clearance=%s\nrequested v/w=%s\nlast TX xyz=%s\nwaiting=%s\nfirst fault=%s" % (
                "ENTRY PREVIEW / NO TEST COMMAND" if preview else phase.value,
                debug.get("tracking_state", sample.state), sample.epoch,
                self.clock.monotonic()-sample.measured_at,
                route['limits']['margin_m']+anchor.uncertainty_m,
                str(debug.get('clearance_m')),
                str([round(v,3) for v in debug.get('command', [0.,0.])]),
                'unknown' if sent is None else '%s (%.2fs ago)' % (
                    [round(v,3) for v in sent['normalized_xyz']],self.clock.monotonic()-sent['sent_at']),
                'second-flight entry alignment' if debug.get('second_flight_alignment') else ','.join(debug.get('incomplete_conditions', [])),
                'none' if fault is None else fault['reason'])
            text.lifetime = rospy.Duration(.5)
            array.markers.append(text)
        self.markers.publish(array)

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self._timer.shutdown()
        for subscriber in self._subscribers:
            subscriber.unregister()
        self._entry.shutdown()
        self._observe.shutdown()
        self.worker.shutdown()
