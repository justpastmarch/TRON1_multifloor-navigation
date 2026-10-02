"""Optional idle position restoration through the existing NavigationExecutor.

No velocity publisher, new node or second move_base client. A new mission first
cancels and joins the correction before it may use the shared executor.
"""
from __future__ import annotations

import math
import threading
import time

from .navigation_executor import NavigationRequest, NavigationOutcome


class NavigationHold:
    def __init__(self, navigation, pose_snapshot, r_on, r_off, yaw_on, yaw_off,
                 correction_timeout, cancel_timeout, clock=time.monotonic):
        values=(r_on,r_off,yaw_on,yaw_off,correction_timeout,cancel_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values) or not (r_off < r_on and yaw_off < yaw_on):
            raise ValueError("hold needs positive, ordered commissioning thresholds")
        self.navigation, self.pose_snapshot = navigation, pose_snapshot
        self.r_on,self.r_off,self.yaw_on,self.yaw_off = r_on,r_off,yaw_on,yaw_off
        self.correction_timeout,self.cancel_timeout,self.clock = correction_timeout,cancel_timeout,clock
        self._lock=threading.RLock()
        self._target=None
        self._generation=None
        self._epoch=0
        self._thread=None
        self._started=0.
        self._cancel_sent=False
        self._request=None
        self.detail="disabled until a successful mission arms a target"

    def arm(self, location, generation, takeover=None):
        self.suspend()
        with self._lock:
            self._target,self._generation=location,generation
            epoch=self._epoch
            try:
                # Timers cannot start correction or invalidate ownership between
                # activation and the Supervisor's acknowledged release.
                if takeover is not None:
                    takeover()
                if self._epoch != epoch or self._target is not location:
                    raise RuntimeError("arrival ownership changed during handoff")
            except Exception:
                self._target=None;self._epoch+=1
                raise
            self.detail="watching confirmed arrival position"

    @property
    def armed(self):
        with self._lock:
            return self._target is not None

    def fail_localization(self, epoch, detail):
        with self._lock:
            if epoch != self._epoch or self._target is None:
                return
            self._target=None;self._epoch+=1
            request=self._request
            active=self._thread is not None and self._thread.is_alive()
            self.detail=detail
        if active:self.navigation.request_cancel_for(request)

    def suspend(self):
        with self._lock:
            self._target=None
            self._epoch+=1
            worker=self._thread
            request=self._request
        if worker is not None and worker.is_alive():
            self.navigation.request_cancel_for(request)
            worker.join(self.cancel_timeout)
            if worker.is_alive():
                raise RuntimeError("arrival correction cancellation has no terminal confirmation")

    def _cancelled(self, epoch):
        with self._lock:
            return epoch != self._epoch or self._target is None or self._cancel_sent

    def tick(self):
        cancel=False
        request=None
        with self._lock:
            target=self._target
            if target is None:
                return
            snapshot=self.pose_snapshot()
            active=self._thread is not None and self._thread.is_alive()
            request=self._request
            if snapshot is None or snapshot[0] != target.floor_id or snapshot[1] != self._generation:
                self._target=None;self._epoch+=1
                self.detail="hold released: map generation or localization evidence changed"
                cancel=active
            else:
                _,_,x,y,yaw=snapshot
                distance=math.hypot(target.x-x,target.y-y)
                yaw_error=abs(math.atan2(math.sin(target.yaw-yaw),math.cos(target.yaw-yaw)))
                if active:
                    within=distance<=self.r_off and yaw_error<=self.yaw_off
                    timed_out=self.clock()-self._started>=self.correction_timeout
                    if (within or timed_out) and not self._cancel_sent:
                        self._cancel_sent=True;cancel=True
                        self.detail="hold tolerance reached" if within else "hold correction timed out"
                        if timed_out:self._target=None;self._epoch+=1
                elif distance>=self.r_on or yaw_error>=self.yaw_on:
                    self._cancel_sent=False
                    self._started=self.clock()
                    epoch=self._epoch
                    self._request=NavigationRequest(target,self._generation)
                    self._thread=threading.Thread(target=self._restore,args=(self._request,epoch),name="arrival-nav-restore",daemon=True)
                    self._thread.start()
                    self.detail="restoring arrival position through existing NAV"
        if cancel:self.navigation.request_cancel_for(request)

    def _restore(self,request,epoch):
        try:
            result=self.navigation.execute(request,lambda:self._cancelled(epoch))
            if result.outcome is NavigationOutcome.NAVIGATION_FAILED:
                raise RuntimeError("arrival restoration navigation failed")
            if result.outcome is NavigationOutcome.CANCELLED and not self._cancelled(epoch):
                raise RuntimeError("arrival restoration externally cancelled; automatic retry disabled")
            if result.outcome is NavigationOutcome.SUCCEEDED and not self._cancelled(epoch):
                pose = self.pose_snapshot()
                target = request.location
                if (pose is None or pose[:2] != (target.floor_id, request.expected_generation) or
                        math.hypot(target.x-pose[2], target.y-pose[3]) > self.r_off or
                        abs(math.atan2(math.sin(target.yaw-pose[4]), math.cos(target.yaw-pose[4]))) > self.yaw_off):
                    raise RuntimeError("NAV success outside hold tolerance; precision commissioning required")
        except Exception as error:
            with self._lock:
                if epoch==self._epoch:
                    self._target=None;self._epoch+=1
                    self.detail=str(error)


class RosNavigationHold:
    def __init__(self,navigation,state,locations):
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger,TriggerResponse,Empty
        from multifloor_manager.ros_services import BoundedServiceCaller
        self.state=state
        self.locations={x.id:x for x in locations}
        self.core=NavigationHold(navigation,state.hold_pose,
            *(float(rospy.get_param("~arrival_hold/"+key)) for key in
              ("r_on","r_off","yaw_on","yaw_off","correction_timeout","cancel_timeout")))
        self.publisher=rospy.Publisher("~hold_status",String,queue_size=1,latch=True)
        self.release=rospy.ServiceProxy("/stair_supervisor/release_arrival_hold",Trigger)
        self._nomotion_name="/request_nomotion_update"
        self._nomotion=rospy.ServiceProxy(self._nomotion_name,Empty)
        self._services=BoundedServiceCaller(.5)
        self._localization_timer=rospy.Timer(
            rospy.Duration(float(rospy.get_param("~arrival_hold/localization_refresh_sec",.5))),
            lambda _e:self._refresh_localization())
        self._timer=rospy.Timer(rospy.Duration(.2),lambda _e:self._tick())
        def stop(_request):
            try:
                self.suspend()
                response=self.release()
                return TriggerResponse(success=response.success,message=response.message)
            except Exception as error:
                return TriggerResponse(success=False,message=str(error))
        self._stop_service=rospy.Service("~stop_arrival_hold",Trigger,stop)
        rospy.on_shutdown(self.shutdown)

    def _refresh_localization(self):
        # While idle, AMCL's motion threshold may otherwise leave its pose stale.
        # A failed refresh cancels this hold; it never invents a fresh pose.
        with self.core._lock:
            if not self.core.armed:
                return
            epoch=self.core._epoch
        try:
            self._services.call(self._nomotion_name,self._nomotion)
        except Exception as error:
            import rospy
            detail="hold localization refresh failed: "+str(error)
            self.core.fail_localization(epoch,detail)
            rospy.logwarn(detail)

    def _tick(self):
        self.core.tick()
        self.publisher.publish(self.core.detail)

    def suspend(self):self.core.suspend()

    def arm(self,location_id,location_override=None):
        floor=self.state.floor_state()
        location=location_override or self.locations[location_id]
        if floor.floor_id != location.floor_id:
            raise ValueError("arrival hold floor does not match confirmed mission location")
        # READY may be published after the pose used to confirm the new map.
        # Obtain a pose associated with this generation before releasing LiDAR.
        deadline=time.monotonic()+3.
        while True:
            pose=self.state.hold_pose()
            if pose is not None and pose[:2]==(location.floor_id,int(floor.map_generation)):
                break
            if time.monotonic()>=deadline:
                raise RuntimeError("arrival hold has no fresh pose in the confirmed map")
            self._services.call(self._nomotion_name,self._nomotion)
            time.sleep(.05)
        # At a stair entry, hold the accepted waiting pose rather than pulling
        # back toward the nominal 5 cm map target after approach completion.
        if location.type in ("STAIR_ENTRY", "SCAN", "LANDING") or location_override is not None:
            from dataclasses import replace
            location = replace(location, x=pose[2], y=pose[3], yaw=pose[4])
        def takeover():
            response=self._services.call("/stair_supervisor/release_arrival_hold",self.release)
            if not response.success:
                raise RuntimeError(response.message)
        self.core.arm(location,int(floor.map_generation),takeover)

    def shutdown(self):
        self._timer.shutdown()
        self._localization_timer.shutdown()
        self.core.suspend()
        self._stop_service.shutdown()
