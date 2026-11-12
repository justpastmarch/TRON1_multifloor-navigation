"""Exclusive navigation and stair command ownership state machine."""

from __future__ import annotations

import math
import threading
from typing import Callable

from .configuration import StairProfile, StairSupervisorConfiguration
from .robot_transport import TransportFault
from .stair_evidence import EvidenceReport, Phase
from .supervisor_types import (
    AdmissionValidator,
    CommandTransport,
    PhaseEvidence,
    ResultCode,
    StairGoal,
    SupervisorClock,
    SupervisorState,
    TraversalResult,
)


class _LidarTransitionFailed(Exception):
    pass


class StairSupervisor:
    """Mutable sole owner of one transport and its command epoch."""

    _SAFE_CHECKPOINTS = frozenset(
        (Phase.VERIFY_ENTRY, Phase.LANDING, Phase.EXIT_CONFIRM)
    )

    def __init__(
        self,
        configuration: StairSupervisorConfiguration,
        transport: CommandTransport,
        evidence: PhaseEvidence,
        clock: SupervisorClock,
        feedback: Callable[[EvidenceReport], None],
        admission: AdmissionValidator,
        nav_freshness_sec: float,
        turn_linear_mps: float = 0.0,
        lidar_control=None,
        operator_recovery_enabled=False,
    ) -> None:
        self._configuration = configuration
        self._profiles = {profile.id: profile for profile in configuration.profiles}
        self._transport = transport
        self._evidence = evidence
        self._clock = clock
        self._feedback = feedback
        self._admission = admission
        self._nav_freshness_sec = nav_freshness_sec
        self._turn_linear_mps = turn_linear_mps
        self._lidar_control = lidar_control
        self._operator_recovery_enabled = operator_recovery_enabled
        self._arrival_hold = False
        self._landing_test_hold = False
        self._manual_received_at = None
        self._manual_active = False
        self._retained_loss = False
        self._state = SupervisorState.DISARMED
        self._ownership_epoch = 0
        self._latest_nav: tuple[float, float, float, int] | None = None
        self._navigation_limits = (
            configuration.robot.move_base.max_vel_x,
            configuration.robot.move_base.max_vel_theta,
        )
        self._navigation_limit_revision = 0
        self._state_lock = threading.RLock()
        # Lock order: _tx_lock, then _state_lock; never the reverse.
        # _tx_lock serializes ownership changes with the supervisor's own robot
        # I/O, so a command decided for one owner/epoch cannot be sent after a
        # change. _state_lock is held only for short reads/writes, never across
        # robot I/O or LiDAR evaluation, so status queries and joystick/cmd_vel
        # callbacks do not wait for a slow socket.
        self._tx_lock = threading.RLock()
        self._traversal_lock = threading.Lock()
        self._closed = False
        self._shutdown_requested = threading.Event()
        self._traversal_thread_id = None
        self._operator_run = None
        self._operator_request = None
        self._operator_applying = False
        self._operator_result = {}

    def operator_status(self):
        with self._state_lock:
            result = dict(self._operator_run or {}, pending=self._operator_request is not None or self._operator_applying, result=dict(self._operator_result))
            result['continue_supported'] = self._lidar_control is not None and self._operator_recovery_enabled
            result['resume_available'] = bool(self._operator_run and not self._operator_run.get('active')
                and self._state is SupervisorState.STAIR and self._retained_loss
                and self._lidar_control is not None)
            return result

    def request_operator_phase(self, request):
        with self._state_lock:
            run = self._operator_run
            if not run or not run.get('active') or self._state is not SupervisorState.STAIR:
                raise ValueError('실행 중인 계단 임무가 없습니다.')
            if (request.get('run_id') != run['run_id'] or request.get('revision') != run['revision']
                    or request.get('confirmed') is not True):
                raise ValueError('단계가 변경되었습니다. 현재 상태를 확인하고 다시 선택하세요.')
            phase = request.get('phase')
            kind = request.get('kind', 'set_phase')
            if kind not in ('set_phase', 'continue'):
                raise ValueError('지원하지 않는 단계 요청입니다.')
            if phase not in run['phases']:
                raise ValueError('현재 경로에 없는 단계입니다.')
            if kind == 'continue' and (not run.get('waiting') or phase != run.get('phase')):
                raise ValueError('같은 단계 이어가기는 중단 대기 중인 현재 단계에서만 가능합니다.')
            if self._operator_request is not None or self._operator_applying:
                raise ValueError('이전 단계 전이를 처리 중입니다.')
            self._operator_request = dict(kind=kind, phase=phase, run_id=run['run_id'], revision=run['revision'], request_id=request.get('request_id',''))
            self._operator_result = dict(request_id=request.get('request_id',''),phase=phase,kind=kind,state='ACCEPTED')
            return dict(accepted=True, phase=phase, kind=kind, run_id=run['run_id'])

    @property
    def state(self) -> SupervisorState:
        with self._state_lock:
            return self._state

    @property
    def ownership_epoch(self) -> int:
        with self._state_lock:
            return self._ownership_epoch

    def start(self) -> None:
        """Open the sole command session and arm NAV forwarding."""
        if self._shutdown_requested.is_set():
            raise TransportFault("supervisor has shut down")
        try:
            self._transport.start()
        except TransportFault:
            with self._state_lock:
                self._state = SupervisorState.FAULT
            self._close_transport()
            raise
        with self._tx_lock, self._state_lock:
            self._state = SupervisorState.NAV

    def set_navigation_limits(self, linear_mps=None, angular_radps=None, *, revision=None) -> tuple[float, float]:
        """Apply NAV-only limits without changing stair commands or ownership."""
        with self._state_lock:
            previous = self._navigation_limits
            values = (previous[0] if linear_mps is None else linear_mps,
                      previous[1] if angular_radps is None else angular_radps)
            full_scale = self._configuration.robot.websocket_full_scale
            if any(not math.isfinite(v) or v < 0 for v in values):
                raise ValueError('NAV limits must be finite and nonnegative')
            limits = (min(values[0], full_scale.linear_mps),
                      min(values[1], full_scale.angular_radps))
            if revision is not None:
                if type(revision) is not int or revision <= 0:
                    raise ValueError('NAV limit revision must be a positive integer')
                if revision < self._navigation_limit_revision:
                    raise ValueError('superseded NAV limit request')
                if revision == self._navigation_limit_revision and limits != previous:
                    raise ValueError('NAV limit revision reused with different limits')
                self._navigation_limit_revision = revision
            self._navigation_limits = limits
            return self._navigation_limits

    def accept_navigation(self, linear_mps: float, angular_radps: float) -> None:
        """Capture a finite NAV command in the currently owned epoch."""
        with self._state_lock:
            if self._state is not SupervisorState.NAV:
                return
            if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
                self._latest_nav = None
                return
            self._latest_nav = (
                linear_mps,
                angular_radps,
                self._clock.monotonic(),
                self._ownership_epoch,
            )
            # A fresh moving NAV request takes over the temporary arrival hold.
            # Idle zero publications are not evidence that another goal owns it.
            if linear_mps != 0.0 or angular_radps != 0.0:
                self._arrival_hold = False

    def stream_navigation(self) -> None:
        """Emit one NAV tick or a complete zero for absent/stale input.

        The decision is a short snapshot under _state_lock; LiDAR evaluation
        and robot I/O run under _tx_lock only. Ownership changes also take
        _tx_lock, so the snapshot cannot go stale before this tick is sent.
        """
        try:
            with self._tx_lock:
                with self._state_lock:
                    if self._shutdown_requested.is_set():
                        return
                    now = self._clock.monotonic()
                    landing_hold = self._landing_test_hold
                    manual_fresh = (not self._manual_active and self._manual_received_at is not None
                                    and 0 <= now-self._manual_received_at <= .5)
                    retained = self._retained_loss
                    state = self._state
                    arrival_hold = self._arrival_hold
                    command = self._latest_nav
                    epoch = self._ownership_epoch
                    max_linear, max_angular = self._navigation_limits
                if landing_hold:
                    if not manual_fresh:
                        self._retain_lidar_loss('landing hold released: manual input or joystick feedback stale')
                        self._zero_barrier()
                        return
                    report = self._lidar_control.evaluate(Phase.LANDING, now, command_required=True)
                    if report.faulted:
                        self._retain_lidar_loss(report.detail)
                        self._zero_barrier()
                        return
                    self._transport.update_twist(*self._lidar_control.command())
                    self._transport.send_current()
                    return
                if retained:
                    self._lidar_control.evaluate(self._lidar_control.phase, self._clock.monotonic())
                    self._zero_barrier()  # command neutralization, not physical hold evidence
                    return
                if state is not SupervisorState.NAV:
                    return
                if arrival_hold and self._lidar_control is not None:
                    report = self._lidar_control.evaluate(Phase.EXIT_CONFIRM, self._clock.monotonic(), command_required=True)
                    if report.faulted:
                        self._retain_lidar_loss(report.detail)
                        return
                    self._transport.update_twist(*self._lidar_control.command())
                    self._transport.send_current()
                    return
                linear, angular = 0.0, 0.0
                if command is not None:
                    command_linear, command_angular, received_at, command_epoch = command
                    fresh = self._clock.monotonic() - received_at <= self._nav_freshness_sec
                    if fresh and command_epoch == epoch:
                        linear, angular = command_linear, command_angular
                linear = max(-max_linear, min(max_linear, linear))
                angular = max(-max_angular, min(max_angular, angular))
                self._transport.update_twist(linear, angular)
                self._transport.send_current()
        except TransportFault:
            self._latch_fault()
            raise

    def traverse(
        self,
        goal: StairGoal,
        cancellation_requested: Callable[[], bool],
        *,
        phase_test=None,
        resume=None,
    ) -> TraversalResult:
        """Execute a normal route, one phase, or an entry-to-phase test prefix."""
        from_entry = False
        if resume is not None:
            try:
                if phase_test is not None or not isinstance(resume, dict) or set(resume) != {'run_id', 'revision', 'phase'}:
                    raise ValueError('invalid stair resume request')
                resume_phase = Phase(resume['phase'])
                if type(resume['revision']) is not int or not isinstance(resume['run_id'], str):
                    raise ValueError('invalid stair resume checkpoint')
            except (ValueError, TypeError, KeyError) as error:
                return TraversalResult(ResultCode.INVALID_GOAL, str(error))
        if phase_test is not None:
            try:
                required = {"route_id", "phase", "max_duration_sec", "operator_confirmed"}
                if not isinstance(phase_test, dict) or not required <= set(phase_test) or set(phase_test) - required - {"from_entry"}:
                    raise ValueError("phase_test needs route_id, phase, max_duration_sec, operator_confirmed")
                from_entry = phase_test.get("from_entry", False)
                if type(from_entry) is not bool:
                    raise ValueError("from_entry must be boolean")
                test_phase = Phase(phase_test["phase"])
                duration = phase_test["max_duration_sec"]
                if self._lidar_control is None or phase_test["operator_confirmed"] is not True:
                    raise ValueError("phase test requires LiDAR control and operator confirmation")
                if phase_test["route_id"] != goal.stair_id or (test_phase is Phase.EXIT_CONFIRM and not from_entry):
                    raise ValueError("phase test route mismatch; final arrival requires a connected entry test")
                if type(duration) not in (float, int) or not math.isfinite(duration) or duration <= 0:
                    raise ValueError("phase test duration must be positive finite seconds")
            except (ValueError, TypeError, KeyError) as error:
                return TraversalResult(ResultCode.INVALID_GOAL, str(error))
        if not self._profiles:
            return TraversalResult(
                ResultCode.CAPABILITY_DISABLED,
                "stair capability is disabled",
            )
        if not self._traversal_lock.acquire(blocking=False):
            return TraversalResult(ResultCode.BUSY, "another traversal owns commands")
        try:
            self._traversal_thread_id = threading.get_ident()
            if self._shutdown_requested.is_set():
                return TraversalResult(ResultCode.STAIR_FAILED, "supervisor is shutting down", cancelled=True)
            profile = self._profiles.get(goal.stair_id)
            if profile is None or profile.direction is not goal.direction:
                return TraversalResult(ResultCode.INVALID_GOAL, "unknown stair or direction")
            if not profile.enabled:
                return TraversalResult(
                    ResultCode.CAPABILITY_DISABLED,
                    "stair profile is disabled",
                )
            with self._state_lock:
                test_resume = phase_test is not None and self._state is SupervisorState.STAIR and (self._retained_loss or self._landing_test_hold)
                run = self._operator_run
                action_resume = (resume is not None and self._state is SupervisorState.STAIR and self._retained_loss
                    and run is not None and not run.get('active') and run.get('route_id') == goal.stair_id
                    and run.get('run_id') == resume['run_id'] and run.get('revision') == resume['revision']
                    and resume_phase.value in run.get('phases', ()))
                if resume is not None and not action_resume:
                    return TraversalResult(ResultCode.ENTRY_REJECTED, 'stair route or checkpoint changed; refresh the current run')
                if self._state is not SupervisorState.NAV and not test_resume and not action_resume:
                    return TraversalResult(ResultCode.BUSY, "supervisor is not ready for traversal/test")
                admitted_state = self._state
                ownership_epoch = self._ownership_epoch
            if phase_test is None and resume is None:
                admission = self._admission.validate(goal, ownership_epoch)
                if not admission.accepted:
                    code = (
                        ResultCode.COMMUNICATION_LOST
                        if admission.communication_error
                        else ResultCode.ENTRY_REJECTED
                    )
                    return TraversalResult(code, admission.reason)
            # Operator phase tests are not floor-transition missions. The fresh
            # LiDAR anchor, surveyed route and phase geometry remain mandatory.
            if self._lidar_control is not None:
                try:
                    if phase_test is not None:
                        phases, budgets = self._lidar_control.test_plan(profile, test_phase, duration, from_entry)
                        self._lidar_control.prepare(profile, self._clock.monotonic(), phase_test=True, start_phase=phases[0])
                    elif resume is None:
                        self._lidar_control.prepare(profile, self._clock.monotonic(), start_phase=Phase.VERIFY_ENTRY)
                    elif resume_phase.value == run['phase']:
                        self._lidar_control.continue_phase(self._clock.monotonic())
                    else:
                        self._lidar_control.operator_phase(resume_phase, self._clock.monotonic())
                except ValueError as error:
                    return TraversalResult(ResultCode.ENTRY_REJECTED, str(error))
            with self._tx_lock, self._state_lock:
                if self._state is not admitted_state or self._ownership_epoch != ownership_epoch:
                    return TraversalResult(ResultCode.ENTRY_REJECTED, "stair admission ownership changed")
                self._state = SupervisorState.STAIR
                self._latest_nav = None
                self._retained_loss = False
                self._landing_test_hold = False
            if phase_test is not None:
                # A phase test rearms the controller and invalidates any old
                # full-mission turn pivot/checkpoint; never offer that stale run.
                with self._state_lock:
                    self._operator_run = None
                return self._execute_phase_test(profile, phases, budgets, duration, cancellation_requested,
                                                admitted_state=admitted_state, retained_loss=test_resume)
            return self._execute_profile(profile, cancellation_requested, resume_phase=resume_phase if resume is not None else None)
        finally:
            with self._state_lock:
                if self._operator_run is not None:
                    self._operator_run['active'] = False
                if self._operator_request is not None or (self._operator_applying and self._operator_result.get('state') == 'ACCEPTED'):
                    self._operator_result.update(state='REJECTED',reason='임무가 종료되어 전이를 적용하지 않았습니다.')
                self._operator_request = None
                self._operator_applying = False
            self._traversal_thread_id = None
            self._traversal_lock.release()

    def _execute_phase_test(self, profile, phases, budgets, duration, cancellation_requested,
                            *, admitted_state, retained_loss):
        """Test the selected sequence without normal success or floor handoff."""
        control = self._lidar_control
        try:
            self._evidence.arm(profile, self._clock.monotonic(), phase_test=True, start_phase=phases[0])
        except ValueError as error:
            with self._tx_lock, self._state_lock:
                self._state, self._retained_loss = admitted_state, retained_loss
            return TraversalResult(ResultCode.ENTRY_REJECTED, str(error))
        except Exception as error:
            return self._control_exception(error)
        with self._tx_lock, self._state_lock:
            self._arrival_hold = False
        try:
            control.test_status = dict(state="MODE_ACK", target_phase=phases[-1].value,
                                       phases=[p.value for p in phases], max_duration_sec=duration)
            try:
                neutral = phases[0] in (Phase.VERIFY_ENTRY, Phase.ALIGN, Phase.FORWARD_SEGMENT_1)
                self._set_lidar_mode(True, cancellation_requested, phase_override=phases[0], neutral_only=neutral)
                started = self._clock.monotonic()
                # Mode acknowledgment has its own budget; all test timers begin here.
                control.started_at = started
                if neutral:
                    control.reset_command(started)
                deadline = started + min(duration, profile.timeout_sec)
                reason = "phase test target reached; zero velocity; not stair arrival"
                for phase in phases:
                    phase_started = self._clock.monotonic()
                    self._evidence.begin_phase(phase, phase_started)
                    phase_deadline = min(deadline, phase_started + budgets[phase])
                    control.test_status.update(state="RUNNING", current_phase=phase.value,
                                               phase_budget_sec=budgets[phase])
                    while True:
                        now = self._clock.monotonic()
                        control.test_status.update(elapsed_sec=now-started, phase_elapsed_sec=now-phase_started)
                        if self._shutdown_requested.is_set() or cancellation_requested():
                            raise _LidarTransitionFailed("phase test interrupted; zero velocity; operator takeover")
                        if now >= phase_deadline:
                            raise _LidarTransitionFailed("phase test time complete at " + phase.value + "; not stair arrival")
                        report = self._evidence.evaluate(phase, now)
                        self._feedback(report)
                        if self._shutdown_requested.is_set() or cancellation_requested():
                            raise _LidarTransitionFailed("phase test interrupted before command send")
                        if self._clock.monotonic() >= phase_deadline:
                            raise _LidarTransitionFailed("phase test time complete at " + phase.value + "; not stair arrival")
                        if report.faulted:
                            raise _LidarTransitionFailed(report.detail)
                        if report.complete:
                            break
                        self._transport.update_twist(*control.command())
                        self._transport.send_current()
                        self._clock.sleep(self._transport.stream_period_sec)
                control.test_status.update(state="TARGET_REACHED")
                if phases[-1] is Phase.LANDING:
                    with self._tx_lock:
                        with self._state_lock:
                            now = self._clock.monotonic()
                            eligible = (not cancellation_requested() and not self._shutdown_requested.is_set()
                                        and not self._manual_active and self._manual_received_at is not None
                                        and 0 <= now-self._manual_received_at <= .5)
                        if eligible:
                            self._zero_barrier()
                            try:
                                control.begin_landing_hold(now)
                            except ValueError as error:
                                reason = 'landing reached, hold unavailable: '+str(error)
                            else:
                                with self._state_lock:
                                    self._landing_test_hold = True
                                control.test_status.update(state='LANDING_HOLD')
                                return TraversalResult(ResultCode.STAIR_FAILED,
                                    'phase test target reached; landing position hold active; not stair arrival', cancelled=True)
                        else:
                            reason = 'landing reached; no hold: manual input, stale joystick feedback or cancellation'
            except _LidarTransitionFailed as error:
                reason = str(error)
                control.test_status.update(state="STOPPED", reason=reason)
            self._retain_lidar_loss(reason)
            return TraversalResult(ResultCode.STAIR_FAILED, reason, cancelled=True)
        except TransportFault as error:
            control.test_status.update(state="TRANSPORT_FAULT", reason=str(error))
            self._latch_fault()
            return TraversalResult(ResultCode.COMMUNICATION_LOST, str(error))
        except Exception as error:
            control.test_status.update(state="CONTROL_EXCEPTION", reason=str(error))
            return self._control_exception(error)

    def _execute_profile(
        self,
        profile: StairProfile,
        cancellation_requested: Callable[[], bool],
        *, resume_phase=None,
    ) -> TraversalResult:
        started_at = self._clock.monotonic()
        try:
            if resume_phase is None:
                self._evidence.arm(profile, started_at)
        except ValueError as error:
            with self._tx_lock, self._state_lock:
                self._state = SupervisorState.NAV
            return TraversalResult(ResultCode.ENTRY_REJECTED, str(error))
        except Exception as error:
            return self._control_exception(error)
        with self._tx_lock, self._state_lock:
            self._arrival_hold = False
        cancel_pending = False
        phases = (self._lidar_control.traversal_phases(profile) if self._lidar_control is not None
                  else tuple(p for p in Phase if p not in (Phase.ROOFTOP_TURN, Phase.FORWARD_SEGMENT_3)))
        import uuid
        with self._state_lock:
            self._operator_result = {}
            self._operator_applying = False
            self._operator_run = dict(run_id=uuid.uuid4().hex, route_id=profile.id, active=True,
                                      phase=(resume_phase or phases[0]).value, phases=[p.value for p in phases], revision=0,
                                      last_operator_phase=None, error='', waiting=False, stop_reason='')
        try:
            if self._lidar_control is None:
                self._zero_barrier()
                self._transport.request_stair_mode(True)
            else:
                self._set_lidar_mode(True, cancellation_requested, resume_phase, True, preserve_phase=resume_phase is not None)
            phase_index = phases.index(resume_phase) if resume_phase is not None else 0
            first_resumed_phase = resume_phase is not None
            while phase_index < len(phases):
                phase = phases[phase_index]
                if first_resumed_phase:
                    first_resumed_phase = False  # preserve turn pivot/substep on same-phase resume
                else:
                    self._evidence.begin_phase(phase, self._clock.monotonic())
                with self._state_lock:
                    self._operator_run.update(phase=phase.value, revision=self._operator_run['revision']+1)
                    self._operator_applying = False
                while True:
                    with self._state_lock:
                        operator_request, self._operator_request = self._operator_request, None
                        self._operator_applying = operator_request is not None
                        if operator_request is not None and operator_request['revision'] != self._operator_run['revision']:
                            self._operator_run['error'] = '요청 후 단계가 변경되어 전이를 적용하지 않았습니다.'
                            self._operator_result.update(state='REJECTED',reason=self._operator_run['error'])
                            operator_request = None
                            self._operator_applying = False
                    if operator_request is not None and cancellation_requested():
                        with self._state_lock:
                            self._operator_result = dict(request_id=operator_request['request_id'],state='REJECTED',reason='정지 요청으로 전이를 적용하지 않았습니다.')
                            self._operator_applying = False
                        operator_request = None
                    if operator_request is not None:
                        target = Phase(operator_request['phase'])
                        kind = operator_request.get('kind', 'set_phase')
                        try:
                            if self._lidar_control is None:
                                raise ValueError('LiDAR 제어가 없는 임무는 단계 전이를 지원하지 않습니다.')
                            waiting = self._operator_run.get('waiting', False)
                            if kind == 'continue':
                                if not waiting or target is not phase:
                                    raise ValueError('같은 단계 이어가기는 중단 대기 중인 현재 단계에서만 가능합니다.')
                                with self._tx_lock:
                                    with self._state_lock:
                                        self._lidar_control.continue_phase(self._clock.monotonic())
                                    self._zero_barrier()
                                self._set_lidar_mode(True, cancellation_requested, target, True, preserve_phase=True)
                                with self._tx_lock, self._state_lock:
                                    self._retained_loss = False
                                    self._operator_run.update(last_operator_phase=target.value, error='', waiting=False,
                                                              revision=self._operator_run['revision']+1)
                                    self._operator_result = dict(request_id=operator_request['request_id'],
                                        phase=target.value,kind=kind,state='APPLIED',reason='현재 단계의 진행 위치를 유지해 계속합니다.')
                            else:
                                with self._tx_lock:
                                    with self._state_lock:
                                        self._lidar_control.operator_phase(target, self._clock.monotonic())
                                        self._retained_loss = False
                                    self._zero_barrier()
                                if waiting:
                                    self._set_lidar_mode(True, cancellation_requested, target, True)
                                phase_index = phases.index(target)-1
                                with self._state_lock:
                                    self._operator_run.update(last_operator_phase=target.value, error='', waiting=False)
                                    self._operator_result = dict(request_id=operator_request['request_id'],phase=target.value,
                                        kind=kind,state='APPLIED',reason='사용자 지정 단계 적용')
                                break
                        except (ValueError, _LidarTransitionFailed) as error:
                            if self._operator_run.get('waiting'):
                                self._retain_lidar_loss(str(error))
                            with self._state_lock:
                                self._operator_run['error'] = str(error)
                                self._operator_result = dict(request_id=operator_request['request_id'],state='REJECTED',reason=str(error))
                                self._operator_applying = False
                    if self._shutdown_requested.is_set():
                        return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
                    if self._operator_run.get('waiting'):
                        if cancellation_requested():
                            self._retain_lidar_loss('operator cancelled recovery wait')
                            return TraversalResult(ResultCode.STAIR_FAILED, 'operator cancelled recovery wait', cancelled=True)
                        self._clock.sleep(self._transport.stream_period_sec)
                        continue
                    report = self._evidence.evaluate(phase, self._clock.monotonic())
                    self._feedback(report)
                    cancel_pending = cancel_pending or cancellation_requested()
                    if self._lidar_control is not None and cancel_pending:
                        self._retain_lidar_loss("cancelled; physical handoff still required")
                        return TraversalResult(ResultCode.STAIR_FAILED, "cancelled; supervisor retains stair ownership", cancelled=True)
                    if phase in self._SAFE_CHECKPOINTS and cancel_pending:
                        return self._finish(cancelled=True)
                    if report.faulted:
                        if self._lidar_control is not None:
                            if self._operator_recovery_enabled:
                                self._retain_lidar_loss(report.detail)
                                with self._state_lock:
                                    self._operator_run.update(waiting=True, stop_reason=report.detail,
                                                              revision=self._operator_run['revision']+1)
                                continue
                            if self._try_entry_nav_recovery():
                                return TraversalResult(ResultCode.ENTRY_REJECTED, report.detail + "; flat entry restored to NAV; retry available")
                            self._retain_lidar_loss(report.detail)
                            return TraversalResult(ResultCode.STAIR_FAILED, report.detail)
                        self._zero_barrier()
                        self._latch_fault()
                        return TraversalResult(ResultCode.STAIR_FAILED, report.detail)
                    if report.complete:
                        break
                    linear, angular = (self._lidar_control.command() if self._lidar_control is not None
                                       else self._phase_command(phase, profile))
                    if self._shutdown_requested.is_set():
                        return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
                    self._transport.update_twist(linear, angular)
                    self._transport.send_current()
                    self._clock.sleep(self._transport.stream_period_sec)
                phase_index += 1
            with self._state_lock:
                self._operator_run['active'] = False
            return self._finish(cancelled=False, cancellation_requested=cancellation_requested)
        except _LidarTransitionFailed as error:
            if self._shutdown_requested.is_set():
                return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
            try:
                if self._try_entry_nav_recovery():
                    return TraversalResult(ResultCode.ENTRY_REJECTED, str(error) + "; flat entry restored to NAV; retry available", cancelled=cancellation_requested())
                self._retain_lidar_loss(str(error))
            except TransportFault as transport_error:
                self._latch_fault()
                return TraversalResult(ResultCode.COMMUNICATION_LOST, str(transport_error))
            return TraversalResult(ResultCode.STAIR_FAILED, str(error), cancelled=cancellation_requested())
        except TransportFault as error:
            self._latch_fault()
            return TraversalResult(ResultCode.COMMUNICATION_LOST, str(error))
        except Exception as error:
            return self._control_exception(error)

    def _control_exception(self, error):
        """Contain an unexpected computation/callback failure at the owner."""
        reason = "control exception %s: %s" % (type(error).__name__, error)
        try:
            if self._lidar_control is not None:
                self._retain_lidar_loss(reason)
            else:
                self._zero_barrier()
                self._latch_fault()
        except TransportFault as transport_error:
            self._latch_fault()
            return TraversalResult(ResultCode.COMMUNICATION_LOST, str(transport_error))
        return TraversalResult(ResultCode.STAIR_FAILED, reason)

    def _set_lidar_mode(self, enabled, cancellation_requested, phase_override=None, neutral_only=False, *, preserve_phase=False):
        """Maintain measured support correction while mode acknowledgment waits."""
        phase = phase_override if phase_override is not None else (Phase.VERIFY_ENTRY if enabled else Phase.EXIT_CONFIRM)
        started = self._clock.monotonic()
        if not preserve_phase:
            self._lidar_control.begin_phase(phase, started)
        if neutral_only:
            self._zero_barrier()
            self._lidar_control.reset_command(started)
        next_send = started
        def progress():
            nonlocal next_send
            now = self._clock.monotonic()
            if self._shutdown_requested.is_set() or cancellation_requested():
                raise _LidarTransitionFailed("mode handoff interrupted")
            if now - started >= self._lidar_control.route["limits"]["handoff_sec"]:
                raise _LidarTransitionFailed("mode handoff exceeded commissioned budget")
            if now < next_send:
                return
            report = self._lidar_control.evaluate(phase, now, command_required=True, neutral_only=neutral_only)
            self._feedback(report)
            if self._shutdown_requested.is_set() or cancellation_requested():
                raise _LidarTransitionFailed("mode handoff interrupted before command send")
            if self._clock.monotonic()-started >= self._lidar_control.route["limits"]["handoff_sec"]:
                raise _LidarTransitionFailed("mode handoff exceeded commissioned budget")
            if report.faulted:
                raise _LidarTransitionFailed(report.detail)
            self._transport.update_twist(*((0., 0.) if neutral_only else self._lidar_control.command()))
            self._transport.send_current()
            next_send = now + self._transport.stream_period_sec
        self._transport.request_stair_mode_with_feedback(enabled, progress)

    def _phase_command(
        self,
        phase: Phase,
        profile: StairProfile,
    ) -> tuple[float, float]:
        linear = min(
            profile.linear_speed,
            self._configuration.robot.websocket_full_scale.linear_mps,
        )
        angular = min(
            profile.angular_speed,
            self._configuration.robot.websocket_full_scale.angular_radps,
        )
        first_linear = math.copysign(linear, profile.flight_1_distance_m)
        landing_angular = math.copysign(angular, profile.landing_turn_yaw_rad)
        second_linear = math.copysign(linear, profile.flight_2_distance_m)
        if profile.downhill_forward_inputs:
            # Convert normalized targets into the transport's internal scale;
            # normalize_twist divides by that same scale before sending.
            # These are input intensities, not measured downhill m/s.
            full_scale = self._configuration.robot.websocket_full_scale.linear_mps
            first_linear, second_linear = (value * full_scale for value in profile.downhill_forward_inputs)
        commands = {
            Phase.VERIFY_ENTRY: (0.0, 0.0),
            Phase.ALIGN: (0.0, 0.0),
            Phase.FORWARD_SEGMENT_1: (first_linear, 0.0),
            Phase.LANDING: (0.0, 0.0),
            Phase.TURN_TO_NEXT_FLIGHT: (min(self._turn_linear_mps, linear), landing_angular),
            Phase.FORWARD_SEGMENT_2: (second_linear, 0.0),
            Phase.EXIT_CONFIRM: (0.0, 0.0),
        }
        return commands[phase]

    def _finish(self, *, cancelled: bool, cancellation_requested=lambda: False) -> TraversalResult:
        if self._shutdown_requested.is_set():
            return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
        if self._lidar_control is None:
            self._zero_barrier()
            self._transport.request_stair_mode(False)
            self._zero_barrier()
        else:
            self._set_lidar_mode(False, cancellation_requested)
        if self._lidar_control is not None and not cancelled:
            started = self._clock.monotonic()
            limits = self._lidar_control.route['limits']
            settle_budget = limits.get('exit_settle_sec', limits['handoff_sec'])
            self._lidar_control.begin_phase(Phase.EXIT_CONFIRM, started)
            while True:
                if self._shutdown_requested.is_set():
                    return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
                now = self._clock.monotonic()
                report = self._lidar_control.evaluate(Phase.EXIT_CONFIRM, now, command_required=True)
                self._feedback(report)
                cancel = cancellation_requested()
                if report.faulted or cancel or now - started > settle_budget:
                    self._retain_lidar_loss("arrival handoff not established")
                    return TraversalResult(ResultCode.STAIR_FAILED, "arrival handoff not established", cancelled=cancel)
                self._transport.update_twist(*self._lidar_control.command())
                self._transport.send_current()
                if report.complete:
                    break
                self._clock.sleep(self._transport.stream_period_sec)
        with self._tx_lock:
            with self._state_lock:
                shutdown = self._shutdown_requested.is_set()
                cancelled_now = not shutdown and self._lidar_control is not None and cancellation_requested()
            if shutdown:
                return TraversalResult(ResultCode.STAIR_FAILED, "supervisor shutdown", cancelled=True)
            if cancelled_now:
                self._retain_lidar_loss("cancelled before arrival ownership transfer")
                return TraversalResult(ResultCode.STAIR_FAILED, "cancelled before arrival ownership transfer", cancelled=True)
            self._hand_back_to_nav(cancelled)
        if cancelled:
            return TraversalResult(ResultCode.STAIR_FAILED, "cancelled", cancelled=True)
        return TraversalResult(ResultCode.OK, "stair traversal complete")

    def _hand_back_to_nav(self, cancelled: bool) -> None:
        """Ownership transfer to NAV; caller holds _tx_lock."""
        with self._state_lock:
            self._ownership_epoch += 1
            self._latest_nav = None
            self._state = SupervisorState.NAV
            self._arrival_hold = self._lidar_control is not None and not cancelled
            if self._arrival_hold:
                # Floor/tag/map preparation owns its own deadlines. Continue
                # measured flat-exit correction until NAV takes over or releases.
                self._lidar_control.begin_arrival_hold()

    def _try_entry_nav_recovery(self):
        """Restore NAV only before ascent, with repeated flat-pose/RC checks."""
        control = self._lidar_control
        check = getattr(control, 'can_return_from_entry', None)
        if check is None:
            return False
        started = self._clock.monotonic()
        def eligible():
            now = self._clock.monotonic()
            return (not self._shutdown_requested.is_set() and not self._manual_active and
                    self._manual_received_at is not None and
                    0 <= now-self._manual_received_at <= .5 and check(now))
        with self._state_lock:
            if not eligible():
                return False
        def progress():
            with self._tx_lock:
                with self._state_lock:
                    ok = eligible() and self._clock.monotonic()-started < control.route['limits']['handoff_sec']
                if not ok:
                    raise _LidarTransitionFailed('flat-entry recovery no longer confirmed')
                self._zero_barrier()
        try:
            progress()
            self._transport.request_stair_mode_with_feedback(False, progress)
            with self._tx_lock:
                with self._state_lock:
                    if not eligible():
                        return False
                self._zero_barrier()
                # Decided by the single check above (pre-lock-split behavior):
                # input that arrives during this zero send does not cancel it.
                with self._state_lock:
                    self._restore_entry_nav(control)
                return True
        except _LidarTransitionFailed:
            return False

    def _restore_entry_nav(self, control) -> None:
        """Caller holds _tx_lock and _state_lock."""
        self._latest_nav = None
        self._ownership_epoch += 1
        self._retained_loss = False
        self._arrival_hold = self._landing_test_hold = False
        self._state = SupervisorState.NAV
        control.interrupt()

    def _retain_lidar_loss(self, reason: str) -> None:
        """Execute the commissioned response without assuming zero/WALK holds.

        A terminal action result does not transfer command ownership to NAV.
        Only neutral velocity is requested; no physical posture hold is implied.
        Transport/session remains for explicit operator handoff; no emergency API.
        """
        with self._tx_lock:
            with self._state_lock:
                self._state = SupervisorState.STAIR
                self._latest_nav = None
                self._arrival_hold = False
                self._landing_test_hold = False
                first = not self._retained_loss
            if first:
                self._lidar_control.loss_response(self._transport)
                self._transport.update_twist(0., 0.)
                with self._state_lock:
                    self._retained_loss = True

    def accept_manual_input(self, active: bool) -> None:
        """Only relinquish the landing test hold; never auto-resume after RC."""
        with self._state_lock:
            self._manual_received_at = self._clock.monotonic()
            self._manual_active = bool(active)
            release = active and self._landing_test_hold
        if release:
            with self._tx_lock:
                with self._state_lock:
                    release = self._landing_test_hold
                if release:
                    self._retain_lidar_loss('landing hold released by operator')
                    self._zero_barrier()

    def release_arrival_hold(self) -> bool:
        """Explicit stop/new mission may disable the temporary flat-exit hold."""
        with self._tx_lock:
            with self._state_lock:
                landing = self._landing_test_hold
                if not landing:
                    if self._state is not SupervisorState.NAV:
                        return False
                    if not self._arrival_hold:
                        return True  # a late release must not clear a newer NAV command
                    self._arrival_hold = False
                    self._latest_nav = None
                    self._ownership_epoch += 1
            if landing:
                self._retain_lidar_loss('landing hold explicitly released')
            self._zero_barrier()
            return True

    def prepare_rooftop_navigation(self) -> bool:
        """Reassert WALK for an RF flat tour through the sole command owner."""
        with self._tx_lock:
            with self._state_lock:
                if self._state is not SupervisorState.NAV or self._traversal_lock.locked():
                    return False
            self._transport.send_walk_mode()
            self._zero_barrier()
            with self._state_lock:
                self._latest_nav = None
            return True

    def acknowledge_physical_handoff(self) -> bool:
        """Operator-only recovery after a retained loss, never automatic resume."""
        with self._tx_lock:
            with self._state_lock:
                if not self._retained_loss or self._traversal_lock.locked():
                    return False
            # The mode request waits for the robot; only _tx_lock is held.
            self._transport.request_stair_mode(False)
            self._zero_barrier()
            with self._state_lock:
                self._latest_nav = None
                self._ownership_epoch += 1
                self._retained_loss = False
                self._state = SupervisorState.NAV
            return True

    def _zero_barrier(self) -> None:
        self._transport.update_twist(0.0, 0.0)
        self._transport.send_current()

    def _latch_fault(self) -> None:
        with self._tx_lock:
            with self._state_lock:
                self._state = SupervisorState.FAULT
                self._latest_nav = None
            self._close_transport()

    def _close_transport(self) -> None:
        if not self._closed:
            self._closed = True
            try:
                self._transport.close()
            except TransportFault:
                with self._state_lock:
                    self._state = SupervisorState.FAULT
                    self._latest_nav = None

    def shutdown(self) -> None:
        """Stop motion before releasing the sole transport session."""
        self._shutdown_requested.set()
        if self._traversal_thread_id != threading.get_ident():
            with self._traversal_lock:
                self._shutdown_transport()
        else:
            self._shutdown_transport()

    def _shutdown_transport(self) -> None:
        with self._tx_lock:
            if not self._closed:
                try:
                    self._zero_barrier()
                except TransportFault:
                    with self._state_lock:
                        self._state = SupervisorState.FAULT
                self._close_transport()
            with self._state_lock:
                if self._state is not SupervisorState.FAULT:
                    self._state = SupervisorState.DISARMED
