"""Finite-horizon response envelopes. Pure computation, no ROS or transport.

Commands are nominal transport inputs, never observations. Commissioned bounds
describe a conditional field model, not a proof of contact stability. Unverified
models can be evaluated in shadow mode but cannot admit production commands.
"""
from dataclasses import dataclass
import itertools
import math
import numpy as np


def validate_model(model):
    keys = {'mode', 'commissioned', 'source', 'forward_gain', 'yaw_gain',
            'response_sec', 'position_error_m', 'velocity_error_mps',
            'yaw_error_rad', 'yaw_rate_error_radps', 'max_age_sec'}
    if not isinstance(model, dict) or set(model) != keys:
        raise ValueError('motion_prediction requires explicit response bounds and provenance')
    if model['mode'] not in ('shadow', 'control') or type(model['commissioned']) is not bool:
        raise ValueError('motion_prediction mode/commissioned invalid')
    if model['mode'] == 'control' and not model['commissioned']:
        raise ValueError('unverified motion_prediction may only run in shadow mode')
    if not isinstance(model['source'], str) or not model['source'].strip():
        raise ValueError('motion_prediction needs response evidence source')
    for name in ('forward_gain', 'yaw_gain'):
        pair = model[name]
        if (not isinstance(pair, (list, tuple)) or len(pair) != 2 or
                any(type(x) not in (int, float) or not math.isfinite(x) for x in pair)
                or not 0 <= pair[0] <= pair[1]):
            raise ValueError('motion_prediction gain interval invalid')
    for name in keys - {'mode', 'commissioned', 'source', 'forward_gain', 'yaw_gain'}:
        value = model[name]
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError('motion_prediction bounds must be finite and positive')
    return model


@dataclass(frozen=True)
class MotionState:
    position: np.ndarray
    yaw: float
    velocity: np.ndarray
    yaw_rate: float
    age: float
    heading_spread: float = 0.
    velocity_spread: float = 0.


def response_envelope(state, command, horizon, model, steps=24):
    """Return extreme response paths and uncertainty radii at every time.

    Propagate translation by age, preserving the original measurement outside
    this function. Heading has already been advanced causally by the caller.
    Current velocity/slip persists for zero input and decays with response lag;
    zero input does not freeze the predicted body. Interval extremes alone are
    not claimed to cover every nonlinear/physical response between them.
    """
    if (not 0 <= state.age <= model['max_age_sec'] or horizon <= 0
            or not np.isfinite(np.r_[state.position, state.velocity, state.yaw,
                                    state.yaw_rate, state.heading_spread,
                                    state.velocity_spread, command, horizon]).all()):
        raise ValueError('motion_prediction state invalid or beyond propagation age')
    times = np.linspace(0., horizon, steps + 1)
    dt = horizon / steps
    lag = model['response_sec']
    decay = math.exp(-dt / lag)
    heading = np.array([math.cos(state.yaw), math.sin(state.yaw)])
    forward = float(state.velocity @ heading)
    slip = state.velocity - forward * heading
    propagated = state.position + state.velocity * state.age
    # Vectorize response hypotheses together; keep only the time recurrence.
    gains = [sorted(set((pair[0], sum(pair)/2., pair[1])))
             for pair in (model['forward_gain'], model['yaw_gain'])]
    gain_v,gain_w=np.asarray(list(itertools.product(*gains))).T
    count=len(gain_v)
    centers=np.empty((count,steps+1,2));angles=np.empty((count,steps+1))
    centers[:,0]=propagated;angles[:,0]=state.yaw
    v=np.full(count,forward);w=np.full(count,state.yaw_rate);yaw=np.full(count,state.yaw)
    for i in range(steps):
        next_v=gain_v*command[0]+(v-gain_v*command[0])*decay
        next_w=gain_w*command[1]+(w-gain_w*command[1])*decay
        yaw_mid=yaw+(w+next_w)*dt/4.
        drift=slip*math.exp(-(times[i]+dt/2.)/lag)
        centers[:,i+1]=centers[:,i]+dt*((v+next_v)[:,None]/2.*np.column_stack((np.cos(yaw_mid),np.sin(yaw_mid)))+drift)
        yaw=yaw+(w+next_w)*dt/2.
        angles[:,i+1]=yaw
        v,w=next_v,next_w
    elapsed = state.age + times
    position_error = model['position_error_m'] + (model['velocity_error_mps']+state.velocity_spread)*elapsed
    yaw_error = model['yaw_error_rad'] + state.heading_spread + model['yaw_rate_error_radps']*elapsed
    return dict(times=times, centers=np.asarray(centers), angles=np.asarray(angles),
                position_error=position_error, yaw_error=yaw_error,
                propagated_position=propagated, measured_position=state.position,
                model='lagged_measured_motion_envelope', physical_hold_verified=False)
