#!/usr/bin/env python3
"""Post-run field check for stair traversal bags (read-only; never opens ROS or the robot).

Usage: field_check.py [--config stair_lidar_3f_4f_test.yaml] [--json out.json] BAG [BAG ...]

Reports, per bag:
  0. config_sha256 recorded by the supervisor vs the local config file
  1. flight lateral drift: slope of ey along the flight axis (deg; ~0 when the
     entry template matches the route axes)
  2. entry_attitude_check outcomes (re-level actions at stair entry)
  3. margin recovery usage (flight / landing / turn) and the basis used
  4. landing and rooftop turns: duration, yaw turned, body-forward drift speed
     while only rotation was commanded, minimum clearance, peak |w|
  5. first faults (phase, tracking state, reason) and command transmit gaps
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import rosbag
import yaml

DEBUG = '/stair_supervisor/control_debug'
STATUS = '/stair_supervisor/tracking_status'
TX = '/stair_supervisor/websocket_tx'
TURN_PHASES = ('TURN_TO_NEXT_FLIGHT', 'ROOFTOP_TURN')
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'stair_lidar_3f_4f_test.yaml'


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def flight_axes(config):
    axes = {}
    for route in yaml.safe_load(config.read_text())['routes']:
        for i in (1, 2, 3):
            flight = route.get('flight_%d' % i)
            if flight:
                d = np.asarray(flight[1][:2], float) - np.asarray(flight[0][:2], float)
                axes[(route['id'], 'FORWARD_SEGMENT_%d' % i)] = (np.asarray(flight[0][:2], float), d / np.linalg.norm(d))
    return axes


def unique_poses(ticks):
    out = []
    for d in ticks:
        if d.get('pose') is None or d.get('yaw') is None or d.get('tracking_state') != 'TRACKED':
            continue
        if out and out[-1]['pose'] == d['pose'] and out[-1]['yaw'] == d['yaw']:
            continue
        out.append(d)
    return out


def segments(rows, phases=None):
    """Contiguous runs of the same (route, phase, run_id)."""
    runs, key, cur = [], None, []
    for t, d in rows:
        k = (d.get('route_id'), d.get('phase'), d.get('run_id'))
        if k != key:
            if cur:
                runs.append((key, cur))
            key, cur = k, []
        if phases is None or k[1] in phases:
            cur.append(dict(d, _t=t))
    if cur:
        runs.append((key, cur))
    return [(k, c) for k, c in runs if c and (phases is None or k[1] in phases)]


def flight_drift(rows, axes):
    out = []
    for (route, phase, _), ticks in segments(rows):
        if (route, phase) not in axes:
            continue
        origin, axis = axes[(route, phase)]
        u = unique_poses(ticks)
        if len(u) < 10:
            continue
        p = np.array([d['pose'][:2] for d in u]) - origin
        along, lateral = p @ axis, p @ np.array([-axis[1], axis[0]])
        if along.max() - along.min() < .5:
            continue
        slope, _ = np.polyfit(along, lateral, 1)
        clearance = [d['clearance_m'] for d in u if d.get('clearance_m') is not None]
        out.append(dict(route=route, phase=phase, samples=len(u), progress_m=float(along.max() - along.min()),
                        drift_slope_deg=float(math.degrees(math.atan(slope))),
                        lateral_start_m=float(lateral[0]), lateral_end_m=float(lateral[-1]),
                        min_clearance_m=min(clearance) if clearance else None))
    return out


def turn_report(rows):
    out = []
    for (route, phase, _), ticks in segments(rows, TURN_PHASES):
        u = unique_poses(ticks)
        turned = sum(abs(wrap(b['yaw'] - a['yaw'])) for a, b in zip(u, u[1:]))
        creep = []
        for a, b in zip(u, u[1:]):
            sent = (a.get('last_sent_twist') or {}).get('normalized_xyz')
            dt = b['_t'] - a['_t']
            if not sent or abs(sent[0]) > 1e-6 or abs(sent[2]) < .02 or not 0 < dt < .6:
                continue
            dp = np.asarray(b['pose'][:2]) - np.asarray(a['pose'][:2])
            creep.append((math.cos(a['yaw']) * dp[0] + math.sin(a['yaw']) * dp[1]) / dt)
        clearance = [d['clearance_m'] for d in ticks if d.get('clearance_m') is not None]
        commands = [abs(d['command'][1]) for d in ticks if d.get('command')]
        out.append(dict(route=route, phase=phase, duration_s=float(ticks[-1]['_t'] - ticks[0]['_t']),
                        yaw_turned_deg=float(math.degrees(turned)),
                        forward_creep_mps=float(np.median(creep)) if len(creep) >= 5 else None,
                        creep_pairs=len(creep), min_clearance_m=min(clearance) if clearance else None,
                        peak_w=max(commands) if commands else None,
                        inward_guidance_ticks=sum(bool((d.get('landing_turn') or {}).get('inward_guidance')) for d in ticks),
                        landing_recovery_ticks=sum(bool(d.get('landing_margin_recovery')) for d in ticks),
                        non_tracked_ticks=sum(d.get('tracking_state') != 'TRACKED' for d in ticks)))
    return out


def check(path, config, axes):
    rows, shas, attitude, faults, tx = [], collections.Counter(), [], {}, []
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=[DEBUG, STATUS, TX]):
            try:
                d = json.loads(msg.data)
            except ValueError:
                continue
            if topic == TX:
                if d.get('title') == 'request_twist':
                    tx.append(t.to_sec())
                continue
            if topic == STATUS:
                shas[d.get('config_sha256')] += 1
                check_ = d.get('entry_attitude_check')
                if check_ and (not attitude or attitude[-1] != check_):
                    attitude.append(check_)
                continue
            if d.get('phase') is None:
                continue
            rows.append((t.to_sec(), d))
            fault = d.get('first_fault')
            if isinstance(fault, dict) and fault.get('at') not in faults:
                faults[fault.get('at')] = dict(phase=fault.get('phase'), tracking_state=fault.get('tracking_state'),
                                               reason=fault.get('reason'), route=d.get('route_id'),
                                               clearance_m=fault.get('clearance_m'), roll=fault.get('roll'),
                                               pitch=fault.get('pitch'))
    expected = hashlib.sha256(config.read_bytes()).hexdigest()
    recovery = collections.Counter()
    for _, d in rows:
        for key in ('flight_margin_recovery', 'landing_margin_recovery', 'turn_margin_recovery'):
            if d.get(key):
                recovery[key] += 1
        if d.get('flight_margin_recovery'):
            recovery['flight_basis:' + str(d.get('flight_margin_recovery_basis'))] += 1
    # Transmit gaps only while actively tracking a moving phase (idle periods
    # between attempts legitimately send nothing).
    active = np.array(sorted(t for t, d in rows if d.get('tracking_state') == 'TRACKED'
                             and d.get('phase') not in (None, 'VERIFY_ENTRY')))
    gaps = []
    for a, b in zip(tx, tx[1:]):
        if active.size:
            i = np.searchsorted(active, a)
            if i < active.size and active[i] - a < .2 and (i == 0 or a - active[i-1] < .2):
                gaps.append(b - a)
    gaps = np.asarray(gaps)
    return dict(bag=os.path.basename(path),
                config=dict(expected=expected, recorded=dict(shas), match=set(shas) <= {expected, None} and expected in shas),
                flights=flight_drift(rows, axes), entry_attitude_checks=attitude,
                recovery_ticks=dict(recovery), turns=turn_report(rows), first_faults=list(faults.values()),
                tx=dict(count=len(tx), gap_p99_s=float(np.percentile(gaps, 99)) if gaps.size else None,
                        gap_max_s=float(gaps.max()) if gaps.size else None))


def fmt(v, spec='%.3f'):
    return '-' if v is None else spec % v


def show(r):
    print('\n=== %s' % r['bag'])
    c = r['config']
    print('0. config: %s  recorded=%s' % ('OK' if c['match'] else 'MISMATCH (not this branch?)',
          ', '.join('%s x%d' % (str(k)[:12], n) for k, n in c['recorded'].items())))
    print('1. flights (drift slope ~0 deg expected):')
    for f in r['flights']:
        print('   %-18s %-17s slope %+6.1f deg  lateral %+.3f -> %+.3f m over %.2f m  min clearance %s m' % (
            f['route'], f['phase'], f['drift_slope_deg'], f['lateral_start_m'], f['lateral_end_m'],
            f['progress_m'], fmt(f['min_clearance_m'])))
    print('2. entry attitude checks: %s' % ('none recorded' if not r['entry_attitude_checks'] else ''))
    for a in r['entry_attitude_checks']:
        print('   %s' % json.dumps(a)[:240])
    print('3. recovery ticks: %s' % (r['recovery_ticks'] or 'none'))
    print('4. turns:')
    for t in r['turns']:
        print('   %-18s %-20s %5.1f s  turned %5.0f deg  creep %s m/s (n=%d)  min clearance %s m  peak |w| %s  inward %d  landing-recovery %d  non-tracked %d' % (
            t['route'], t['phase'], t['duration_s'], t['yaw_turned_deg'], fmt(t['forward_creep_mps'], '%+.3f'),
            t['creep_pairs'], fmt(t['min_clearance_m']), fmt(t['peak_w']), t['inward_guidance_ticks'],
            t['landing_recovery_ticks'], t['non_tracked_ticks']))
    print('5. first faults: %s' % ('none' if not r['first_faults'] else ''))
    for f in r['first_faults']:
        print('   %s / %s / %s: %s (clearance %s)' % (f['route'], f['phase'], f['tracking_state'],
              str(f['reason'])[:120], fmt(f['clearance_m'])))
    print('   command TX: %d sent, gap p99 %s s, max %s s' % (r['tx']['count'], fmt(r['tx']['gap_p99_s']), fmt(r['tx']['gap_max_s'])))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args()
    axes = flight_axes(args.config)
    results = []
    for path in args.bags:
        try:
            results.append(check(path, args.config, axes))
        except (rosbag.ROSBagException, OSError) as error:
            print('\n=== %s\n   skipped: %s' % (os.path.basename(path), error))
            continue
        show(results[-1])
    if args.json:
        args.json.write_text(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
