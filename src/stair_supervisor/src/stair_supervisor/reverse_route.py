"""Derive forward-facing downhill geometry from the same surveyed uphill route.

Two-flight routes (one landing) and the three-flight RF route (landing plus
roof landing) are both reversed from their surveyed UP geometry: flights are
traversed in reverse order and direction, entry/exit regions swap, and every
turn reuses the UP route's flat pivots. Only the ``entry_map`` start pose and
the forward descent inputs are new configuration.
"""
from copy import deepcopy
import math


def _heading(flight):
    return math.atan2(flight[1][1] - flight[0][1], flight[1][0] - flight[0][0])


def _reversed_middle_turn(mid, next_flight_start, yaw_in, yaw_out):
    """Traverse a surveyed approach/rotation landing turn backwards.

    ``mid`` is the UP route's ``[approach, rotate, cross, rotate, approach]``
    path on that landing. Its cross-landing leg is reused in reverse; the last
    approach point belongs to the next flight, never an invented rectangle.
    """
    b = list(mid[-2][:2])
    c = list(mid[1][:2])
    cross = math.atan2(c[1] - b[1], c[0] - b[0])
    return [[*b, yaw_in], [*b, cross], [*c, cross], [*c, yaw_out], [*next_flight_start[:2], yaw_out]]


def reverse_route(spec, source, profile, scale):
    if source['direction'] != 'UP':
        raise ValueError('reverse route requires a surveyed UP route')
    roof = 'flight_3' in source
    expected = {'id', 'reverse_of', 'commissioned', 'entry_map'} | ({'short_descent_input'} if roof else set())
    if set(spec) != expected:
        raise ValueError('reverse route requires ' + ', '.join(sorted(expected)))
    inputs = profile.downhill_forward_inputs
    if len(inputs) != 2:
        raise ValueError('forward descent requires two long-flight inputs (downhill_forward_inputs)')
    if roof and not 0 < spec['short_descent_input'] <= 1:
        raise ValueError('RF descent requires a short-flight input in (0, 1]')
    flights = 3 if roof else 2
    r = deepcopy(source)
    r.update(id=spec['id'], direction='DOWN', commissioned=spec['commissioned'],
             reverse_of=source['id'], entry_map=deepcopy(spec['entry_map']),
             landing_turn_policy='supported_region',
             flight_input_v=([scale * spec['short_descent_input']] if roof else [])
             + [scale * inputs[0], scale * inputs[1]])
    r.pop('entry_reference', None)
    r.pop('motion_prediction', None)
    for i in range(1, flights + 1):
        j = flights + 1 - i
        r['flight_%d' % i] = deepcopy(source['flight_%d' % j][::-1])
        r['flight_%d_polygon' % i] = deepcopy(source['flight_%d_polygon' % j])
    r['entry_polygon'] = deepcopy(source['exit_polygon'])
    r['exit_polygon'] = deepcopy(source['entry_polygon'])
    # Clearance belongs to the physical region, not its traversal direction.
    for destination, origin in (('entry', 'exit'), ('exit', 'entry')):
        key = destination + '_support_margin_m'
        source_key = origin + '_support_margin_m'
        r['limits'].pop(key, None)
        if source_key in source['limits']:
            r['limits'][key] = source['limits'][source_key]
    yaw = [None] + [_heading(r['flight_%d' % i]) for i in range(1, flights + 1)]
    if roof:
        r['roof_turn_policy'] = 'supported_region'
        r['landing_polygon'] = deepcopy(source['roof_landing_polygon'])
        r['roof_landing_polygon'] = deepcopy(source['landing_polygon'])
        # Same flat pivots, traversed in reverse.
        a = list(source['roof_turn_path'][0][:2])
        r['turn_path'] = [[*a, yaw[1]], [*a, yaw[2]], [*r['flight_2'][0][:2], yaw[2]]]
        r['roof_turn_path'] = _reversed_middle_turn(source['turn_path'], r['flight_3'][0], yaw[2], yaw[3])
    else:
        # One landing: the same physical region and its surveyed cross leg.
        r['turn_path'] = _reversed_middle_turn(source['turn_path'], r['flight_2'][0], yaw[1], yaw[2])
    limits = r['limits']
    limits.update(min_flight_v=min(r['flight_input_v']),
                  max_v=max(max(r['flight_input_v']), limits['hold_v']),
                  flight_restart_v=min(r['flight_input_v']))
    # Slow descent takes longer than ascent; progress remains sensor-based.
    if roof:
        r['phase_test_limits'].update(FORWARD_SEGMENT_1=60., FORWARD_SEGMENT_2=180., FORWARD_SEGMENT_3=180.,
                                      TURN_TO_NEXT_FLIGHT=90., ROOFTOP_TURN=120.)
    else:
        r['phase_test_limits'].update(FORWARD_SEGMENT_1=180., FORWARD_SEGMENT_2=180., TURN_TO_NEXT_FLIGHT=90.)
    return r
