"""Derive forward-facing downhill geometry from the same surveyed uphill route."""
from copy import deepcopy
import math


def reverse_route(spec, source, profile, scale):
    if set(spec) != {'id','reverse_of','commissioned','short_descent_input','entry_map'}:
        raise ValueError('reverse route requires id, reverse_of, commissioned, short_descent_input, entry_map')
    if source['direction'] != 'UP' or 'flight_3' not in source:
        raise ValueError('RF reverse route requires the surveyed three-flight UP route')
    inputs = profile.downhill_forward_inputs
    if len(inputs) != 2 or not 0 < spec['short_descent_input'] <= 1:
        raise ValueError('RF descent requires short-flight input and two long-flight inputs')
    r=deepcopy(source)
    r.update(id=spec['id'],direction='DOWN',commissioned=spec['commissioned'],
             reverse_of=source['id'],entry_map=deepcopy(spec['entry_map']),
             landing_turn_policy='supported_region',roof_turn_policy='supported_region',
             flight_input_v=[scale*spec['short_descent_input'],scale*inputs[0],scale*inputs[1]])
    r.pop('entry_reference',None);r.pop('motion_prediction',None)
    for i,j in [(1,3),(2,2),(3,1)]:
        r['flight_%d'%i]=deepcopy(source['flight_%d'%j][::-1])
        r['flight_%d_polygon'%i]=deepcopy(source['flight_%d_polygon'%j])
    r['entry_polygon']=deepcopy(source['exit_polygon'])
    r['exit_polygon']=deepcopy(source['entry_polygon'])
    r['landing_polygon']=deepcopy(source['roof_landing_polygon'])
    r['roof_landing_polygon']=deepcopy(source['landing_polygon'])
    wrap=lambda x:math.atan2(math.sin(x),math.cos(x))
    # Same flat pivots, traversed in reverse. Last approach point belongs to
    # the next flight; never substitute a mirrored/invented landing rectangle.
    roof=source['roof_turn_path'];mid=source['turn_path']
    a=list(roof[0][:2]);b=list(mid[-2][:2]);c=list(mid[1][:2])
    yaw1=math.atan2(r['flight_1'][1][1]-r['flight_1'][0][1],r['flight_1'][1][0]-r['flight_1'][0][0])
    yaw2=math.atan2(r['flight_2'][1][1]-r['flight_2'][0][1],r['flight_2'][1][0]-r['flight_2'][0][0])
    yaw3=math.atan2(r['flight_3'][1][1]-r['flight_3'][0][1],r['flight_3'][1][0]-r['flight_3'][0][0])
    cross=math.atan2(c[1]-b[1],c[0]-b[0])
    r['turn_path']=[[*a,yaw1],[*a,yaw2],[*r['flight_2'][0][:2],yaw2]]
    r['roof_turn_path']=[[*b,yaw2],[*b,cross],[*c,cross],[*c,yaw3],[*r['flight_3'][0][:2],yaw3]]
    limits=r['limits'];limits.update(min_flight_v=min(r['flight_input_v']),max_v=max(max(r['flight_input_v']),limits['hold_v']),
        flight_restart_v=min(r['flight_input_v']))
    # Slow descent takes longer than ascent; progress remains sensor-based.
    r['phase_test_limits'].update(FORWARD_SEGMENT_1=60.,FORWARD_SEGMENT_2=180.,FORWARD_SEGMENT_3=180.,
        TURN_TO_NEXT_FLIGHT=90.,ROOFTOP_TURN=120.)
    return r
