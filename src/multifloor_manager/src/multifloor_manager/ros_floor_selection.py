"""Operator relocation, distinct from a completed stair traversal."""
import rospy
from nav_msgs.srv import GetMap
from multifloor_manager.msg import FloorState
from multifloor_manager.srv import SelectFloor, SelectFloorResponse
from multifloor_manager.map_evidence import fingerprint_occupancy_grid, Nanoseconds, MapGeneration, MapGenerationState


class FloorSelection:
    def __init__(self, node):
        self.node = node
        self.service = rospy.Service('/multifloor/select_floor', SelectFloor, self.select)

    def select(self, request):
        node = self.node
        startup = node.startup
        target = request.floor_id
        if target not in node.floors:
            return SelectFloorResponse(False, 'unknown floor')
        if startup is None:
            return SelectFloorResponse(False, 'managed localization is disabled')
        claimed = False
        generation = None
        try:
            with node._active_lock, startup.lock:
                if node._active or startup.busy() or not startup.stationary():
                    return SelectFloorResponse(False, 'finish the mission and stop before selecting the actual floor')
                node._active = True
                claimed = True
                startup.generation += 1
                generation = startup.generation
                startup.request = None
                startup.pose = None
                startup.latest_map = None
                with node.runtime.condition:
                    node.runtime.epoch += 1
                    node.runtime.active_epoch = None
                    node.runtime.localization = None
                    node.runtime.initial_floor = target
                    node.runtime.initial_identity = node.identities[target]
                    node.runtime.map_state = MapGenerationState(MapGeneration(int(node.runtime.map_state.generation)+1), None)
                    node.runtime.publish_floor(FloorState.UNKNOWN, 'operator selecting actual floor: '+target, target)
            path = str(node.config_root / node.floors[target].map_yaml)
            response = node.services.call(node.change_map_name, node.change_map, path)
            if response.result != 0:
                raise ValueError('map server rejected the selected map')
            # Noetic change_map reports result only (its response.map is empty).
            # Read the map actually loaded, then verify its full configured identity.
            map_service = rospy.get_param('~get_map_service', '/static_map')
            grid = node.services.call(map_service, rospy.ServiceProxy(map_service, GetMap)).map
            fingerprint = fingerprint_occupancy_grid(grid, Nanoseconds(rospy.Time.now().to_nsec()))
            if fingerprint.identity() != node.identities[target]:
                raise ValueError('map response does not match selected floor')
            with startup.lock, node.runtime.condition:
                if startup.generation != generation:
                    raise ValueError('floor selection was superseded')
                node.runtime.map_state = MapGenerationState(node.runtime.map_state.generation, fingerprint)
                node.runtime.initial_floor = target
                node.runtime.initial_identity = node.identities[target]
                node.runtime.map_guard = None
                node.runtime.tag_state = node.runtime.tag_context = None
                node.runtime.tag_accepted = False
                node.runtime.costmap_identity = None
                startup.latest_map = grid
                startup.request = ('auto', None)
                node.runtime.publish_floor(FloorState.UNKNOWN, 'selected '+target+'; waiting for localization', target)
                startup.publish('SEARCHING', 'actual floor selected; searching pose')
                startup.wake.set()
            return SelectFloorResponse(True, 'map changed; localization must confirm READY before driving')
        except Exception as error:
            if claimed:
                with startup.lock, node.runtime.condition:
                    if generation is None or startup.generation == generation:
                        node.runtime.publish_floor(FloorState.UNKNOWN, 'floor selection failed: '+str(error), target)
            return SelectFloorResponse(False, str(error))
        finally:
            if claimed:
                with node._active_lock:
                    node._active = False
