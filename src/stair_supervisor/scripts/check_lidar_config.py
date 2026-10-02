#!/usr/bin/env python3
"""Read-only launch/configuration check; no ROS initialization or transport."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--mode', choices=('observe', 'control'), default='control')
    parser.add_argument('--snapshot-dir', type=Path)
    parser.add_argument('--mission', action='store_true', help='Require a registered automatic entry for the 3F->4F mission')
    parser.add_argument('--status-json', type=Path, help='Compare a saved tracking_status JSON object')
    args = parser.parse_args()
    # Resolve this checkout first when invoked directly, before catkin rebuild.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
    from stair_supervisor.configuration import load_lidar_configuration
    from stair_supervisor.stair_feedback import StairFeedback
    from stair_supervisor.lidar_tracking import load_imu_rotation
    import numpy
    import open3d
    import scipy
    import rospy
    import websocket
    from livox_ros_driver2.msg import CustomMsg
    config = load_lidar_configuration(args.config.resolve(), args.mode, args.mode == 'observe')
    calibration = (config.root / config.document['calibration']).resolve()
    _, calibration_sha256 = load_imu_rotation(calibration)
    control = StairFeedback(None, config.document['base_from_lidar'], config.document['routes']) if config.document['configured'] else None
    if args.mode == 'control' and not control.routes:
        raise ValueError('control configuration has no routes')
    result = dict(mode=args.mode, config_path=config.source_path, config_sha256=config.source_sha256,
                  calibration_path=str(calibration), calibration_sha256=calibration_sha256,
                  configured=config.document['configured'], python=sys.executable,
                  numpy=numpy.__version__, scipy=scipy.__version__, open3d=open3d.__version__,
                  routes=[dict(id=r['id'], commissioned=r['commissioned'], phase_test_limits=r.get('phase_test_limits'))
                          for r in config.document['routes']], robot_commands_sent=0)
    if args.mission:
        route=control.routes.get('stair_3f_4f_up') if control is not None else None
        blockers=[]
        if route is None or not route['commissioned']:
            blockers.append('3F->4F automatic mission entry has not been registered')
        reference=None if route is None else route.get('entry_reference')
        if reference is None:
            blockers.append('missing surveyed entry point-cloud reference; phase-test preview is not an automatic mission reference')
        else:
            path=config.root/reference['path']
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=reference['sha256']:
                blockers.append('entry point-cloud reference missing or fingerprint mismatch')
            if reference.get('unique_geometry_verified') is not True:
                blockers.append('entry reference has no operator geometry verification')
        result['automatic_entry_configured']=not blockers
        result['mission_blockers']=blockers
        if blockers:
            print(json.dumps(result,indent=2))
            raise SystemExit(2)
    if args.status_json:
        status = json.loads(args.status_json.read_text())
        for key in ('mode', 'config_path', 'config_sha256'):
            if status.get(key) != result[key]:
                raise ValueError('running configuration mismatch: ' + key)
        if status.get('control_policy') != StairFeedback.policy_version:
            raise ValueError('running control code differs; restart existing Supervisor')
        result['running_configuration_matches'] = True
    if args.snapshot_dir:
        args.snapshot_dir.mkdir(parents=True, exist_ok=True)
        content = args.config.read_bytes()
        if hashlib.sha256(content).hexdigest() != config.source_sha256:
            raise ValueError('configuration changed during check')
        calibration_content = calibration.read_bytes()
        if hashlib.sha256(calibration_content).hexdigest() != calibration_sha256:
            raise ValueError('calibration changed during check')
        (args.snapshot_dir / 'lidar-config.yaml').write_bytes(content)
        (args.snapshot_dir / 'lidar-imu-calibration.yaml').write_bytes(calibration_content)
        (args.snapshot_dir / 'configuration.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
