#!/usr/bin/env bash
# Runs ON the mini PC (deployed as ~/.local/share/tron1-stair-supervisor/mapping.sh).
# The mapping console, its ROS master and all recording stay on the mini PC, so a
# weak link to the workstation only affects viewing. No robot motion transport.
#   start   local master + sensors attached to it + mapping console on 127.0.0.1:8766
#   stop    stop the console (seals recordings) and the master; sensors go back to waiting
#   status  print unit states
set -Eeuo pipefail
base="${HOME}/.local/share/tron1-stair-supervisor"
workspace="${base}/ws"
output="${HOME}/TRON1_maps"
port="${MAPPING_CONSOLE_PORT:-8766}"
ros_env="set +u; source /opt/ros/noetic/setup.bash; source ${HOME}/catkin_ws/devel/setup.bash; source ${workspace}/devel/setup.bash; \
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1; unset ROS_HOSTNAME"
active() { systemctl --user is-active --quiet "$1"; }
case "${1:-}" in
    start)
        if active tron1-mapping.service; then
            # New code (second argument "changed") is loaded only when nothing is recording.
            state="$(curl -s -m 3 "http://127.0.0.1:${port}/api/state" || true)"
            if [[ "${2:-}" == changed && "$state" == *'"recording": false'* && "$state" != *'"state": "RUNNING"'* && "$state" != *'"state": "QUEUED"'* ]]; then
                echo 'restarting mapping console with updated code'
                systemctl --user stop tron1-mapping.service
            else
                echo 'mapping console already running'; exit 0
            fi
        fi
        mkdir -p "${output}/mapping" "${output}/artifacts"
        if ! active tron1-mapping-master.service; then
            systemd-run --user --collect --unit=tron1-mapping-master --property=KillSignal=SIGINT \
                bash -c "${ros_env}; exec roscore -p 11311" >/dev/null
        fi
        ready=0
        for _ in $(seq 1 60); do
            if bash -c "${ros_env}; timeout 2 rostopic list" >/dev/null 2>&1; then ready=1; break; fi
            sleep .5
        done
        [[ "$ready" == 1 ]] || { echo 'local ROS master did not start' >&2; exit 1; }
        # Re-attach the sensor watcher: with no workstation master it picks this one.
        systemctl --user restart tron1-sensors.service
        systemd-run --user --collect --unit=tron1-mapping --property=KillSignal=SIGINT --property=TimeoutStopSec=90s \
            --setenv=MAPPING_LOCAL_RAW=1 \
            --setenv=STAIR_LIDAR_CONFIG="${workspace}/src/stair_supervisor/config/stair_lidar_3f_4f_test.yaml" \
            --setenv=MAPPING_OUTPUT_ROOT="${output}/mapping" --setenv=SCAN_OUTPUT_BASE="${output}/artifacts" \
            bash -c "${ros_env}; exec rosrun mission_manager mission_manager_node.py _mode:=mapping _console_port:=${port} _scan_output_base:=${output}/artifacts" >/dev/null
        for _ in $(seq 1 120); do
            if curl -s -m 2 -o /dev/null "http://127.0.0.1:${port}/api/state"; then echo 'mapping console ready'; exit 0; fi
            active tron1-mapping.service || { journalctl --user -u tron1-mapping -n 30 --no-pager >&2; exit 1; }
            sleep .5
        done
        echo 'mapping console did not answer' >&2; exit 1
        ;;
    stop)
        systemctl --user stop tron1-mapping.service 2>/dev/null || true
        systemctl --user stop tron1-mapping-master.service 2>/dev/null || true
        systemctl --user restart tron1-sensors.service
        echo 'mapping console stopped'
        ;;
    status)
        for unit in tron1-mapping-master tron1-mapping tron1-sensors; do
            printf '%s %s\n' "$unit" "$(systemctl --user is-active "$unit" 2>/dev/null || true)"
        done
        ;;
    *) echo 'Usage: mapping.sh start|stop|status' >&2; exit 2 ;;
esac
