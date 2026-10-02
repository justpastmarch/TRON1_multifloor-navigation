#!/usr/bin/env bash
# Reuse the installed receiver only if it actually publishes to this master.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/config.env"
ssh -o BatchMode=yes -o ConnectTimeout=5 "${MINI_PC_USER}@${MINI_PC_HOST}" bash -s <<REMOTE
source /opt/ros/noetic/setup.bash
source '${MINI_PC_WORKSPACE}/devel/setup.bash'
export ROS_MASTER_URI='http://${ROS_MASTER_HOST}:${ROS_MASTER_PORT}'
export ROS_IP='${MINI_PC_ROS_IP}' ROBOT_TYPE='${ROBOT_TYPE}'
unset ROS_HOSTNAME
if timeout 4s rostopic echo -n 1 '${SENSOR_JOY_TOPIC}' >/dev/null 2>&1; then
    echo '[JOY] existing receiver is publishing'
    exit 0
fi
root="\$HOME/.local/share/tron1-sensor-joy"
test -x "\$root/venv/bin/python" && test -r "\$root/sensor_joy_bridge.py" || exit 1
# A receiver left alive across a ROS-master restart may be unregistered.
pids=\$(pgrep -f "^\$root/venv/bin/python \$root/sensor_joy_bridge.py( |\$)" || true)
if [ -n "\$pids" ]; then
    kill -TERM \$pids
    for pid in \$pids; do
        for attempt in 1 2 3 4 5 6 7 8 9 10; do
            kill -0 "\$pid" 2>/dev/null || break
            sleep 0.5
        done
        if kill -0 "\$pid" 2>/dev/null; then
            echo '[JOY] previous receiver has not exited; refusing a duplicate' >&2
            exit 1
        fi
    done
fi
nohup "\$root/venv/bin/python" "\$root/sensor_joy_bridge.py" \
    --robot-ip '${ROBOT_HOST}' --topic '${SENSOR_JOY_TOPIC}' \
    >"\$root/receiver.log" 2>&1 < /dev/null &
printf '%s\n' "\$!" >"\$root/receiver.pid"
timeout 10s rostopic echo -n 1 '${SENSOR_JOY_TOPIC}' >/dev/null 2>&1 || exit 1
echo '[JOY] receiver reconnected; fresh messages confirmed'
REMOTE
