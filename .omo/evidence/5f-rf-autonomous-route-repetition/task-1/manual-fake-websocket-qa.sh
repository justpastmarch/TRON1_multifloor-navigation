#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ROS_PORT="${TASK1_ROS_PORT:-11421}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME

# shellcheck disable=SC1091
source "${ROOT}/devel/setup.bash"

qa_pid=""
core_pid=""
cleanup() {
    trap - INT TERM EXIT
    if [[ -n "$qa_pid" ]]; then
        kill -TERM "$qa_pid" 2>/dev/null || true
    fi
    if [[ -n "$core_pid" ]]; then
        kill -TERM -- "-${core_pid}" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-${core_pid}" 2>/dev/null || true
        wait "$core_pid" 2>/dev/null || true
    fi
}
trap cleanup INT TERM EXIT

setsid roscore -p "$ROS_PORT" >/dev/null 2>&1 < /dev/null &
core_pid=$!
sleep 2

PYTHONPATH="${ROOT}/src/stair_supervisor/test:${PYTHONPATH:-}" python3 -c '
import rospy
from test_websocket_tx_ros import make_node

rospy.init_node("task1_fake_websocket_qa")
node, socket, factory, clock = make_node("/test/manual_tx_qa")
rospy.sleep(2.0)
clock.sleep(0.3)
node.stream_once()
rospy.sleep(1.0)
node.shutdown()
print("FAKE_WS_TX_COUNT={}".format(len(socket.sent_payloads)))
print("FAKE_WS_LAST={}".format(socket.sent_payloads[-1]))
print("FAKE_WS_CONNECTIONS={}".format(factory.calls))
rospy.signal_shutdown("task 1 QA complete")
' &
qa_pid=$!

sleep 1
printf '%s\n' '--- /stair_supervisor/websocket_tx ---'
timeout 8s rostopic echo -n 1 /stair_supervisor/websocket_tx
wait "$qa_pid"
qa_pid=""
