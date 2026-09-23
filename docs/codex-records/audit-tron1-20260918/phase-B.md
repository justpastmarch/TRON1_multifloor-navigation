# Phase B — launch, process, topic ownership

Status: static ownership reconstruction complete; live deployment ownership UNVERIFIED.

## Evidence boundary

OBSERVED repository: run.sh:178-257 creates lock, local master, remote sensor start/reuse command, tunnel and system launch; run.sh:198-210 cleans only recorded local PIDs; run.sh:382 waits only for system launch. Mini PC sensor implementation is outside this repository.

OBSERVED installed upstream: /opt/ros/noetic/lib/python3/dist-packages/roslaunch/core.py:432-434 defaults respawn=false and required=false; pmon.py:560-625 shuts down for required process failure and otherwise respawns only configured processes. Repository system.launch has no required nodes; apriltag.launch:21 explicitly enables detector respawn. A child dying does not by itself imply top-level launch exit.

OBSERVED installed actionlib: action_client.py:589-630 checks a received status and goal/cancel/result/feedback connections. verify_action_servers.py:17-34 therefore supplements registration-only checks in run.sh:313-350. Do not report readiness as registration-only.

Runtime observation command: date -u; ps -eo pid,ppid,stat,comm filtered for ROS/SSH/Python; ss -ltn. Observation: 2026-09-18T02:42:09Z. No matching process rows were returned within this execution environment; ss reported 'Cannot open netlink socket: Operation not permitted'. This does NOT establish that host or Mini PC ROS is stopped. No remote query, ROS publish, service mutation or robot command was executed.

## Ownership map

All rows describe OBSERVED repository declarations unless marked otherwise. Recovery is a code-path description, not an executed or approved operating procedure.

| Resource | Start / stop owner | Health / failure detection | Recovery and isolation |
|---|---|---|---|
| ROS master | run.sh:212 / cleanup:202-207 | startup list + master PID:214-220 | no post-ready master restart; cleanup stops local components |
| Mini PC sensor launch | remote shell via run.sh:224-225 / remote restart branch only | each listed topic must yield one message within 5s for reuse | any missing camera/lidar/odom/scan causes entire wf_mapping restart request; local exit leaves it alive |
| /scan | external wf_mapping contract, system.launch:30 | freshness + exact publisher count, run.sh:354-368 | actual external publisher implementation UNVERIFIED |
| /tron/wheel_odom_raw | external wf_mapping contract | run.sh:225,354-358; supervisor odom subscriber | actual process and recovery UNVERIFIED |
| AprilTag | system include -> apriltag.launch | duplicate process/registration startup check; tag stream check | detector respawns after 3s; relay has no respawn; camera absent blocks global readiness |
| map_server | navigation.launch:13 / roslaunch | /map/info startup; multifloor /map subscription | no configured respawn; map replacement is multifloor service client ownership |
| AMCL | navigation.launch:18 / roslaunch | TF wrapper; pose evidence in multifloor and mission | no configured respawn; arbitrary-pose recovery not established |
| move_base | wait_for_tf_exec.sh then navigation.launch:47 / roslaunch | /tf text match before exec; costmap startup messages | no configured respawn; TF gate has no total deadline |
| /navigation/cmd_vel | move_base remap -> supervisor subscriber | 0.25s supervisor freshness, ros_entrypoint:72 | stale input emits zero; actual sole-publisher enforcement not established |
| Robot WebSocket | supervisor RobotTransport via run.sh tunnel | send outage budget; mode response checks | transport FAULT latched; no tunnel reconnect loop; firmware disconnect behavior UNVERIFIED |
| mission_manager | system.launch:69 / roslaunch | action connection + floor/supervisor heartbeat | no respawn; action cancellation/recovery deferred to C |
| multifloor_manager | system.launch:51 / roslaunch | action connection + state heartbeat | FAULT path and READY prerequisite identified, recovery deferred to C |
| stair_supervisor | system.launch:60 / roslaunch | action connection + state heartbeat | timer shut down on transport fault; no reset/reconnect ROS interface in read boundary |
| RViz | system.launch:81, optional / roslaunch | no global readiness check | no respawn; rviz=false supported by launch but not exposed by run.sh |
| astra-web.service | external Mini PC user service per docs:7-12 | manual service/process inspection in docs:86-97 | run.sh contains no Astra service-state check; live service ownership UNVERIFIED |

## Topics and command ownership

OBSERVED: navigation.launch:49 remaps move_base output; ros_node.py (stair):91-102 subscribes NAV and odometry; supervisor.py:80-112 accepts NAV only in NAV, ignores nonfinite inputs, fences local ownership epoch and zeros absent/stale NAV. Stair execution owns the same transport in STAIR and increments epoch after successful/cancelled finish (supervisor.py:148-153,218-228).

OBSERVED: multifloor ros_node.py:87-94 subscribes map, tags, AMCL, scan, odometry and global costmap. It publishes initialpose and calls change_map, nomotion_update, clear_costmaps. Mission ros_state.py:59-84 observes floor, supervisor, odometry and AMCL; ordinary health:151-166 only checks floor/supervisor state and receive freshness.

UNVERIFIED: actual /livox/imu, lidar and camera publisher processes, external TF authority, Astra command path, firmware watchdog, and whether other clients can command the robot concurrently.

## Provisional findings to consolidate after C–F

- B-01 OBSERVED / Safety S1 / Mobility M3: capability-wide startup dependencies. run.sh:70-73,105-107,225,348-360 makes camera/tag/stair readiness global. The camera-specific remote reuse failure also restarts the entire sensor stack. Narrow readiness and restart scope to requested capability. Need camera-missing flat-navigation and healthy-lidar camera-restart isolation tests; do not conflate no visible tags with no detector messages.
- B-02 OBSERVED / Safety S1 / Mobility M3: transport fault has no in-process recovery. robot_transport.py:81-98,185-189; ros_node.py (stair):145-151; supervisor.py:234-248. run.sh:382 has no tunnel recovery. Transient sends inside outage budget are tolerated, so 'every packet loss faults immediately' would be false. Need bounded reconnect with invalidated old command epoch and verified zero/ownership before rearm; physical reconnect safety UNVERIFIED.
- B-03 INFERRED / Safety S3 / Mobility M1: cleanup can remove transport before shutdown-zero delivery. run.sh:202-207 sends master then tunnel then system signals; robot_transport.py:261-276 relies on connected transport for zero frames. No wait for supervisor stop acknowledgement precedes tunnel removal. Actual persistence of motion on disconnect depends on unverified firmware watchdog. Preserve transport until bounded stop attempt completes; require safe fake-transport signal-order test before hardware validation.
- B-04 OBSERVED / Safety S1 / Mobility M2: navigation launch wrapper couples exec to RViz XMLRPC helper. wait_for_tf_exec.sh:20-25 only matches map frame text (does not check source_frame connectivity) then synchronously invokes rviz_tf_reconnect.py:25-42 with no explicit timeout. Helper targets /rviz_navigation but launch names /rviz. INFERRED: absent node usually raises and proceeds; a reachable but hanging endpoint may indefinitely block move_base. Narrow to actual TF readiness and make visualization independent; no need for a new node.

## Phase record

Read files: 36 total in coverage.json, comprising initial baseline/launch/validator files plus all three node entrypoints, package entrypoints, node construction, mission ROS state, supervisor transport/client and ownership core, TF/RViz helpers, camera relay, and Mini PC operations document. No tests executed.

Confirmed facts: ownership and startup/cleanup paths above. Remaining inference: physical stop behavior, live sensor provenance, post-reboot ROS connections, failure timing in deployed network. New targets: floor FAULT recovery; mission anchor and cancellation; stair physical evidence; shutdown transport ordering; TF helper stall; production stair configuration. Remaining phases: C, D, E, F.
