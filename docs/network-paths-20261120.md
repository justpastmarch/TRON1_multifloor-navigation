# robotnet / Tailscale 연결 경로 자동 선택

2026-11-20 적용. 계단 건물(robotnet)에서는 기존 LAN 주소를 그대로 쓰고, 다른 장소에서는 같은 기기를 Tailscale로 연결한다. 정지 조건이나 게이트는 추가하지 않았다.

## 선택 규칙

`config.env`를 읽을 때 정한다.

1. 미니 PC LAN 주소 `192.168.1.56`이 ping(1 s)에 응답하면 `lan`.
2. 아니면 Tailscale 주소 `100.86.246.109`가 ping(2 s)에 응답하면 `tailscale`.
3. 둘 다 응답이 없으면 `lan`. 미니 PC가 부팅 중일 수 있으므로 `run.sh`의 기존 부팅 대기를 그대로 쓴다.

`TRON_NETWORK=lan` 또는 `TRON_NETWORK=tailscale`로 직접 지정할 수 있다. 정한 값은 export되어 하위 스크립트가 다시 검사하지 않는다. `run.sh`와 `run_mapping.sh`는 시작할 때 `[NET] <경로> path: mini PC ..., ROS master ...`를 출력한다.

| 경로 | MINI_PC_HOST / MINI_PC_ROS_IP | ROS_MASTER_HOST |
|---|---|---|
| lan | 192.168.1.56 | 192.168.1.26 |
| tailscale | 100.86.246.109 | 100.78.87.6 |

로봇 WebSocket은 원래 미니 PC를 거치는 SSH 터널이라 두 경로 모두 같은 방식으로 동작한다.

## 미니 PC

`watch_master.py`는 `ROS_MASTER_URI`(LAN)와 `ROS_MASTER_URI_ALT`(Tailscale)를 이 순서로 번갈아 확인한다. 먼저 응답한 master에 맞춰 `ROS_IP` 또는 `ROS_IP_ALT`로 센서 launch를 실행하고, 그 master만 감시한다. master가 끊기거나 바뀌면 기존과 같이 종료하고 systemd 재시작 후 다시 두 주소를 찾는다.

`~/.config/tron1-sensors.env`에 추가한 줄:

```
ROS_MASTER_URI_ALT=http://100.78.87.6:11311
ROS_IP_ALT=100.86.246.109
```

배포 전 파일은 `~/.local/lib/tron1-sensors/watch_master.py.bak_20261120_100633`, `~/.config/tron1-sensors.env.bak_20261120_100633`에 있다.

## OBSERVED (2026-11-20, 미니 PC는 휴대폰 핫스팟)

- 자동 선택 결과 `tailscale`, 판정 시간 약 1.1 s.
- 미니 PC 로그: `Master ready at http://100.78.87.6:11311 (ROS_IP 100.86.246.109)`.
- 개발 PC 수신: `/scan` 10.0 Hz, `/tron/wheel_odom_raw` 100.3 Hz, `/tf` 100.5 Hz. 수신 시각 − 메시지 시각 p50/p95/max: scan 0.052/0.141/0.298 s, odom 0.065/0.125/0.236 s, tf 0.090/0.181/0.320 s(두 PC 시계 차 포함).
- Tailscale 직접 연결 시 ping 49~107 ms. 중계(DERP) 시 더 느리고 핫스팟이 끊긴 사례가 있었다.
- LAN 경로는 robotnet 복귀 후 `[NET] lan path` 출력으로 확인해야 한다(미실시).

## 운용 주의

- Tailscale 경로는 지도 작성과 조종기 주행용이다. 자율주행·계단은 robotnet에서 사용한다.
- 미니 PC에 Tailscale SSH "check" 모드가 켜져 있으면 승인 기간이 지난 뒤 SSH가 브라우저 승인을 기다리며 멈춘다. `run.sh`가 미니 PC 확인 단계에서 멈추면 터미널의 login.tailscale.com 주소로 승인한다.
