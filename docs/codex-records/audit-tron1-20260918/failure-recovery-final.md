# Phase E ownership closure — startup, shutdown and ten failures

대상 root: `/home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation`. 아래 repository 상대 경로의 기준이다. 이는 Phase E 정적 failure-boundary matrix이며15 E2E의 최종 판정이 아니다. 운영 명령·ROS·SSH·로봇 명령을 실행하지 않았다.

## 공통 경계와 시간 해석

OBSERVED repository: `run.sh:212,224,236,246`은 local master→원격sensor재사용/전체재시작→tunnel→system.launch 순서다. :218은master,:240은tunnel을시작중한번검사한다. :348–368은action/state/sensor/freshness/publisher수의시작gate이며:382부터는system_pid만wait한다. **운용 후 각 master/tunnel/sensor component를 반복 점검하고 복구하는 loop는 없다.** :225의원격재사용검사는5개topic을각5초간읽고하나라도실패하면wf_mapping전체restart를요청한다. sensor내부process/respawn/TF생산은외부workspace라UNVERIFIED다.

OBSERVED installed ROS upstream: `/opt/ros/noetic/lib/python3/dist-packages/roslaunch/core.py:432–434`의기본respawn=false/required=false와`pmon.py:560–625`의required사망시shutdown·명시respawn만재시작동작을재확인했다. repository의map_server/amcl/move_base/세applicationnode/RViz는required/respawn설정없음. `apriltag.launch:21` detector만respawn=true/3초이며camera_info relay에는없다. 따라서한node죽음이항상전체launch종료라는주장은틀리다. 모든process종료등다른조건은별개다.

OBSERVED repository: `supervisor.py:95–109`는마지막NAV명령수신후0.25초를초과하면zero를만들고기본40Hz timer가send한다(`ros_entrypoint.py:72–73`, `ros_node.py:103–106`). 이는 **유한한정지거리나실제0.275초이내정지보장아님**이다. thread scheduling,ROS timer,send성공,firmware 및기계응답은UNVERIFIED다. `robot_transport.py:177–183`과`robot_tolerances.py:21–23`은마지막성공send후watchdog0.25초초과를실패시검사한다. OS가send성공으로받았지만로봇이수신하지못한경우의end-to-end감지상한은입증되지않았다.

OBSERVED: `ros_state.py:151–166`의ordinaryhealth는floor/supervisor의수신freshness및FAULT/connected를검사한다. scan/odom/camera를매tick감시하는전역runtimegate가아니다. `ros_segments.py:83–96`는segment진입때health를평가한다. `navigation_executor.py:290–298`은timeout없는resultwait다. 설치actionlib `simple_action_client.py:121–140`는zero/defaulttimeout을무기한으로정의한다. “2초statefreshness”를모든activegoal의2초failure감지상한으로말하면안된다.

OBSERVED shutdown: `run.sh:202–207`은pids등록순서master→tunnel→system에INT를보내고2초후동일순서TERM을보낸다. supervisor의normalclosezero는`robot_transport.py:261–276`에서연결을필요로한다. INFERRED: tunnel을먼저해제하면뒤supervisor의zero전달이실패할수있다(B-03 Safety S3/Mobility M1). 물리정지와firmwarewatchdog는UNVERIFIED다. 원격sensor는localcleanup대상이아니며계속남는것이문서계약이다.

## 열 가지 장애의 감지와 영향

시간은config/code predicate의값이다. 프로세스·네트워크·로봇의실측deadline은관찰하지않았다. S/M은기존finding의적용범위이며새root cause10개를만들지않는다.

| 장애 | 시작 중 처리(OBSERVED) | 운용 후 감지시간·정지범위 | 자동복구와 근거 |
|---|---|---|---|
| 1. wf_mapping 전체 종료 | :225에서5topic중하나라도5초probe실패하면전체restart요청. 후속readygate통과해야시작완료 | wrapper의직접사망감지상한없음. scan/odom/TF/camera가함께멎는경로는INFERRED,실제child생존여부UNVERIFIED. 진행STAIR에이전odom이있으면0.12초gap초과후다음evidence평가에서FAULT/zero시도. NAV는sensor/TFupstream동작과명령freshness에의존;즉시전체정지보장없음 | wrapper운용중restart없음. 외부wf_mapping의childrespawn UNVERIFIED. B-01 S1/M3, D-01 S2/M1, C-04 S4/M2 |
| 2. LiDAR만 종료 | /livox/lidar probe실패가wf_mapping전체restart유발 | LiDAR→scan공급중단시scan단독watchdog는현재costmap설정에없음. TF도늙으면pose획득이실패할수있지만TFfreshness와scanfreshness는서로다름. NAV의정지deadline UNVERIFIED. odom계속정상이면STAIR가LiDAR부재만으로즉시중단되는gate는없음 | wrapper복구없음;LiDAR드라이버respawn외부UNVERIFIED. D-01 S2/M1, B-01 S1/M3 |
| 3. odom bridge만 종료 | /tron/wheel_odom_raw probe실패→전체sensorrestart | 진행STAIR: lastodom기준0.12초gap(0.20초freshness보다먼저)후다음loop평가에서FAULT. 최초odom자체가없으면waiting상태로profiletimeout300초경로도존재. NAV는odom→TF외부생산관계에따라pose실패가능,ordinarymissionhealth는odom안봄. handoff/floor localization은freshodom부재로거부 | terminalSTAIRFAULT자동reset없음. bridge내부restart는외부UNVERIFIED. C-04 S4/M2, C-02 S2/M3 |
| 4. camera만 종료 | image/info부재가전체sensorrestart유발;tagstream부재가wrapper전체startup차단 | 운용NAV에camera즉시stopgate없음. floortransition태그확인에는기본tagtimeout10초(:83)가작용하고이후FAULT가능. 활성STAIR는odom기반이라camera상실만으로즉시멈춘다고말할수없음. 실제전체정지상한없음 | detector사망은3초respawn하나카메라/relay복구와다름. 카메라driver복구외부UNVERIFIED. B-01 S1/M3 |
| 5. ROS master 종료/재시작 | ownmasterPID및rostopiclist확인실패면시작중단(:214–220) | ready후wrappermaster감시없음. 기존TCPROS연결이얼마나유지되거나새master에재등록되는지는이번runtime에서UNVERIFIED. master죽음만으로모든command가즉시끊긴다고주장하지않음. actionterminal부재시상한없는BUSY가능(C-01) | master재시작·epoch재동기화자동경로없음. 기존프로세스가새master등록·params·latchedstate를정확히회복하는지미검증. C-01 S2/M3, C-11 S2/M2 |
| 6. SSH tunnel 종료 | 시작2초뒤PID부재면종료(:239–243) | ready후PID감시없음. command send실패가관찰되면0.25초outagebudget검사후FAULT;mode요청에는별도requesttimeout기본8초. 이는실제disconnect검출상한아님. FAULT에socketclose/timer중지,새NAV거부. 끊긴통로를통한zero전달및기체정지는UNVERIFIED | tunnelrestartloop없고RobotTransport.start는재시작불가. B-02 S1/M3, B-03 S3/M1 |
| 7. stair_supervisor FAULT | stateNAV기다리는startupgate불통과 | softwareFAULT감지는발생경로별즉시publish또는0.5초stateheartbeat. mission은다음segment진입health에서거부하나이미대기중인NAV가항상그시간내끝나는것아님. supervisor NAV출력막고socket닫음. connected정상상태의zero시도는존재하나fault이후전달보장은없음 | reset/rearm/reconnectROSinterface없음. 새로운process가필요한구조이나재시작후실제pose·epoch복원절차완결성없음. B-02 S1/M3,C-01 S2/M3,C-04 S4/M2 |
| 8. move_base 종료 | move_baseactionconnection과costmapinfo가준비되지않아gate불통과 | publisher정말사라지면마지막NAV명령0.25초초과후다음40Hztickzero시도. move_base자체종료를감지해missionterminal로바꾸는deadline없고result없는대기는계속될수있음. 다른node·sensor·RViz자동종료안함 | required=false/respawn=false. retry는ABORTED/LOST등terminal을받아야진행하므로사망후무조건retry아님. C-01 S2/M3 |
| 9. RViz 종료 | system.launch의rviz=false지원;wrapper에RVizreadygate없음. TFhelper는별도B-04경계 | GUI소실. 이미실행중인mission/NAV/stair를RViz사망만으로stop시키는코드없음. launch사망감지poll은upstream에있지만물리정지목표와무관 | RVizrespawn없음. manualUI만다시올리는경로와managedviewer재생성은구분해야함. B-04 S1/M2 |
| 10. Mini PC reboot | SSH/connect또는sensorprobe불능이면startup불가;기동가능한경우다음run.sh가sensorstart요청 | sensor/TF/SSHtunnel이함께영향받는것은INFERRED. 각각위1/3/6경계로전파;네트워크검출과firmware실물정지상한UNVERIFIED. workstationmaster/application은별도machine이므로함께자동종료한다고단정불가 | 문서상Astraautostartdisable,현재stack은매번workstation ./run.sh로시작. 실행중run.sh의rebootrecoveryloop없음. B-01 S1/M3,B-02 S1/M3,C-11 S2/M2 |

센서수치근거: `stair_profiles.yaml:18–22,39–43,57–61`; `stair_evidence.py:141–155`; `supervisor.py:166–189`. scan-loss근거: `costmap_common_params.yaml`에는expected_update_rate없음; 공식ROS Noetic obstacle_layer.cpp:97의default0.0와observation_buffer.cpp:231–234의always-current, move_base.cpp:829–833의noncurrentzero를대조했다. 원문hash/URL은OUT/upstream/manifest.json에있다. TF는costmap_2d_ros.cpp:546–591의별도pose/age검사이며repository global/local costmap transform_tolerance각0.5초다. TF실패로NAV가멎을수있다는것이scan독립watchdog가존재한다는뜻은아니다. 실제설치binary와외부TFwriter경로는UNVERIFIED다.

## 실제 존재하는 수동 경로와 최소 변경 제안

아래는문서/코드에명령이있다는검증이다. 실행지시·이번실행·복구성공보증이아니다. **지원되지않는resetservice,재접속옵션,특정driver launch명을발명하지않았다.** 공통fullrestart는기존설정의initialfloor/pose/anchor를재적용하므로C-11과현장위치확인이선행돼야한다. 단순restart가고장직전mission을안전하게resume한다는증거는없다.

| 장애 | repository에 존재하는 수동 명령/절차 | 없음·한계 | 최소 범위 변경 제안(INFERRED) |
|---|---|---|---|
| wf_mapping | docs/mini-pc-stack-switching-ko.md:302–306의tmux종료및정확한roslaunch패턴INT/TERM, :312–313종료확인; :117–130의 ./run.sh 재구성 | 개별child health/owner inventory없음. 전체sensorrestart절차만있음 | 센서owner가실패한child만재시작하고scan/odom/TFepoch를다시검증. 외부driver지원확인전실제명령은미정 |
| LiDAR | 위전체sensorstop/start와README.md:287–289의publisher/time확인 | LiDAR단독restart명령은현재repo문서에없음 | scanstaleness에영향받는NAV만zero/pause하고LiDAR/converter의실제실패component만복구;camera정상process유지 |
| odom bridge | 전체sensorstop/start 및docs:187,289의odom관찰 | bridge단독restart와STAIRFAULTrearm명령없음 | bridgeowner복구후odom재기준·freshsample·floorpose·admission재검증;계단중자율재개기능을무조건추가하지않음 |
| camera | 전체sensorstop/start;README.md:290의camera remap/tagartifact확인 | camera-onlyrestart명령없음. detector3초respawn이camera재시작을대체하지않음 | camera/tag에의존하는floor/stair-admission만차단하고평지NAVcapability별gate. driver/relay/detector중고장component만재시작 |
| master | docs:117–130의 ./run.sh startup와:295의Ctrl+C shutdown | master-only재시작후전체registration/params/actiongeneration회복명령없음 | master상실을통합owner가감지해새goal차단,transport살아있을때boundedzero,master/state의명시적freshgeneration재구성 |
| tunnel | docs/legacy-rviz-viewer.md:52에foregroundSSH -L 명령존재;managedlane는run.sh:236 | legacy별도lane명령이며managedstack와동시실행해선안됨(:3–5). tunnel만재생성해도latchedtransport가회복안됨 | transportowner의새session만만들고oldcommand/token폐기·zero/firmware/ownership확인뒤명시rearm. globalmaster/sensor재시작은불필요한범위 |
| stair FAULT | docs/legacy-rviz-viewer.md:68에standalone node시작명령,README.md:275,278–279에점검/수동복구문구 | reset/rearm명령없음. standalone명령은managed기동중중복owner를만드는복구명령으로쓰면안됨 | 같은owner안에검증된fault분류·명시rearm단계. transport상태·실제정지·현재floor/anchor가검증되기전동작재개금지 |
| move_base | legacy-rviz-viewer.md:37의navigation.launch 명령존재;managedlane전체 ./run.sh | navigation.launch는map_server/AMCL/move_base모두기동;실행중managedstack에중복실행하는targeted복구명령아님. move_base-onlyrestore절차없음 | childserver상실deadline→해당missionterminal/zero. 필요component재시작후currentmap/localization유지·새goalidentity로재요청 |
| RViz | legacy-rviz-viewer.md:86의manualviewer명령,late-publisher-troubleshooting.md:193의reconnecthelper명령 | manualviewer는directgoal도구가있어managedviewer와동등하지않음. helper는이미있는RVizconnection수정이고deadRViz를기동하지않음 | managedviewer만재생성하는지원절차;navigation실행과visualizationhelper분리. mission/master/sensorrestart불필요 |
| Mini PC reboot | mini-pc-stack-switching-ko.md:297,365–367은다음 ./run.sh로sensorstart. :197–227은processROSenv확인 | 실행중자동rebootrecovery없고복구후pose/commandepoch승계계약없음 | mini-PCsensor/tunnel의새epoch만재구성하고commandownerfreshhandshake·정지·localization확인. workstation영향없는component보존 |

## 종합 및 잔여 한계

OBSERVED: 시작체크는많지만운용중processdeath감지와재기동소유권은약하다. INFERRED: 단일component장애가전체재시작으로확대되거나(C-11초기상태재적용),application을살려둔채terminal없는BUSY/FAULT로고착될수있다(B-02/C-01/C-02). 좁은복구는고장component와그증거generation을복원하고새명령을명시적으로재승인하는범위다. hardwarewatchdog,물리정지,외부sensorstackrestartgranularity는UNVERIFIED다.

E마감용새정적근거만추가했으며phaseF/E2E판정을대신하지않았다. 생성물은이retainedMarkdown1개,임시artifact0,target쓰기0,실행시험0이다. 새process는foregroundread명령뿐이며종료됐다.
