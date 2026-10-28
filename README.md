# quest_arm_ik — Meta Quest 3 → 양팔 14축 휴머노이드 상반신 원격제어 (pink / Pinocchio + MuJoCo)

MoveIt Servo 대신 QP 미분 IK를 직접 돌리는 ROS 2(ament_python, Jazzy) 패키지.
특이점·관절한계·자기충돌 근처에서 **멈추지 않고 느려질 뿐**이며, 타임스탬프에 의존하지 않아 sim time 문제가 없다.
모델은 URDF / xacro / MuJoCo MJCF(전신 모델이어도 됨) 그대로 사용. **SH-Humanoid(삼현 2세대) 저장소와 바로 맞물리게 되어 있다.**

```
SH-Humanoid Unity  SHPOSE UDP 15000 / SHABS UDP 15001 ─┐
팀 udp_pose_bridge  /teleop/{left,right}_wrist_pose      ├─▶ quest_bridge ─▶ /teleop/{side}_hand/{target_raw,engaged}
Isaac ROS 4.6 isaac_ros_teleop  /xr_teleop/ee_poses+controller_data │                         │
자체 Quest 브리지 (PoseStamped + Joy) / quest_mock       ─┘        dual_arm_ik (클러치 매핑 + One-Euro + pink IK + 자기충돌, 100 Hz)
                                                      │          ├─▶ /left|right_arm_forward_controller/commands  (팀 ros2_control / mujoco_sim)
                                                      │          ├─▶ /left|right_arm_controller/joint_trajectory   (JTC)
                                                      │          └─▶ /teleop/joint_targets (JointState: position + 중력보상 effort, 100 Hz)
                                                      │                └─▶ upper_body_streamer (50 Hz, 속도 제한·FF·워치독) ─▶ /upper_body/command (실기체 HRController, 1 kHz)
                                                      │                      ◀── /upper_body/motor_state (측정 모터각 → IK 초기화·재동기화, 스트리머 추종 감시)
                                                      └─▶ /teleop/{side}_hand/grasp (트리거 0~1) ─▶ hand_grasp ─▶ /teleop/hand_joint_targets (Dex3 손가락 14관절)
```
그립(측면 버튼) = 클러치, 트리거(앞쪽 버튼) = 손 쥐기. 시작 자세는 두 로봇 다 팔을 옆으로 내린 자세(`mujoco_sim.initial_posture`)라 조작자도 같은 자세로 그립을 쥐고 시작하면 된다.

## SH-Humanoid 저장소와의 관계 (meta_quest/sh_gen2_teleop 대비)
| 팀 저장소 | 이 패키지 | 비고 |
|---|---|---|
| `udp_pose_bridge.py` (SHPOSE UDP→PoseStamped) | `quest_bridge input_mode: shpose_udp` 가 내장 (또는 `team_topics` 로 팀 노드 출력 그대로 사용) | 패킷 형식 동일, Unity 수정 불필요 |
| `abs_input.py` (SHABS 15001, 유니트리식 절대) | `quest_bridge input_mode: shabs_udp` (그립 플래그로 클러치) | 머리 기준 재매핑은 하지 않음 — 상대 매핑이라 불필요 |
| `pink_ik_bridge.py` + `servo_pose_bridge.py` | `dual_arm_ik` (+ `mapper.py`, `ik_core.py`) | 가중 posture·limit_margin·max_step·resync·중력 FF 모두 유지. 손으로 재던 캡슐 충돌 필터 → URDF 충돌 메시(convex hull) + 스탠드 기본도형 QP 제약으로 대체 |
| `quest_ik.yaml` | `config/sh_humanoid_v2.yaml` | 팀 값(관절 순서, teleop_ready, delta_scale 0.85, posture_weights, margin 0.03) 이식 |
| MoveIt `demo.launch.py`(RViz + ros2_control mock) | `mujoco_sim` 또는 그대로 사용 (`sim:=false`, 출력이 팀 forward 컨트롤러 토픽) | `ready.sh` 로 forward 컨트롤러 전환 후 실행 |
| `ros2_interfaces` (UpperBodyCommand 등) | `output_mode: upper_body` / `joint_states_source: upper_motor_state` | 모터 ID·부호·오프셋은 docs 에 없음 → 실측 후 config 에 기입 |


| 파일 | 역할 |
|---|---|
| `quest_arm_ik/ik_core.py` | **ROS 비의존 IK 코어** — 손 FrameTask×2 + PostureTask + 관절 한계 + active-set 자기충돌 제약. URDF/MJCF 로드, 전신 모델이면 14축만 남김 |
| `quest_arm_ik/mapper.py` | **ROS 비의존 매핑** — 클러치(engage) 상대 매핑, 스케일, 요 정렬, One-Euro/slerp 필터, 입력 timeout |
| `quest_arm_ik/mujoco_sim.py` | **MuJoCo 상반신 시뮬** — 전신 MJCF의 베이스를 공중 고정, 다리는 키프레임 유지, 14축 position 제어, /joint_states 발행, 뷰어 |
| `quest_arm_ik/dual_arm_ik_node.py` | ROS 노드 — 위 둘을 묶고 관절 명령 발행 |
| `quest_arm_ik/quest_bridge.py` | 입력 어댑터 — `shpose_udp`/`shabs_udp`(SH-Humanoid Unity UDP) / `team_topics` / `xr_teleop`(Isaac ROS) / `pose_joy` 한쪽 입력이 100 ms 넘게 끊기면 `Quest 입력 끊김` WARN(v0.7.57) |
| `quest_arm_ik/protocols.py` | SHPOSE/SHABS 패킷·msgpack 파서 (ROS 비의존) |
| `quest_arm_ik/urdf_tools.py` | xacro 전개, `package://` 해석, 링크 충돌형상을 기본도형으로 교체 — IK 와 시뮬이 같은 전처리 결과를 씀 |
| `quest_arm_ik/diag_teleop.py` | 실행 중 진단: 노드 중복·발행자 수·IK 명령 vs 측정 팔꿈치각·엔게이지 토글·입력 수신률 (1 Hz) |
| `quest_arm_ik/xr_dump.py` | `/xr_teleop/*` 규약(포즈 순서·frame_id·msgpack 키)을 한 번 찍어 보는 도구 |
| `quest_arm_ik/quest_mock.py` | 헤드셋 없이 시험하는 가짜 컨트롤러 — 그립 → 손 들어 올리기 → 원운동 + 쥐기/펴기 반복 (`mock_traj.py` 가 ROS 비의존 궤적) |
| `quest_arm_ik/hand_grasp_node.py` | 트리거(0~1) → Dex3 손가락 관절 목표 `/teleop/hand_joint_targets` (`hand.py` 가 ROS 비의존 매핑: 열림/쥠 각, 저역통과) |
| `tools/hr_controller_v2/` | **삼현 HRController 시연용 패치**(`hr_controller_demo.patch`, 미니PC 배포 브랜치 feature/100hz-control 기준 7파일, `git apply` → `cmake --build build/release`): Thor 직결 UDP 링크를 제어기에 내장(파이썬 브리지·JSON·IPC 경로 제거, STATE 100 Hz, 메인 루프 poll 1 ms), enable 때 위치 루프 OD 튜닝 쓰기/읽기 로그. 배포 브랜치는 이미 CAN 지령 200 Hz. 링크 모듈 소스(`hr_udp_link.hpp/.cpp`)와 가짜 제어기(`hr_udp_link_fake_controller.cpp`)는 `test/test_udp_bridge.py --cpp` 가 g++ 로 빌드해 릴레이와 맞춰 본다 (런북 6-1e) |
| `quest_arm_ik/hr_udp_relay_node.py` + `hr_udp_proto.py` + `tools/hr_udp_bridge.py` | **실기체 UDP 링크(권장)** — Thor 의 릴레이가 삼현 브리지와 같은 `/upper_body/*` 토픽·서비스를 제공하고, 미니PC 의 `hr_udp_bridge.py`(ROS 불필요, 표준 라이브러리)가 HRController IPC·상태 파일과 UDP 로 잇는다. DDS 를 링크에 태우지 않아 Fast DDS 발행 블로킹(2026-10-21)을 피함 STATE 가 100 ms 넘게 끊기면 seq·제어기 시각으로 유실/미니PC 정지/지연을 갈라 WARN(v0.7.57) |
| `quest_arm_ik/upper_body_streamer_node.py` | **실기체 스트리머** — (v0.7.47 제동 거리 법칙: 목표 속도 피드포워드 + √(2·a_max·|e|), 멈춘 목표 둘레 진동 없음) IK 관절 목표(100 Hz) → 삼현 HRController `UpperBodyCommand` 50 Hz 기본(미니PC 배포 제어기의 CAN 지령 슬롯은 200 Hz, IK 가 100 Hz 이므로 실기체에선 `rate: 100.0` 권장, 200 도 verify 로 시험 가능). 첫 명령 = 측정 모터각, 틱당 v_max·dt 제한, 속도 피드포워드, IK 끊김·추종 오차·`/teleop/hold` 시 정지, 정지에서 재개할 땐 목표까지 `v_catchup` 0.3 rad/s 로 접근(v0.7.55; 추종 오차 정지는 명령을 측정값으로 되돌림, 3회면 정지 유지), 상태를 `/upper_body/streamer_state` 로 발행, motor_states/IK 목표/자기 틱이 100 ms 넘게 끊기면 WARN(v0.7.57) (`upper_body.py` 가 ROS 비의존 로직: MotorMap·Streamer·fill_command) |
| `quest_arm_ik/choreography.py` / `motion_show.py` | **동작 쇼**(v0.7.56): 하트·양손 인사(와이퍼, 손바닥 정면)·달리기(±60°, 2회)·손 키스 안무(관절 공간 키프레임 + 사인, 최소 저크 전환; 손-손(손가락 포함) ≥ 90 mm 로 실기체 오차에도 닿지 않게)와 그것을 `/teleop/joint_targets` 로 내는 노드 — 스트리머가 `/upper_body/streamer_state` 로 commanding=1 을 알린 뒤 그 순간의 측정 자세에서 시작, 스트리머가 멈추면(disable·추종 오차·hold) 쇼도 일시정지했다 이어감. 실기체 `launch … input:=show hr:=true sim:=false relay:=false`, 시뮬 리허설 `hr:=true sim:=true`(릴레이 끄고); `tools/render_show.py` 로 ROS 없이 GIF 미리보기 (런북 7-2) |
| `quest_arm_ik/motor_calib.py` | 실기체 모터 매핑·드라이버 동작 실측 도구 — `watch`(실시간 표) / `zero`(오프셋) / `jog N 0.1`(ID·부호; `--once --duration` 으로 duration 의미) / `hold`(명령 수신) / `watchdog`(명령 끊김 시 동작) / `stiffness`(kp 단위·실효 강성) / `verify`(관절별 기대 방향 시험 + 추종 요약; `--delta2 0.5` 양방향 ±0.5 rad(어깨 roll 은 바깥쪽만, URDF 한계 안으로 자름), `--velocity 0.5` 피크 속도, `--rate 100`) |
| `quest_arm_ik/frames.py` | 좌표계 변환식(Unity/OpenXR→ROS), One-Euro 필터, slerp 저역통과 |
| `config/sh_humanoid_v2.yaml` | **삼현 2세대 프로필** (팀 관절 순서·teleop_ready·컨트롤러 토픽·스탠드 충돌형상·실기체 매핑 자리) |
| `config/dual_arm_ik.yaml` | G1 29-DoF 프로필 |
| `test/test_sh_chain.py` | SH 모델: Unity SHPOSE 스트림→파서→매핑(relative/absolute)→IK→MuJoCo, 스탠드 기둥 쪽으로 밀어도 30 mm 에서 정지 |
| `test/test_ik_core.py` | 합성 URDF로 IK 코어 검증 (모델 파일 불필요) |
| `test/test_g1_sim_loop.py` | G1 MJCF: Pinocchio FK = MuJoCo FK, IK→시뮬 추종, 자기충돌 30 mm 유지·100 Hz 예산 |
| `test/test_teleop_chain.py` | G1 MJCF: 컨트롤러→매핑→IK→시뮬 전체 체인 (추종·회전·release·re-engage) |
| `test/test_xr_chain.py` | Isaac ROS `/xr_teleop` 스트림 흉내(ROS 축·squeeze 클러치·비활성 포즈) → 요 오프셋 → IK → MuJoCo, G1(0°)·SH(90°) |
| `test/test_rest_start.py` | 팔을 내린 시작 자세에서 mock 궤적(들어 올리기 → 원 → 쥐기)이 끝까지 추종되는지 — G1 / G1+손 / SH+Dex3 |
| `test/test_grasp.py` | 트리거 → 손가락 관절 매핑과 MuJoCo 쥐기/펴기 (SH+Dex3, g1_with_hands) |
| `test/test_protocols.py` | 패킷 파서·msgpack 키·OpenXR→ROS 변환이 isaac_ros_teleop 의 transform_rotation 과 일치하는지 (모델 불필요) |
| `test/test_motion_show.py` | 동작 쇼: 안무 속도·한계 여유, MuJoCo 자기충돌 0, 가짜 ROS 체인(motion_show → 스트리머 → HR 에뮬레이션) 추종 (SH_REPO 필요) |
| `test/test_upper_body.py` | 스트리머 로직: 매핑 왕복, 점프 금지, v_max·a_max 제한, IK 끊김 정지, 추종 오차 정지(되돌림·재접근·3회 정지 유지), 재개 시 v_catchup 접근, 15 슬롯 채우기, config 매핑 일관성 (모델 불필요) |
| `test/test_hr_chain.py` | **실기체 경로 리허설** — 가짜 rclpy(`test/fakeros`)로 mock→bridge→IK→스트리머→HRController 에뮬레이션(mujoco_sim)→motor_state 전체를 한 프로세스에서: config 주기·추종·워치독·hold·매핑 오류 시 자동 정지 |
| `scripts/check_model.py` | **새 로봇 모델 반입 검사** — 임의 URDF/MJCF + config 로 이름·FK 일치·IK 수렴·시뮬 추종·100 Hz 예산을 ROS 없이 검사 |

## 설치 (Thor: Isaac ROS 4.6 Docker 모드)
의존성(apt + pip)은 컨테이너가 아니라 **이미지 레이어**(`docker/Dockerfile.zz_quest_teleop`)에 굽는다 — 컨테이너는 재부팅마다 새로 만들어져 안에 설치한 것이 사라지기 때문.
```bash
# 호스트 (컨테이너 밖)
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/Sejong-Medical-Robotics-Lab/SH-Humanoid.git      # 팀 저장소 (description / moveit_config / ros2_interfaces)
rm -rf SH-Humanoid/integration/ros2_interfaces/{build,install,log}            # 커밋된 빌드 산출물 제거
python3 -m zipfile -e ~/quest_arm_ik.zip .                                     # 또는 cp -r /path/to/quest_arm_ik .
bash quest_arm_ik/scripts/setup_persistent.sh                                  # 레이어 등록 (1회)
docker stop isaac_ros_dev_container 2>/dev/null; isaac-ros activate --build-local   # 레이어 빌드 (1회; 끝에 'deps ok' 두 줄) → 컨테이너 진입
# 컨테이너 안
cd $ISAAC_ROS_WS && colcon build --symlink-install \
   --packages-select SH_Humanoid_v2_description SH_Humanoid_v2_moveit_config ros2_interfaces quest_arm_ik
source src/quest_arm_ik/scripts/env.sh                                         # 새 터미널마다 (install/setup.bash + G1/SH + QPY + PYTHONNOUSERSITE=1)
```
Thor 이미지의 시스템 python 은 NVIDIA isaac_ros_teleop 용(isaacteleop + dex_retargeting 0.4.6 + **pinocchio 2.7.0** + numpy 1.26)이라 우리 스택(pin 3.x, pink, mujoco, numpy 2)과 한 사이트에 공존할 수 없다.
그래서 레이어는 우리 의존성을 **전용 venv `/opt/quest_py`** 에 넣고(`cmeel-urdfdom<5`, `cmeel-tinyxml2<11` 캡 — pin 바이너리 soname 일치), `teleop.launch.py` 가 우리 노드만 그 인터프리터(`$QPY`)로 실행한다. rclpy 등 ROS 패키지는 `setup.bash` 의 PYTHONPATH 로 들어온다.
오프라인 테스트·`check_model.py` 도 컨테이너에서는 `$QPY`(= /opt/quest_py/bin/python3) 로 실행한다.
레이어를 못 쓰면 컨테이너 안에서 `bash scripts/setup_container.sh`(같은 내용을 직접 설치; 컨테이너가 새로 만들어질 때마다). 맥북 등 ROS 없는 환경은 `python3 -m pip install -r requirements.txt`.
단계별 전체 실행 순서(2단계 → 실기체 → 시연)는 **`docs/RUNBOOK.md`** 에 있다. IK 동작 튜닝(스케일·손목·자세 과제·회전 클러치)은 **`docs/IK_TUNING.md`**. 모니터 없이 맥북 ssh 만으로 운영하는 tmux 스크립트는 `scripts/thor_demo_tmux.sh`·`scripts/minipc_hr_tmux.sh`(런북 7-1).
G1 모델(MJCF, 메시 포함)은 MuJoCo Menagerie에서:
```bash
cd $ISAAC_ROS_WS/src && git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie && git sparse-checkout set unitree_g1        # → unitree_g1/g1.xml
```

## ROS 없이 먼저 검증 (맥북에서도 가능: `python3 -m pip install -r requirements.txt`; Thor 컨테이너에서는 아래 `python3` 대신 `$QPY`)
```bash
python3 test/test_ik_core.py
SH_REPO=/abs/SH-Humanoid python3 test/test_sh_chain.py          # 삼현 2세대: SHPOSE 스트림→매핑→IK→MuJoCo, 스탠드 충돌 정지
python3 scripts/check_model.py --model /abs/SH-Humanoid/urdf_moveit/SH_Humanoid_v2_description/urdf/SH_Humanoid_v2.xacro \
    --config config/sh_humanoid_v2.yaml --package-dirs /abs/SH-Humanoid/urdf_moveit \
    --srdf /abs/SH-Humanoid/urdf_moveit/SH_Humanoid_v2_moveit_config/config/SH_Humanoid_v2.srdf
G1_MJCF=/abs/mujoco_menagerie/unitree_g1/g1.xml python3 test/test_g1_sim_loop.py
G1_MJCF=/abs/mujoco_menagerie/unitree_g1/g1.xml python3 test/test_teleop_chain.py
G1_MJCF=/abs/mujoco_menagerie/unitree_g1/g1.xml SH_REPO=/abs/SH-Humanoid python3 test/test_xr_chain.py   # 경로 B 스트림 → G1·SH
G1_MJCF=/abs/g1.xml G1H_MJCF=/abs/g1_with_hands.xml SH_REPO=/abs/SH-Humanoid python3 test/test_rest_start.py   # 팔 내린 시작 자세 + 쥐기
SH_REPO=/abs/SH-Humanoid G1H_MJCF=/abs/g1_with_hands.xml python3 test/test_grasp.py
python3 test/test_protocols.py
```

## 다른 로봇(삼현 2세대 상반신 등)으로 바꾸기
G1 파일은 검증용 기본값일 뿐이다. 로봇이 바뀌면 **그 로봇의 URDF(또는 MJCF)** 를 `model_path` 로 넣고 config 의 이름만 맞춘다:
1. `controlled_joints` 14개(왼팔 7 → 오른팔 7, 어깨→손목 순), `left/right_hand_frame`(손목 끝 또는 손 링크), `base_frame`(URDF 루트 링크)
2. `nominal_posture` — 그 로봇의 축 부호에 맞춰 "팔꿈치 살짝 굽히고 팔을 약간 벌린" 자세로 재설정
3. `mujoco_sim.keyframe: ""` (URDF 엔 키프레임이 없음). 액추에이터·키프레임이 없는 순수 URDF 도 그대로 시뮬 가능 (position 액추에이터 `pd_kp/pd_kd` 자동 추가, fixed 링크 유지, implicitfast)
4. 메시가 `package://` 면 `package_dirs` 에 그 패키지가 들어 있는 디렉터리를 적는다 (IK 자기충돌·시뮬 뷰어 둘 다 사용)
5. 검사:
```bash
python3 scripts/check_model.py --model /abs/samhyun_gen2.urdf --config config/dual_arm_ik.yaml --package-dirs /abs/ros_ws/src
```
   전부 OK 가 나올 때까지 config 를 고친다. 자주 걸리는 것: 관절 한계가 없음(`<limit>` 추가), 손 링크가 fixed 로 붙어 MuJoCo 에서 사라짐(MJCF 변환본이면 URDF 로), 관성 누락(`<inertial>` 추가), 자기충돌용 `<collision>` 형상 없음(없으면 `enable_self_collision: false`).

## 실행 — 삼현 2세대 (robot:=sh, 모델·SRDF·메시 경로 자동)
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock        # 1) 헤드셋 없이: MuJoCo 팔이 정면(+y)으로 원을 그리면 OK
ros2 launch quest_arm_ik teleop.launch.py robot:=sh                    # 2) Unity ControllerPoseLogger (UDP 15000, 그립=클러치) → MuJoCo
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=shabs_udp   #    AbsolutePoseSender (UDP 15001) 를 쓸 때
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=team_topics #    팀 udp_pose_bridge 를 띄워 둔 채 그 토픽을 받을 때
ros2 launch quest_arm_ik teleop.launch.py robot:=sh sim:=false         # 3) 팀 demo.launch.py(RViz) 또는 실기체 ros2_control 로 출력
ros2 launch quest_arm_ik teleop.launch.py robot:=sh hand:=box          #    기존 박스 손 모델로 (기본은 G1 Dex3-1 3지 핸드 = 전시 사양)
ros2 topic echo /teleop/joint_targets                                   #    팀 약속 토픽 (position=명령각, effort=중력보상)
```
손 모델은 팀 저장소 `SH_Humanoid_v2.xacro` 의 `hand` 인자(`dex3`|`box`)로 고르며, `hand:=` 는 두 노드의 `xacro_args` 파라미터(`"hand:=box dex3_fingers:=revolute"` 식)로 전달된다.
플랜지 프레임 `*_hand_link` 는 두 손이 같아 IK 설정은 공통이다 (`SH-Humanoid/urdf_moveit/README.md` 참고).
Unity 쪽은 그대로: PoseLogger 의 `Ros2 Computer Address` 에 Thor IP, 그립을 쥔 동안만 패킷이 오고 그 순간이 클러치 기준점이 된다.
`mapping_mode: relative`(기본) 는 그립 순간 **로봇 손의 현재 위치**에서 변화량×0.85 만큼 움직이고(점프 없음), `absolute` 는 팀 Servo 브릿지처럼
Unity 가 보내는 중립 위치 기준으로 움직인다(그립 순간 준비 자세로 당겨짐).

## 실행 — Quest 3 를 Isaac ROS 4.6 `isaac_ros_teleop` 으로 받을 때 (경로 B, Unity·Windows 불필요)
```bash
sudo apt-get install -y ros-jazzy-isaac-ros-teleop                               # IsaacTeleop 1.3.x + CloudXR 런처 포함
ros2 launch isaac_ros_teleop isaac_ros_teleop.launch.py cloudxr_accept_eula:=true  # 터미널 A: CloudXR 런타임을 노드가 직접 띄움
#   Quest 브라우저: https://nvidia.github.io/IsaacTeleop/client/release-1.3.x/#/real/isaacros → Thor IP → 인증서 수락 → H.264 → Connect
ros2 run    quest_arm_ik xr_dump                                               # 터미널 B: /xr_teleop 규약 확인 (poses[0]=왼손, squeeze 키)
G1=$ISAAC_ROS_WS/src/mujoco_menagerie/unitree_g1/g1.xml
ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=$G1 input:=xr_teleop   # G1 로 먼저 (요 오프셋 0)
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop                   # 삼현 2세대 (요 오프셋 90 자동: ROS x 정면 → base_link y 정면)
```
`/xr_teleop/ee_poses` 는 [왼손, 오른손] 컨트롤러 포즈를 **ROS 축(x = Connect 순간 머리가 향한 방향, z 상방, 원점 바닥)** 으로 주고,
`/xr_teleop/controller_data`(msgpack) 의 `left/right_squeeze_value`(그립 버튼) 가 클러치가 된다. 조작자는 로봇(또는 화면 속 로봇)과 **같은 방향**을 보고 선다.
launch 인자: `robot` (sh | g1 | custom), `input` (xr_teleop | shpose_udp | shabs_udp | team_topics | pose_joy | mock), `engage` (grip | always), `yaw_offset` (deg),
`sim`, `viewer`, `viewer_view` (behind | front), `model_path`, `package_dirs`, `srdf_path`, `sim_model_path`, `config`, `forward_axis`.

## 실행 — G1 (MuJoCo Menagerie, 헤드셋 없이)
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=$G1 input:=mock
```

## 동작 방식
- **클러치**: engage 순간 컨트롤러 자세 `ctrl0`와 로봇 손 자세 `hand0`를 저장, 이후 변화량만 적용
  `p = hand0.p + s·R_cal·(p_ctrl − p_ctrl0)`, `R = R_cal·R_ctrl·R_ctrl0ᵀ·R_calᵀ·hand0.R`.
  절대 좌표 정렬이 필요 없고 놓았다 다시 잡으면 이어서(re-index) 조작. `engage_mode: always`면 첫 포즈에서 engage.
- **회전 클러치** (`orientation_clutch`, v0.7.35): `always` 는 그립 중 늘 컨트롤러 회전을 따라감(위 식). `trigger`(SH 기본) 는 **검지 트리거**
  (`/teleop/{side}_hand/grasp` > 0.5) 를 당긴 동안만 — 그 순간의 컨트롤러·손 자세가 회전 기준 — 따라가고, 그 외에는 손 태스크의 회전 cost 를 0 으로 하고
  **손목 3관절을 잠근다**(속도 한계 ~0). 사람과 팔 비율이 다른 로봇이 손 방향을 맞추느라 손목을 혼자 꺾는 것을 막는다. Dex3 손 쥐기와 같은 버튼이라 손을 쓸 때는 `always`.
- **팔꿈치 힌트** (`elbow_hint_cost`, v0.7.38): 그립 순간 로봇 전완 방향 d0 를 컨트롤러 회전 변화만큼 돌린 d 로 '손 목표 − 전완 길이·d' 를 팔꿈치 위치 과제(가중 0.5)에
  준다 → 여유 자유도(스위블)가 사람 전완 방향을 따라간다. trigger 클러치에서 손목을 돌리는 동안은 끔. 0 = 끔.
- **IK**: 매 주기 pink `build_ik` → 활성 충돌쌍 제약 추가 → QP → 관절속도 → 적분. 손 태스크는 가중 최소제곱이라 도달 불가 목표는 가장 가까운 자세로 수렴.
- **자기충돌**: 메시 충돌형상을 convex hull로 바꾸고(G1 메시 그대로는 pair당 ~1 ms라 불가), 2홉 이내·기본자세에서 가까운·항상 겹치는 쌍을 제거한 뒤, 매 주기 거리만 전부 계산하고 `collision_activation` 안의 쌍만 `-nᵀ(J₁−J₂)Δq ≤ gain·(d−d_min)` 제약으로 넣는다 (G1: 287쌍, 평균 1.9 ms).
- **안전**: 입력이 `input_timeout` 이상 끊기면 자동 해제, 해제 시 현재 자세에서 즉시 정지, QP 실패 시 충돌 제약 없이 절반 속도로 재시도(횟수 로그).

## 튜닝
| 증상 | 조정 |
|---|---|
| 손이 떨림 | `filter_min_cutoff` ↓(5.0 → 3.0), `filter_beta` ↓(5), `orientation_filter_alpha` ↓(0.2), `task_gain` ↓ — 먼저 `diag_latency` 의 '가장 조용한 2 s σ' 가 3 mm 를 넘는지(넘으면 추적 문제) |
| 반응이 굼뜸 | `filter_beta` ↑, `task_gain` ↑(0.8), `max_joint_velocity` ↑ |
| 팔을 높이 들 때 로봇 팔이 먼저 다 펴지며 손목이 꺾임 / 팔꿈치가 굽은 채 남음 | 스케일 = 로봇 팔 길이 / 조작자 팔 길이: launch `operator_arm:=0.58`(어깨→손바닥 m) 또는 `scale:=`, 실행 중 `ros2 param set /dual_arm_ik operator_arm_length 0.58`. 그립은 팔을 내린 자세에서 |
| 팔꿈치가 부자연스러움 | `nominal_posture` 조정, `posture_cost` ↑(0.02~0.05, 대신 손 오차 수 mm↑) |
| 손목 한계에서 위치가 밀림 | `orientation_cost` ↓(0.1) |
| 손목을 안 움직였는데 로봇 손목이 꺾임 | `orientation_clutch: trigger`(검지 트리거를 당긴 동안만 손목 추종) 또는 `orientation_cost` 0.0(위치만) |
| 그립만 쥐고 가만히 있는데 팔꿈치가 굽음 | `nominal_posture` 가 시작 자세와 다른 것(teleop_ready 팔꿈치 −1.2). SH 기본은 v0.7.35 부터 내린 팔(−0.3, 손목 0). 남는 2.9° 는 한계 0 + `limit_margin` |
| 팔 뻗을 때 뚝뚝 끊김 | `lm_damping` ↑(1e-2) |
| 양팔이 붙지 않음/너무 멀리서 멈춤 | `collision_d_min` 0.02~0.04, 더 보수적이면 `collision_gain` ↓ |
| 특정 관절이 너무 접힘/펴짐 (인접 링크 간섭) | `joint_limits_yaml: "{left_elbow_joint: [-1.8, 0.0], ...}"` — 모델 한계보다 좁은 IK 전용 한계 |
| 전방이 틀어짐 | `quest_yaw_offset_deg` (launch `yaw_offset:=`; xr_teleop 은 forward_axis 로 자동) |
| 뒤처지고 뚝뚝 끊김 (지연) | `ros2 run quest_arm_ik diag_latency` 로 필터·드라이브 지연을 먼저 재고: One-Euro `filter_min_cutoff` 1.5→4, `filter_beta` 0.5→10, `task_gain` 0.8; 실기체는 드라이브 `acc_set_pp`·`loc_kp`(런북 6-1e) → `v_max` ↑ |
| 그립 전에 팔이 저절로 조금 움직임 | 기본은 첫 그립 전 IK 정지(`freeze_until_engaged: true`). 바로 nominal_posture 로 정렬시키려면 false |

## 실기체(삼현 2세대, integration/ros2_interfaces HRController)로 갈 때
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock      hr:=true sim:=true          # 리허설: 시뮬이 HRController 를 흉내 (enable_motors 서비스까지)
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=false v_max:=0.3   # 실기체
```
`hr:=true` 는 IK 를 `output_mode: joint_state` + `joint_states_source: upper_motor_state` 로 두고 `upper_body_streamer` 를 띄운다:
`/teleop/joint_targets`(100 Hz) → 100 Hz `UpperBodyCommand`(position + velocity FF, 빈 슬롯 255) → 미니PC hr_controller(제어기 내장 UDP 링크, 런북 6-1e; CAN 지령 슬롯 200 Hz). 2026-10-20~21 의 '멈췄다 점프'는 미니PC 파이썬 브리지의 Fast DDS 발행 블로킹이었고, DDS 를 링크에서 빼면서(6-1d/6-1e) 사라졌다(2026-10-22 verify: 정지 구간 없음).
스트리머는 **첫 명령을 측정 모터각에서 시작**하고 틱당 `v_max·dt` 이상 움직이지 않으며, IK 가 0.2 s 끊기거나 명령-측정 차이가 0.35 rad 를 넘거나
`/teleop/hold` 가 true 면 그 자리에서 정지한다(명령은 계속 내서 드라이버 워치독 유지). IK 는 로봇이 따라오지 않아 재동기화가 10 s 에 3회면 `/teleop/hold` 를 올린다.
`v_max:=`(기본 config 0.5 rad/s) 하나가 IK `max_joint_velocity` 와 스트리머 한도(×1.25)를 같이 정한다.
**먼저 할 일**: `motor_ids / joint_signs / joint_offsets` 를 실측(`ros2 run quest_arm_ik motor_calib -- watch | zero | jog N 0.1`)으로 채워 config 의
`upper_body_streamer` 블록 한 곳에 적는다(launch 가 IK·시뮬 에뮬레이션에도 넘김). 자세한 순서는 `docs/RUNBOOK.md` 6절. 중력 FF 는 `/teleop/joint_targets` 의 effort.
Quest 바디트래킹 팔꿈치까지 따라가게 하려면 `ik_core`에 `FrameTask(elbow_link, position_cost=0.2, orientation_cost=0)`를 추가.
