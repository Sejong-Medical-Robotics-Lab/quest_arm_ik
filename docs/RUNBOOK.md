# Quest 3 텔레옵 실행 순서 (최종, 경로 B: Isaac ROS teleop · G1 시뮬 검증 → 삼현 2세대)

작성 기준: 2026-10-26 (v0.7.57). 1단계(Thor: JetPack 7.2.1 / Isaac ROS 4.6 Docker 컨테이너, `isaac-ros activate` 에서 `ros2 topic list` 동작) 완료.
장비: Meta Quest 3, Jetson AGX Thor, MacBook. 로봇: Unitree G1(MuJoCo, 검증용) → 삼현 2세대 상반신(SH-Humanoid 저장소 `SH_Humanoid_v2`).

## 0. 전체 그림

```
[Quest 3 브라우저] ─CloudXR(WebXR)─▶ [Thor 컨테이너] isaac_ros_teleop(teleop_ros2_node) ─ /xr_teleop/ee_poses, controller_data ─▶
      quest_bridge(클러치) ─▶ dual_arm_ik(pink IK + 자기충돌, 100 Hz) ─▶ mujoco_sim(G1 → SH) / 팀 RViz·ros2_control / 실기체 HRController
[MacBook] VS Code Remote-SSH 편집, Foxglove(ws://thor:8765) 모니터링, ROS 없는 테스트 실행
```

| 단계 | 내용 | 끝나는 조건 | 소요 |
|---|---|---|---|
| 2 | Thor 워크스페이스: SH-Humanoid + G1(Menagerie) + quest_arm_ik 빌드, 오프라인 테스트, 헤드셋 없이 MuJoCo 구동 | `input:=mock` 으로 G1·SH 양팔이 정면으로 원을 그림 | 반나절 |
| 3 | Quest 3 ↔ Thor: `isaac_ros_teleop` 설치·실행, 헤드셋 연결, `/xr_teleop/*` 축·그립 확인 | `ee_poses` 60 Hz, 앞/위/왼쪽 = x/z/y, 그립 = engaged | 반나절 |
| 4 | **G1** MuJoCo 를 실제 Quest 로 텔레옵 + 튜닝 | 양손 추종 지연 0.2 s 이내, 떨림·점프 없음, 놓으면 정지 | 반나절 |
| 5 | **삼현 2세대** MuJoCo 로 교체 (요 90° 자동) + 스탠드 충돌 확인 + 튜닝 | SH 양팔 추종, 기둥 앞 3 cm 정지 | 반나절 |
| 6 | (선택) 팀 RViz/ros2_control(MoveIt demo) 로 같은 체인 확인 | RViz 로봇이 따라옴 | 2시간 |
| 7 | 실기체: Thor↔미니PC DDS → 인터페이스·단위 확인 → 모터 매핑 실측 → 에뮬레이션 리허설 → 저게인 유지 → mock → Quest (`hr:=true`, 50 Hz 스트리머) | 실기체 양팔이 Quest 를 따라옴, 정지 3종 동작 확인 | 1~2일 |
| 8 | 시연 운영 체크리스트 | — | — |

핵심 사실 (Isaac ROS 4.6 `isaac_ros_teleop` 소스 기준):
- 노드 `teleop_ros2_node` 가 CloudXR 런타임까지 직접 띄운다 → **별도 `python3 -m isaacteleop.cloudxr` 불필요**, G1 WBC(`isaac_ros_unitree_g1_teleop_bringup`) 도 불필요.
- `/xr_teleop/ee_poses` = `geometry_msgs/PoseArray` **[왼손, 오른손]** 컨트롤러 포즈, 이미 **ROS 축(x = Connect 순간 머리가 향한 방향, y 좌측, z 상방), 원점 = 바닥**. 추적 안 되는 손은 (0,0,0).
- `/xr_teleop/controller_data` = msgpack dict, 그립 버튼은 `left_squeeze_value` / `right_squeeze_value` (0~1), 트리거는 `*_trigger_value`.
- 버튼 역할: **그립(측면) = 클러치**(쥔 동안 손이 따라옴), **트리거(앞쪽) = 손 쥐기**(당긴 만큼 Dex3 손가락이 접힘, `/teleop/{side}_hand/grasp` 0~1).
- 시작 자세는 두 로봇 다 **팔을 옆으로 내린 자세**다. 조작자도 같은 자세로 그립을 쥐고 시작하면 되고, 처음 동작은 팔을 들어 올리는 것.
- **위치 스케일 = 로봇 팔 길이 / 조작자 팔 길이**(어깨→손바닥). G1 0.38 m, SH 0.47 m(어깨→손 플랜지). 상대 매핑은 '그립을 쥔 순간의 손'을 기준으로 변위를 스케일하므로,
  팔을 내린 채 쥐면 사람 어깨와 로봇 어깨가 같은 기준점이 되고 이 비율이 맞을 때 로봇 팔이 사람과 같은 정도로 펴지고 굽는다. 비율이 크면 팔을 높이 들 때 로봇 팔이 먼저 다 펴져
  손목이 꺾이고, 작으면 팔꿈치가 굽은 채 남는다. `operator_arm:=0.58`(조작자 어깨→손바닥 [m], ≈ 키 × 0.33) 로 자동 계산, 또는 `scale:=`. 실행 중에는 `ros2 param set /dual_arm_ik operator_arm_length 0.58`.
- 패키지는 이 규약을 기본값으로 갖고 있고(`engage_mode: grip`, squeeze 키), `input:=xr_teleop` 이면 launch 가 **요 오프셋을 자동**(G1 0°, SH 90°) 으로 넣는다.
- 실기체는 `hr:=true` 한 스위치다: IK(100 Hz) → `upper_body_streamer`(**50 Hz** `UpperBodyCommand` — 200 Hz 는 미니PC 브리지가 밀려 튐(6-2b), 팀 trajectory_bridge 와 같은 형식, 첫 명령 = 측정각, 속도 제한, IK 끊김·추종 오차·`/teleop/hold` 시 정지, 정지에서 재개할 땐 목표까지 `v_catchup` 0.3 rad/s 로 접근(v0.7.55)) → 삼현 미니PC HRController(1 kHz).
  `sim:=true` 를 같이 주면 MuJoCo 가 HRController 를 흉내 내어(토픽·서비스 동일) 하드웨어 없이 같은 경로를 리허설한다. 모터 매핑은 config `upper_body_streamer` 블록 한 곳에만 적는다.
- 조작자는 **로봇(또는 화면 속 로봇)과 같은 방향을 보고 선다.** MuJoCo 뷰어 카메라는 기본이 로봇 뒤(behind)라 화면의 앞/오른쪽 = 로봇의 앞/오른쪽.
- Thor 이미지의 **시스템 python 은 NVIDIA 것**이다: isaacteleop 1.3 + dex_retargeting 0.4.6 + pinocchio(pin) **2.7.0** + numpy 1.26 + torch. 우리 스택(pin 3.x, pink, MuJoCo, numpy 2)은
  같은 사이트에 들어갈 수 없어(배포 이름 `pin` 이 같음) **전용 venv `/opt/quest_py`** 에 두고, `teleop.launch.py` 가 우리 노드만 그 인터프리터(`$QPY`)로 실행한다.
  NVIDIA 노드는 시스템 python 그대로. 패키지는 pin 3.9 / 4.1 둘 다에서 검증됐다(v0.6.2: 손 자세·목표를 `base_frame` 기준으로 계산해 MJCF 부동 베이스 처리 차이 흡수).

## 1. (2단계) Thor 워크스페이스 + 오프라인 검증 + mock 구동

### 1-1. 설치 (최초 1회) — 의존성은 컨테이너가 아니라 **이미지 레이어**에 굽는다
Isaac ROS 개발 컨테이너는 Thor 를 재부팅하거나 `isaac-ros activate` 가 컨테이너를 다시 만들 때마다 이미지에서 새로 생성되고,
컨테이너 안에서 apt/pip 로 설치한 것(`ros-jazzy-isaac-ros-teleop`, `python3-pip`, GL, pin/pink …)은 전부 사라진다. 워크스페이스
(`/workspaces/isaac_ros-dev` = 호스트 `~/workspaces/isaac_ros-dev`: 소스·`install/`·`.cloudxr`)만 남는다. 그래서 의존성은
`docker/Dockerfile.zz_quest_teleop` 레이어로 이미지에 넣는다(Isaac ROS CLI 의 custom image layer 기능).

맥북 터미널 (zip 은 Finder 로 홈 폴더에 옮겨 둔다 — `~/Downloads` 는 macOS 가 터미널 접근을 막아 `Operation not permitted` 가 난다):
```bash
scp ~/quest_arm_ik.zip thor:workspaces/isaac_ros-dev/src/
```
Thor **호스트**(컨테이너 밖; 컨테이너 터미널은 모두 `exit`):
```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/Sejong-Medical-Robotics-Lab/SH-Humanoid.git
rm -rf SH-Humanoid/integration/ros2_interfaces/{build,install,log}         # 커밋돼 있는 빌드 산출물 제거
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
(cd mujoco_menagerie && git sparse-checkout set unitree_g1 && touch COLCON_IGNORE)   # → unitree_g1/g1.xml (메시 포함)
python3 -m zipfile -e quest_arm_ik.zip .
bash quest_arm_ik/scripts/setup_persistent.sh     # 레이어 등록: docker/Dockerfile.zz_quest_teleop, scripts/.isaac_ros_common-config, .isaac-ros-cli/config.yaml
isaac-ros activate --build-local 2>&1 | tee ~/build.log   # 레이어 빌드(apt + pip, 수 분~수십 분; 1회) 후 컨테이너 진입
```
빌드 로그 끝에 `quest_arm_ik deps ok: pin 3.x.x pink 4.x.x …` 와 `isaac_ros_teleop deps ok` 두 줄이 찍히면 성공이다.
`isaac-ros activate` 는 이미 떠 있는 컨테이너가 있으면 빌드 없이 그냥 붙으므로, 빌드 전엔 `docker stop isaac_ros_dev_container` 로 내린다.
컨테이너 안(프롬프트 `admin@localhost:/workspaces/isaac_ros-dev$`):
```bash
/opt/quest_py/bin/python3 -c "import pinocchio, pink, mujoco; print('quest_arm_ik deps ok', pinocchio.__version__)"   # 우리 venv (pin 3.9)
python3 -c "import isaacteleop.cloudxr, pinocchio; print('nvidia ok', pinocchio.__version__)"                     # 시스템 python (pin 2.7.0)
cd $ISAAC_ROS_WS
colcon build --symlink-install --packages-select SH_Humanoid_v2_description SH_Humanoid_v2_moveit_config ros2_interfaces quest_arm_ik
source /workspaces/isaac_ros-dev/src/quest_arm_ik/scripts/env.sh   # install/setup.bash + G1/SH 변수 + QPY(=/opt/quest_py/bin/python3) + PYTHONNOUSERSITE=1 (새 터미널마다)
```
- 이후 날마다: 호스트 `isaac-ros activate` → 컨테이너 `source /workspaces/isaac_ros-dev/src/quest_arm_ik/scripts/env.sh`. 재빌드 없음(코드를 바꿨을 때만 `colcon build --symlink-install --packages-select quest_arm_ik`).
- **새 zip 으로 갱신할 때 config 가 패키지 기본값으로 덮인다**(2026-10-26 E 단계에서 필터 지연이 20 → 100 ms 로 되돌아간 원인). 풀기 전에 `cp src/quest_arm_ik/config/sh_humanoid_v2.yaml ~/sh_humanoid_v2.yaml.bak` 하고, 푼 뒤 `diff ~/sh_humanoid_v2.yaml.bak src/quest_arm_ik/config/sh_humanoid_v2.yaml` 로 현장에서 바꾼 값을 되살린다. v0.7.45 부터는 실기체에서 확정한 값(filter_min_cutoff 5.0, filter_beta 10.0, task_gain 0.8, elbow_hint_cost 0)이 패키지 yaml 자체에 들어 있다.
  **절대 경로로 source 한다** — 호스트 `~/.bashrc` 가 컨테이너에 그대로 마운트되어 호스트용 `ISAAC_ROS_WS=/home/<user>/…` 가 컨테이너 값(`/workspaces/isaac_ros-dev`)을 덮어쓰면
  `$ISAAC_ROS_WS/...` 가 `No such file or directory` 가 된다. `env.sh` 가 컨테이너 안이면 `ISAAC_ROS_WS` 를 다시 `/workspaces/isaac_ros-dev` 로 잡아 주므로 그 다음부터는 `$ISAAC_ROS_WS` 를 써도 된다.
- 레이어는 (1) apt 로 `ros-jazzy-isaac-ros-teleop`(이때 NVIDIA 의 pip-shim 이 isaacteleop·pin 2.7.0 등을 시스템 사이트에 넣음)과 GL 라이브러리, (2) 우리 의존성은 **전용 venv `/opt/quest_py`**
  (pin 3.9 + pink + mujoco + qpsolvers/quadprog + numpy 2; `cmeel-urdfdom<5`, `cmeel-tinyxml2<11` 캡으로 pinocchio 바이너리 soname 일치). 시스템 python 은 건드리지 않는다.
  우리 노드는 launch 가 `$QPY`(= /opt/quest_py/bin/python3) 로 실행하고, rclpy 등 ROS 패키지는 `setup.bash` 의 PYTHONPATH 로 들어온다.
- 레이어 빌드가 실패하면 `grep -n -i -B3 -A12 "error" ~/build.log | tail -60` 을 확인한다. 임시로는 컨테이너 안에서
  `bash $ISAAC_ROS_WS/src/quest_arm_ik/scripts/setup_container.sh` (같은 내용을 컨테이너에 직접 설치; 컨테이너가 새로 만들어질 때마다 다시 실행).
- `SH_Humanoid_v2_moveit_config` 가 MoveIt 의존성으로 빌드 실패하면 넘어가도 된다(launch 에 `srdf_path:=$ISAAC_ROS_WS/src/SH-Humanoid/urdf_moveit/SH_Humanoid_v2_moveit_config/config/SH_Humanoid_v2.srdf` 를 직접 준다).
- 시스템 python 에 `pip install`/`pip --user` 로 pinocchio·pink 를 넣지 않는다(NVIDIA 노드가 깨지거나 `libtinyxml*.so` ImportError). 우리 것은 전부 `/opt/quest_py` 에만.

### 1-2. ROS 없이 모델·IK·시뮬 검증 (10분)
```bash
cd $ISAAC_ROS_WS/src/quest_arm_ik
export G1=$ISAAC_ROS_WS/src/mujoco_menagerie/unitree_g1/g1.xml SH=$ISAAC_ROS_WS/src/SH-Humanoid
$QPY test/test_protocols.py                                 # 패킷·msgpack 키·OpenXR→ROS 변환 (모델 불필요)   ($QPY = /opt/quest_py/bin/python3, env.sh)
$QPY test/test_ik_core.py                                   # 합성 URDF 로 IK 코어
G1_MJCF=$G1 $QPY test/test_g1_sim_loop.py                   # G1: FK 일치·추종·자기충돌 30 mm·100 Hz 예산
G1_MJCF=$G1 $QPY test/test_teleop_chain.py                  # G1: 컨트롤러→클러치→IK→시뮬 (release·re-engage)
G1_MJCF=$G1 SH_REPO=$SH $QPY test/test_xr_chain.py          # Isaac ROS 스트림 흉내 → G1(요 0°)·SH(요 90°)
G1_MJCF=$G1 G1H_MJCF=$G1H SH_REPO=$SH $QPY test/test_rest_start.py   # 팔 내린 시작 자세에서 들어 올리기→원→쥐기 추종 (G1·G1+손·SH+Dex3)
SH_REPO=$SH G1H_MJCF=$G1H $QPY test/test_grasp.py           # 트리거 → Dex3 손가락 쥐기/펴기
SH_REPO=$SH $QPY test/test_sh_chain.py                      # SH: 스탠드 기둥 앞 정지
$QPY test/test_upper_body.py                                # 실기체 스트리머 로직 (점프 금지·속도 제한·워치독·15 슬롯, config 매핑 일관성; 모델 불필요)
SH_REPO=$SH $QPY test/test_hr_chain.py                      # 실기체 경로 리허설: 가짜 rclpy 로 mock→IK→스트리머→HRController 에뮬→motor_state 전 구간 (~15 s)
$QPY scripts/check_model.py --model $G1 --config config/dual_arm_ik.yaml
$QPY scripts/check_model.py --model $SH/urdf_moveit/SH_Humanoid_v2_description/urdf/SH_Humanoid_v2.xacro \
  --config config/sh_humanoid_v2.yaml --package-dirs $SH/urdf_moveit \
  --srdf $SH/urdf_moveit/SH_Humanoid_v2_moveit_config/config/SH_Humanoid_v2.srdf
```
전부 `OK` 여야 한다(개발 환경 기준: G1 FK 0.0 mm·충돌 30 mm 유지·4 ms, SH 추종 3.4 mm·기둥 앞 28.8 mm). 맥북에서도 같은 pip 패키지로 동일하게 돌아간다.

### 1-3. 헤드셋 없이 전체 ROS 체인 (G1 → SH)
Thor 모니터 앞에서(SSH 면 호스트 터미널에서 `xhost +local:` 후 컨테이너에서 `export DISPLAY=:1` 또는 `:0`):
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=$G1 input:=mock
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock
```
뷰어에 로봇이 **뒤에서 본 시점**으로 뜨고 **양팔을 옆으로 내린 시작 자세**(`mujoco_sim.initial_posture`; SH 는 팔꿈치만 0.35 rad 살짝 굽힘, G1 은 stand 자세에서 어깨를 0.35 rad 벌림)로 서 있다.
IK 가 뜬 뒤 2초에 mock 이 그립을 쥐고 양손을 **앞·위로 2초간 들어 올린 뒤**(사람이 팔을 드는 동작 흉내: 손끝도 아래→앞으로 60° 들림) 정면으로 원을 천천히 그리고, 그 뒤부터 4초 주기로 **손을 쥐었다 편다**(Dex3 손 모델일 때). 여기까지 보이면 **2단계 끝**.
(손 원 지름: SH 10 cm × 0.85 ≈ 8.5 cm, G1 10 cm × 0.6 = 6 cm; `mock_radius:=` 로 조절. 팔을 내린 자세에서 바로 원을 그리면 아래쪽이 닿지 않으므로 올리는 단계가 있다 — `quest_mock` 의 `lift_forward 0.06 / lift_up 0.15 / lift_time 2.0`.)
확인: `ros2 topic hz /joint_states`(100 Hz), `ros2 topic echo /teleop/joint_targets --once`(SH: 14축 + effort=중력토크).
모니터 없이: `viewer:=false` 로 띄우고 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` → 맥북 Foxglove `ws://<thor-ip>:8765`
(3D 패널에 `/quest_ik/{left,right}_hand/target`, Plot 에 `/joint_states`).

## 2. (3단계) Quest 3 ↔ Thor: isaac_ros_teleop

### 2-1. 네트워크·포트
- Thor 유선, Quest 5 GHz Wi-Fi, 같은 공유기·서브넷(클라이언트 격리 끔). Thor IP 고정(`hostname -I`). Quest 펌웨어 최신.
- 방화벽은 **호스트(컨테이너 밖)** 에서. `sudo ufw status` 가 inactive 면 생략:
```bash
sudo ufw allow 47998/udp && sudo ufw allow 49100,48322/tcp
```

### 2-2. 설치·실행 (터미널 A, 계속 켜 둔다)
```bash
source $ISAAC_ROS_WS/src/quest_arm_ik/scripts/env.sh
python3 -c "import isaacteleop.cloudxr, dex_retargeting.retargeting_config; print('isaacteleop ok')"   # 레이어에 포함되어 있음
ros2 launch isaac_ros_teleop isaac_ros_teleop.launch.py cloudxr_accept_eula:=true
```
(`ros-jazzy-isaac-ros-teleop` 는 1-1 의 이미지 레이어에 들어 있다. 없다고 나오면 레이어 빌드가 안 된 것 → 1-1 또는 `setup_container.sh`.)
첫 실행은 CloudXR 런타임을 `$ISAAC_ROS_WS/.cloudxr` 에 내려받는다(인터넷 필요, 수 분). `CloudXR runtime and WSS proxy started` 뒤에
`No XR client connected ... retrying in 2s` 가 반복되는 것은 **정상**(헤드셋이 붙으면 `TeleopSession started successfully`).

### 2-3. 헤드셋 연결
1. Quest 브라우저에서 `https://nvidia.github.io/IsaacTeleop/client/release-1.3.x/#/real/isaacros` (Isaac ROS 4.6 = IsaacTeleop 1.3.x 클라이언트).
2. **Server IP** 에 Thor IP → `Click https://<ip>:48322/ to accept cert` → 새 탭에서 Advanced → Proceed → `Certificate Accepted` 확인 → 클라이언트 탭으로.
3. **Video Codec = H.264** (Jetson 필수).
4. **로봇(또는 모니터)과 같은 방향을 보고 가만히 선 채** Connect. 이 순간 머리가 향한 방향이 ROS x(정면)이 된다. 방향을 다시 잡으려면 Disconnect → 제자리에서 Connect.
5. 화면이 검게 보이면 클라이언트의 passthrough(AR) 옵션을 켠다 — 렌더링하는 앱이 없으므로 실제 환경을 보면서 조작한다.

### 2-4. 데이터·축·그립 확인 (터미널 B)
```bash
ros2 topic hz /xr_teleop/ee_poses                     # ~60 Hz
ros2 run quest_arm_ik xr_dump                         # poses[0]=왼손, frame_id=world, controller_data 키에 left_squeeze_value 가 보여야 함
$QPY -m quest_arm_ik.quest_bridge --ros-args -p input_mode:=xr_teleop -p engage_mode:=always &   # 우리 venv 로 실행 (ros2 run 은 시스템 python)
ros2 topic echo /teleop/right_hand/target_raw --field pose.position
```
오른손을 **앞으로** 밀면 x 증가, **위로** 들면 z 증가, **왼쪽**으로 밀면 y 증가(ROS 축). 다르면 Connect 순간의 머리 방향 문제 → 재연결.
```bash
kill %1; $QPY -m quest_arm_ik.quest_bridge --ros-args -p input_mode:=xr_teleop -p engage_mode:=grip &
ros2 topic echo /teleop/right_hand/engaged           # 그립 버튼(중지 쪽 측면 버튼)을 쥐면 True, 놓으면 False
ros2 topic echo /teleop/right_hand/grasp             # 검지 트리거(앞쪽 버튼)를 당기면 0→1 (손 쥐기)
```
`xr_dump` 가 다른 순서·키를 보이면 config 의 `ee_poses_order`, `left/right_engage_key` 만 고친다. 여기까지가 **3단계 끝**.
`quest_bridge` 로그는 전환 때만 나온다: 유효 입력이 시작되면 `input started … L 60 Hz, R 60 Hz` 한 줄, 5 초 이상 끊기면 `no input for N s` 한 줄
(컨트롤러를 내려놓아 핸드 트래킹으로 바뀌었거나 헤드셋 대기). 입력률을 계속 보고 싶으면 `-p report_period:=10.0` (launch 는 config 의 `quest_bridge.report_period`).

## 3. (4단계) G1 MuJoCo 를 실제 Quest 로 텔레옵
터미널 A 의 `isaac_ros_teleop` 은 켜 둔 채:
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=$G1 input:=xr_teleop
```
순서: 뷰어의 G1 이 **팔을 옆으로 내린 자세**로 서 있는 것 확인 → 모니터(로봇 뒤 시점)를 보고 서서, 필요하면 그 자세로 Disconnect/Connect → **조작자도 팔을 옆으로 내린 채
그립(측면 버튼)을 쥔다**(그 순간 로봇 손의 현재 위치가 기준 — 로봇과 같은 자세에서 시작하니 따로 맞출 것이 없다) → 팔을 들어 움직인다 → 놓으면 그 자리에 정지, 다시 쥐면 이어서. 로그에 `left hand engaged / released`.
**검지 트리거(앞쪽 버튼)** 를 당기면 당긴 만큼(0~1) 손이 쥐어진다 — G1 손 포함 모델(`model_path:=$G1H`, g1_with_hands.xml)이면 뷰어에서 Dex3 손가락이 접히고, 손 없는 `$G1` 이면 명령만 나가고(`/teleop/hand_joint_targets`) 보이지는 않는다.
로봇을 마주 보고 조작하고 싶으면 `yaw_offset:=180` — 단 좌우도 로봇 기준으로 함께 돌아간다(거울이 아님).

끝나는 조건: 양손이 0.2 s 안에 따라오고, 양손을 가까이 모아도 3 cm 에서 멈추며(자기충돌), 놓으면 즉시 멈추고, 다시 쥐어도 점프가 없다.

튜닝은 `config/dual_arm_ik.yaml` 수정 후 재실행(빌드 불필요):
| 증상 | 항목 |
|---|---|
| 팔을 높이 들면 로봇 팔꿈치가 굽은 채 남음 / 반대로 팔이 먼저 다 펴지며 손목이 꺾임 | 스케일 = 로봇 팔 0.38 m / 조작자 어깨→손바닥. `operator_arm:=<m>`(≈ 키×0.33, 175 cm → 0.58) 또는 `scale:=`; 실행 중 `ros2 param set /dual_arm_ik operator_arm_length 0.58`. 기본 0.6 = 조작자 0.63 m 기준 |
| 손이 떨림 | `filter_min_cutoff` 1.5→1.0, `orientation_filter_alpha` 0.3→0.2 |
| 굼뜸 | `filter_beta` ↑, `task_gain` 0.5→0.8, `max_joint_velocity` ↑ |
| 손목이 억지로 비틀림 / 손목을 안 움직였는데 꺾임 | SH: `orientation_clutch: trigger`(검지 트리거를 당긴 동안만 손목 추종, v0.7.35). 또는 `orientation_cost` ↓ (0.0 = 위치만) |
| 팔꿈치가 벌어짐/몸에 붙음 | `nominal_posture`, `posture_cost` |
| 양팔이 너무 일찍 멈춤 / 닿음 | `collision_d_min` 0.03 (0.02~0.05) |
| 정면이 조금 틀어짐 | `yaw_offset:=<deg>` (재연결 대신) |
| 뷰어 시점 | `viewer_view:=front`, 또는 `viewer:=false` + Foxglove |
| 손이 너무 쉽게/늦게 쥐어짐 | `hand_grasp.tau`(0.1 s) — 트리거 저역통과; 쥠 각은 `hand_grasp.hand_joints_yaml` 의 두 번째 값 |
| 시작 자세를 바꾸고 싶음 | `mujoco_sim.initial_posture`(시뮬 시작 자세, 14개) — 실기체는 드라이버 쪽 초기 자세. IK 의 `nominal_posture`(선호 자세)는 그대로 둔다 |

## 4. (5단계) 삼현 2세대 MuJoCo 로 교체
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop        # 모델·SRDF·메시 경로 자동, 요 오프셋 90° 자동
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hand:=box   # 기존 박스 손 모델로 (기본은 G1 Dex3-1 3지 핸드)
```
- **손 모델**: 전시 사양(Unitree G1 Dex3-1 3지 핸드)이 `SH_Humanoid_v2.xacro` 의 기본(`hand:=dex3`)이다. 플랜지 프레임 `*_hand_link` 는 박스 손과 같은 자리라
  IK·클러치·SRDF 는 그대로이고, 손끝까지 길이만 217 → 170 mm 로 짧아진다. 손가락은 고정(살짝 오므림), 충돌형상은 손바닥+엄지 박스 2개.
  장착 방향(손바닥이 정중선을 보고 검지가 정면)은 `dex3_hand.xacro` 의 `*_hand_dex3_mount` rpy 한 줄로 바꾼다 — 실제 어댑터가 정해지면 맞출 것.
  팀 저장소에 `urdf/dex3_hand.xacro`, `meshes/dex3/` 가 없으면(아직 머지 전) `SH-Humanoid_dex3_hand.zip` 을 저장소 루트(`src/SH-Humanoid/`)에 풀고
  **워크스페이스 루트**(`cd /workspaces/isaac_ros-dev`)에서 `colcon build --symlink-install --packages-select SH_Humanoid_v2_description` 으로 `install/` 에 새 파일을 올린다
  (다른 디렉터리에서 colcon 을 돌리면 그 자리에 별도의 `build/ install/` 이 생기고 소싱된 `install/` 은 그대로라 `XacroException: No such file or directory: …/install/…/urdf/dex3_hand.xacro` 가 난다 — 그 `build/ install/ log/` 는 지운다).
  설치 없이 소스 트리로 바로 띄우려면 `model_path:=$SH/urdf_moveit/SH_Humanoid_v2_description/urdf/SH_Humanoid_v2.xacro package_dirs:=$SH/urdf_moveit`. 뷰어는 v0.6.4 부터 충돌형상 대신 메시(비주얼)를 보여 준다.
- 축 확인(SH base_link: +X 오른쪽, +Y 정면, +Z 위): `ros2 topic echo /quest_ik/right_hand/target --field pose.position` 에서 앞으로 밀면 **y** 증가, 오른쪽 **x** 증가, 위 **z** 증가.
- 스탠드 확인: 손을 몸 쪽(기둥)으로 밀어 넣으면 기둥 앞 약 3 cm 에서 멈춰야 한다(`collision_d_min: 0.03`). 양손끼리도 동일.
- 시뮬은 **팔을 옆으로 내린 자세**(팔꿈치 −0.15 rad, 손이 어깨 바로 아래)에서 시작한다. 조작자도 팔을 내린 채 그립을 쥐고 들어 올리면 된다.
  스케일 기본 0.75 = SH 팔 0.47 m / 조작자 0.62 m. 조작자에 맞추려면 `operator_arm:=0.58` 처럼 어깨→손바닥 길이를 준다(v0.7.3; 이전 0.85 + 팔꿈치 −0.35 시작은 팔을 들면 로봇 팔이 먼저 다 펴지고 손목이 꺾였다). 팀 토픽 `/teleop/joint_targets`(position + 중력 effort) 와 `/left|right_arm_forward_controller/commands` 가 같이 나간다.
  (실기체의 시작 자세는 드라이버/`ready.sh` 가 정한다 — teleop_ready 로 시작하더라도 클러치가 그 자리를 기준으로 삼으므로 문제없다.)
- **손 쥐기**: 트리거(앞쪽 버튼)를 당기면 `/teleop/{left,right}_hand/grasp`(0~1) → `hand_grasp` 노드가 Dex3 손가락 14관절 목표를 `/teleop/hand_joint_targets`(JointState) 로 내고, 시뮬이 그대로 움직인다(launch 가 `dex3_fingers:=revolute` 로 모델을 띄움). 실기체는 Dex3 드라이버가 같은 토픽(또는 grasp 0~1)을 받으면 된다. 끄려면 `grasp:=false`.

튜닝은 `config/sh_humanoid_v2.yaml`: `translation_scale` 0.85(팀 delta_scale), `max_step` 0.03, `posture_weights`(어깨 pitch 0.1·roll/yaw 1.0·팔꿈치 0.3·손목 0.5), `orientation_cost` 0.15, `collision_d_min` 0.03. 나머지 항목은 4단계 표와 같다.
`mapping_mode: absolute` 는 Unity(경로 A) 절대 포즈용이므로 경로 B 에서는 relative 유지.

## 5. (6단계, 선택) 팀 RViz/ros2_control 로 같은 체인 확인
```bash
sudo apt-get install -y ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager
ros2 launch SH_Humanoid_v2_moveit_config demo.launch.py              # RViz + mock 하드웨어
bash $ISAAC_ROS_WS/src/SH-Humanoid/integration/scripts/ready.sh       # teleop_ready 로 이동 후 forward 컨트롤러로 전환
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop sim:=false   # 출력 = /left|right_arm_forward_controller/commands
```
RViz 로봇이 Quest 를 따라오면 실기체 ros2_control 인터페이스와도 호환. 되돌리기 `to_plan.sh`, 복구 `reset_arms.sh`.

## 6. (7단계) 실기체 — 삼현 미니PC(HRController, 1 kHz) ↔ Thor(`hr:=true`, 50 Hz)

### 6-0. 구조와 역할 분담
```
[Quest 3] ─60 Hz─▶ quest_bridge ─▶ dual_arm_ik (100 Hz, pink IK)
                                     ├─▶ /teleop/joint_targets (JointState 14축 + 중력토크 effort)
                                     │        ▼
                                     │   upper_body_streamer (50 Hz) ───/upper_body/command (UpperBodyCommand 15슬롯)──▶ [미니PC] HRController (1 kHz) ─▶ 모터 ×15
                                     ◀───────────────────────────────────/upper_body/motor_state (측정각·속도·토크·온도·오류) ◀──┘
                                                                          /upper_body/system_state (READY/ENABLED/RUNNING/FAULT, run_mode) ◀─┘
   같은 PC 안이 아니라 Thor 컨테이너(--network host) ↔ 미니PC 가 이더넷 DDS 로 묶인다. 서비스 enable_motors / disable_motors / set_run_mode 도 미니PC 것.
```
- **IK 는 100 Hz 그대로**(사람 손 입력은 60 Hz). 100 Hz → `rate`(**50 Hz**) 변환과 안전장치는 전부 `upper_body_streamer` 가 맡는다:
  매 틱(20 ms) `q_cmd` 를 IK 목표 쪽으로 **최대 `v_max·dt` 만큼만** 옮기고(= 보간기이자 속도 제한), 그 변화율을 `velocity` 피드포워드로 싣고 `duration` = 1/rate 를 준다(팀 trajectory_bridge 와 같은 형식; 드라이버는 duration 을 무시하는 것으로 실측됨). 20 ms 계단(≤12.5 mrad)은 모터 내부 위치 루프가 이어 준다.
  **왜 50 Hz 인가(2026-10-20 실측, 6-2b)**: 200 Hz 로 보내면 미니PC 브리지가 명령을 0.5~2.4 s 씩 처리하지 못하다가 한 번에 점프했고(틱당 측정 139~186 mrad, 2 rad/s), 50 Hz 에서는 매끄럽게 추종했다(지연 ≈0.13 s, 오차 ≤10 mrad). 100 Hz 는 `motor_calib verify --rate 100` 으로 확인한 뒤에만 올린다.
- **점프 금지**: 첫 명령은 항상 `motor_state` 의 측정각에서 출발한다(enable 전에는 `q_cmd` 를 측정값에 고정). enable 순간이나 IK 재시작 때 팔이 튀는 일이 구조적으로 없다.
- **정지 3종**: ① IK 목표가 0.2 s 끊김(IK/브리지/헤드셋 죽음) ② 명령-측정 차 0.35 rad 초과(충돌·모터 불능·과부하) ③ `/teleop/hold true`(사람 또는 IK 가 올림) → 그 자리에서 감속 정지, **명령은 계속 `rate`(50 Hz) 로 나간다**(드라이버 쪽 통신 워치독 유지, 현재 위치 유지). 복귀도 그 자리에서 다시 추종(점프 없음).
- **매핑 오류 안전망**: 모터 부호/ID 가 틀리면 스트리머 자신은 모른다(명령·측정이 같은 표를 거침). 대신 로봇이 IK 와 반대로 움직여 IK 의 `재동기화`(내부 자세 ≠ 측정 0.3 rad) 가 반복되고, 10 s 안에 3회면 IK 가 `/teleop/hold` 를 올려 스트리머를 멈춘다. 그래도 **jog 로 부호를 다 확인하기 전에는 `hr:=true sim:=false` 를 띄우지 않는다.**
- 속도는 `v_max:=` 하나로: IK `max_joint_velocity = v_max`, 스트리머 한도 `1.25·v_max`. IK 가 스트리머보다 빠르면 로봇이 뒤처져 재동기화가 반복되므로 **IK 쪽이 느린 것이 원칙**이다. 첫날 0.2~0.3, 익숙해지면 0.5~0.8 rad/s.
- `motor_state`·`system_state` 는 미니PC 발행률 그대로 받는다(1 kHz 여도 됨). 스트리머·IK 구독은 best-effort 라 드라이버가 어느 QoS 로 내도 받는다.
- 15번 모터(팔 14축 외 1개 — 정체 확인 필요: 허리 요? 목?)는 명령 슬롯에 **현재 측정 위치 유지**로 채운다(`extra_motor_ids: [15]`, kp/kd 는 `extra_kp/kd`). 없으면 `[]`.

### 6-1. 전날/아침 준비 (30분)
- 이더넷 케이블 직결 또는 같은 스위치. **고정 IP** 같은 서브넷(예: Thor 192.168.10.1/24, 미니PC 192.168.10.2). Quest 공유기(Wi-Fi)는 별개 인터페이스라 그대로.
- 삼현에 미리 받아 둘 것: 미니PC ROS 배포판(Jazzy?)·RMW(Fast DDS 기본?)·`ROS_DOMAIN_ID`, `ros2_interfaces` 가 저장소 `integration/ros2_interfaces` 와 같은 커밋인지(Jazzy 는 메시지 정의가 다르면 타입 해시 불일치로 통신이 안 됨),
  모터 position 단위(rad? deg? turn?)·부호 규약, `command_mode` 중 지원되는 것(POSITION=1 / POSITION_VELOCITY=4 / IMPEDANCE=5)과 kp/kd 단위·권장값, `duration` 의 의미(0 이면 즉시? 보간?), 명령이 끊겼을 때 드라이버 동작(워치독 시간·정지 방식), `run_mode`(OD 0x7005: IMPEDANCE=0, POSITION=1, VELOCITY=2, TORQUE=3, POSITION_ALT=5) 와 `command_mode` 의 관계, 15번 모터 정체, 비상정지 동작(전원 차단인지 브레이크인지).
  (OD 0x7005 run_mode 번호 체계는 RobStride/CyberGear 계열 모터와 같다 — 그 계열이면 run_mode 0 = MIT(임피던스) 모드에 kp 0~500·kd 0~5 범위. 추정이므로 확인.)
- 작업공간 비우기, 팔 아래 매트, 비상정지 담당 1명 고정(스위치만 잡고 다른 일 안 함). 사람이 로봇 팔 반경 안에 서지 않기.
- Thor 컨테이너: `env.sh` 가 `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` 을 내보낸다 — 미니PC 값에 맞춘다(`export ROS_DOMAIN_ID=<n>` 뒤 source). 두 쪽이 다르면 서로 토픽이 안 보인다.
- 위 질문은 6-8 표에 '묻는 말 / 직접 확인 / 들어가는 곳' 으로 정리돼 있다.
- 리허설 먼저: `SH_REPO=$SH $QPY test/test_hr_chain.py` 와 `ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock hr:=true sim:=true` 가 돌아가는지(아래 6-5 와 같은 화면).

### 6-1b. 실기체에서 확인된 인터페이스 (2026-10-20, Thor 에서 `ros2 node info /hr_state_bridge`)
저장소 문서와 다른 점이 있어 **이 표가 기준**이다 (config 기본값도 이에 맞춰 둠):
| 항목 | 실제 |
|---|---|
| 드라이버 ROS 노드 | `/hr_state_bridge` (+ `/rosbridge_websocket` — 웹 대시보드용 rosbridge) |
| 측정 | `/upper_body/motor_states` (UpperMotorState, **복수형**), `/upper_body/joint_states` (sensor_msgs/JointState, name = axis1~axis15), `/upper_body/system_state`, `/upper_body/axis_map` (String) |
| 명령 | `/upper_body/command` (UpperBodyCommand) — 문서와 같음. 토픽 `/irobe/command/enable_all`, `/irobe/command/disable_all` (std_msgs/Empty) 도 있음 |
| 서비스 | `enable_motors`, `disable_motors`, `set_run_mode` 외에 `/upper_body/clear_fault`, `/upper_body/emergency_stop`, `/upper_body/home_axis` (SimpleMotorResult) |
| motor_id | 1~15 가 아니라 **하드웨어 UID**: 슬롯 1~15 = `34,113,125,52,105,115,57, 51,32,33,31,35,36,37, 90`. 슬롯 15(UID 90) 는 `is_ready=false` → config `extra_motor_ids: []` |
| 단위 | position 양자화 0.0001917 rad (= 12.5625/65536 → 16비트 ±2π), velocity 0.000458 (= 30/65536 → ±15 rad/s), effort 0.00061 (= 40/65536 → ±20 N·m). 즉 CAN 16비트 MIT 식 인코딩, **rad / rad/s / N·m** 로 판단 (부호·영점은 실측) |
| temperature/voltage/current | 0 (미보고) |
`axis_map` 은 축↔UID(또는 관절 이름) 대응표일 가능성이 크니 `ros2 topic echo /upper_body/axis_map --once` 로 먼저 본다. `joint_states` 의 axis1~7 / 8~14 가 왼팔/오른팔(URDF 순서)이라는 가정으로 config `motor_ids` 를 채워 두었으므로 jog 로 확인해 확정한다.

### 6-1c. HRController 소스 분석 (2026-10-20, 삼현 제공 zip = git 2026-07-01 스냅샷 — 미니PC 의 배포본이 더 새 것일 수 있음)
구조(미니PC 한 대 안): `hr_controller`(C++, iRobe 런타임) ← Unix 데이터그램 IPC `/tmp/hr_controller_cmd.sock` ← `tools/ros2_state_bridge.py`(= ROS 노드 `/hr_state_bridge`, Python) ← DDS ← Thor. 상태는 반대로 `hr_controller` 가 `/tmp/hr_controller_state.json` 을 **100 ms 마다** 쓰고 브리지 타이머(기본 10 Hz)가 읽어 `motor_states`·`system_state`·`joint_states`·`axis_map` 으로 발행한다(그래서 측정값은 최대 0.2 s 묵은 값). 모터는 USR-CANET200 이더넷-CAN 변환기(192.168.0.7, **TCP** 2채널 20001/20005, 1 ms 주기로 최대 50 프레임씩)를 거쳐 CAN(삼현 MIT 프로토콜, OD 0x7005 run_mode / 0x7016 loc_ref / 0x7017 limit_spd)으로 간다. 배포 config `config/hr_controller.yaml` 에 축별 **위치 한계(deg)** 가 있고 벗어난 목표는 잘라 쓴다(`MoveToPos clamped` 경고): 어깨 roll 이 왼팔 axis2 [0, 95], 오른팔 axis9 [−95, 10] 로 비대칭 — 실측 부호(왼팔 옆으로 들기 = −)와 맞지 않으면 왼팔은 옆으로 전혀 못 든다 → 6-2b verify 로 확인하고 배포 yaml 을 받아 둘 것.

명령 경로(우리 `UpperBodyCommand` 1개 → 모터):
1. 브리지 `_on_upper_body_command`: `command_mode` 1(POSITION)·6 만 허용, **4/5 는 거부**("position-only controller"). 메시지마다 상태 JSON 파일을 읽고(디스크 I/O), 슬롯마다 IPC 데이터그램 `MOVE_TO_POS`(position 만 — **velocity·duration·kp/kd 는 버려짐**)를 보내며, enable 안 된 축(머리 90 등)에는 매번 `SET_RUN_MODE` 까지 보낸다 → 메시지당 15~16 데이터그램. 구독 QoS **RELIABLE depth 1**(그래서 best_effort 발행은 짝이 안 맞음; depth 1 이라 밀리면 최신 것만 남음).
2. `hr_controller` 메인 루프: 5 ms sleep → IPC 소켓 비우기(논블로킹) → 100 ms 마다 상태 JSON 쓰기. **MoveToPos 데이터그램마다 `IROBE_LOGI("[IPC] MoveToPos …")` 한 줄**(50 Hz 면 700줄/s, 200 Hz 면 3000줄/s — stdout 이 콘솔이면 그 자체가 부하). 소켓 큐 길이는 `net.unix.max_dgram_qlen`(Ubuntu 기본 10): 15개가 한꺼번에 들어오면 브리지의 `sendto` 가 **블로킹**돼 제어기가 비울 때까지 브리지(타이머 포함) 전체가 선다.
3. 코디네이터(5 ms) → 축 유닛(RT 1 kHz): `MoveToPos` 는 `target_pos_` 만 갱신. POSITION 모드에서는 **50 Hz 슬롯에서만** OD 0x7016(loc_ref)을 CAN 으로 쓴다(`send_periodic_non_impedance_ref`). 즉 **명령 주기는 50 Hz 가 상한**(zip 의 main 기준 — 미니PC 배포 브랜치 `feature/100hz-control` 은 `CAN_CTRL_HZ = 200` 으로 200 Hz, 6-1e 참고)이고, 추종은 모터 펌웨어의 위치 루프(limit_spd·loc_kp 는 제어기가 건드리지 않음 → 0.3 rad 스텝에 4.5 rad/s, 20 % 오버슈트가 그 기본값).
4. CAN 채널 프로세스(1 ms): TCP 소켓은 논블로킹 — 송신 버퍼가 차면 프레임을 **버리고** `tx_failed` 를 센다. 코디네이터가 1 s 쿨다운으로 `[HR-WARN] ethernet anomaly tcp(ch1/ch2)=… d_tx_failed=…` 를 콘솔에 찍는다.

**현장 확인(2026-10-21)**: verify 중 hr_controller 콘솔에 `[HR] event: command_commit_failed` 가 계속 찍혔다. 출처는 `HrCoordinator::commit_command` 의 "axis_id not monitored" — 우리가 팀 브리지를 따라 15번 슬롯에 넣어 보내던 머리 모터 90(미연결, axis 15)에 브리지가 메시지마다 `SET_RUN_MODE` + `MOVE_TO_POS` 2건을 보내고 제어기가 둘 다 거부한 것(50 Hz 면 100줄/s, 200 Hz 면 400줄/s). v0.7.19 부터 90 을 보내지 않고 빈 슬롯은 255 로 채운다(`extra_motor_ids: [0]`, `empty_slot_id: 255`; motor_calib 도 is_ready 모터만). 이 이벤트가 알려 준 더 중요한 사실: **메시지 하나가 코디네이터 명령 큐를 16건씩 통과**한다(축마다 1건). 코디네이터가 5 ms 주기에 처리하는 명령 수가 제한돼 있으면(프레임워크 내부, 소스 없음) 50 Hz × 16 = 800건/s 에서 큐가 밀려 **옛 명령이 지연 재생되고 넘치면 멈춤** — 관측과 정확히 같은 모양이고, 200 Hz(3200/s)에서 항상, 50 Hz 에서 경계(한 번은 매끄럽고 다음은 멈춤)인 것도 설명된다. 판정(v0.7.20): 주기를 20↔50 Hz 로 바꿔도 차이가 없었다는 현장 관찰(v0.7.15)이 있으므로 주기가 아니라 **메시지당 명령 건수**로 가른다 — `verify --joint elbow --delta 0.3 --only-motor`(시험 모터 하나만 = 메시지당 IPC 1건) 2회. 이것만 매번 `정지 구간 없음` 이면 명령 건수(코디네이터/IPC/이벤트) 한계 → 삼현에 IPC 묶음(15축 1 데이터그램 = 명령 1건) 요청이 핵심. 1건/메시지에서도 멈추면 IPC·코디네이터는 무죄이고 아래 TCP 쪽이다 — 축 유닛의 CAN 송신은 우리 송신 주기와 무관하게 축마다 50 Hz 로 돌기 때문에 TCP 쪽 장애는 **송신 주기와 무관**(20≈50≈200)하게 나타난다.

**현장 확인 2 (2026-10-21 오후)**: `--only-motor`(IPC 1건/메시지)에서도 멈췄고, 멈추는 동안 **`motor_states` 도 끊겼다**(브리지 타이머 정지 0.7~2 s) — 반면 명령을 전혀 보내지 않으면 `ros2 topic hz /upper_body/motor_states` 가 10 Hz·max 0.100 s 로 완전히 균일. 출력 파일 리다이렉트 + `max_dgram_qlen=512` 로 재시작한 뒤에도 같음. 즉 **명령이 흐를 때만 브리지 프로세스가 통째로 언다**. IPC 큐·콘솔·송신 주기·명령 건수·CANET200 TCP 는 모두 제외. 남은 후보: 명령 콜백 안의 어느 단계(상태 JSON 읽기 = ext4 /tmp, sendto), rclpy/DDS 내부, 명령 중에만 커지는 CPU 부하(CAN TX/RX 가 두 배). **어디서 어는지는 `tools/hr_bridge_timed.py` 로 잰다** — 삼현 브리지를 수정 없이 감싸 20 ms 넘는 단계, 타이머 공백, 프로세스 전체 정지(감시 스레드)를 찍는다. 미니PC 로 가져가기: `scp sejong@192.168.10.1:/home/sejong/workspaces/isaac_ros-dev/src/quest_arm_ik/tools/hr_bridge_timed.py ~/hr_controller/tools/`, 실행법은 파일 머리의 주석.

**원인 확정 (2026-10-21 12:18~12:24, `tools/hr_bridge_timed.py` v3)**: 멈춤 2.0~2.3 s 마다 `PROCESS STALL` + `_tick` + **`publish[axis_map_pub]`**(한 번은 `publish[motor_pub]`)이 같은 시각에 찍혔다. 즉 브리지가 어는 자리는 **RELIABLE 로 발행하는 상태 토픽의 `publish()`** 이고, BEST_EFFORT 인 `joint_states` 와 작은 `system_state` 는 한 번도 안 막혔다. 공유메모리 끔(`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`), 로그 레벨 error, rosout/파일 로그 끔 — 모두 변화 없음. 해석: RELIABLE 발행자는 구독자의 ACK 를 기다리는데, 허브 위에서 명령(Thor→미니PC)과 상태(미니PC→Thor)가 양방향으로 흐를 때 패킷(특히 ~4 KB JSON `axis_map` 의 IP 조각)이 빠지면 재전송·ACK 가 끝날 때까지 `publish()` 가 GIL 을 쥔 채 막혀 Python 브리지 전체가 선다. 명령이 없으면 충돌이 없어 멀쩡했고, 송신 주기·명령 건수·IPC 와 무관했던 것도 설명된다. 그때 RELIABLE 구독자는 **우리 motor_calib**(axis_map·system_state) 였다(`ros2 topic echo/hz` 도 기본 RELIABLE 구독자다).
**조치**: ① (우리, v0.7.23) 상태 토픽 구독을 전부 BEST_EFFORT 로 — motor_calib 의 system_state·axis_map, 스트리머의 system_state. 이제 우리 쪽엔 브리지를 막을 RELIABLE 구독자가 없다. 운영 중 `ros2 topic echo/hz /upper_body/…` 를 쓸 땐 반드시 `--qos-reliability best_effort` 를 붙인다. ② (삼현) 상태 토픽을 BEST_EFFORT(sensor-data QoS) 로 발행하고, 정적인 `axis_map` 은 10 Hz 대신 변경 시/1 Hz 로, `max_blocking_time` 을 짧게. ③ (네트워크) 허브 → 스위치, 또는 Thor↔미니PC 직결 — 조각 유실 자체를 줄인다. Thor 호스트에서 `grep ^Ip: /proc/net/snmp` 의 `ReasmFails` 가 verify 중 늘면 조각 유실이 실제로 있는 것.

'멈췄다가 옛 명령부터 점프' 의 둘째 후보(제외됨): **CANET200 TCP 링크의 재전송 백오프**. 허브를 Thor 트래픽과 같이 쓰는 TCP 가 패킷을 하나 잃으면 RTO(200 ms→400→800→1.6 s…) 동안 전송이 멈추고(모터는 마지막 loc_ref 유지 = 정지), 복구되면 커널 송신 버퍼에 쌓였던 옛 loc_ref 들이 순서대로 나간 뒤 현재 값이 간다(= 옛 값 거쳐 점프). 길이 0.5~2.5 s, 간헐성, 송신 주기 무관, 팀 브리지 때는 안 났을 수 있음(그때 네트워크 구성이 달랐다면) 이 모두 맞는다. 둘째 후보는 2 의 콘솔 로그+소켓 큐 블로킹(이건 상태 갱신도 같이 멈춤).

바로 확인(모두 미니PC, 비용 0):
```bash
cat /proc/sys/net/unix/max_dgram_qlen                      # 10 이면 sudo sysctl -w net.unix.max_dgram_qlen=512 (영구: /etc/sysctl.d/99-hr.conf)
ss -ti dst 192.168.0.7 | grep -o "retrans:[^ ]*\|rto:[^ ]*\|lost:[^ ]*"   # verify 전후 비교: retrans 가 늘면 CANET200 TCP 유실
ip -s link show enp1s0; ip -s link show eno1                # RX/TX errors·dropped
```
그리고 verify 를 돌리는 동안 **hr_controller 콘솔(tty1)에 `[HR-WARN] ethernet anomaly` 나 `[CANET200] … TCP send error` 가 찍히는지** 본다. 찍히면 확정 → CANET200 ↔ enp1s0 를 **직결 케이블**(허브 밖)로, Thor ↔ eno1 도 따로. 삼현 요청: ⓪ **IPC 를 15축 한 묶음**(데이터그램 1개 → 코디네이터 명령 1건)으로, 그리고 코디네이터의 주기당 명령 처리 한도 확인 ① `main.cpp` 433행 `IROBE_LOGI("[IPC] MoveToPos …")` 제거(또는 OD_VERBOSE_LOG 게이트) ② `ros2_state_bridge.py`: 메시지마다 JSON 재읽기 제거(타이머 값 캐시), enable 안 된 축 건너뛰기(매번 SET_RUN_MODE 금지), 가능하면 15축 한 묶음 IPC 1개 ③ `max_dgram_qlen` 512 ④ 상태 JSON 100 ms → 20 ms, 브리지 `--rate 50` ⑤ POSITION 모드에서 `limit_spd`(0x7017)·`loc_kp`(0x701E) 를 config 로 쓰게 하거나 `SetMitTarget` 스트리밍(모드 4/5) 허용 ⑥ 배포 `hr_controller.yaml` 의 axis2/axis9 한계 부호 확인.

### 6-1d. UDP 링크 (v0.7.24) — 미니PC 의 삼현 DDS 브리지 대신 우리 파이썬 브리지 (제어기를 못 고칠 때의 대안; 권장은 6-1e)
2026-10-21 하루의 결론: 링크 위의 Fast DDS 발행(`publish()`)이 명령이 흐를 때마다 0.3~2.5 s 씩 사용자 공간 락에서 막히고(QoS·전송·로그·IPC·케이블·EEE 와 무관하게 재현), 원인을 Fast DDS 밖에서 고칠 수 없었다. 그래서 **DDS 를 링크에 태우지 않는다**: 미니PC 에는 ROS 가 전혀 필요 없는 `tools/hr_udp_bridge.py`(표준 라이브러리만, 단일 스레드 select, 어디서도 기다리지 않음)를 두고, Thor 안의 `hr_udp_relay` 가 삼현 브리지와 **같은 토픽·서비스 이름**을 제공한다. 스트리머·IK·motor_calib·이 런북의 절차는 바꿀 것이 없다.
```
[Thor] upper_body_streamer ─/upper_body/command─▶ hr_udp_relay ══UDP CMD(≤87 B, 50 Hz)══▶ [미니PC] hr_udp_bridge.py ─IPC MoveToPos─▶ hr_controller
       dual_arm_ik/motor_calib ◀─/upper_body/motor_states·system_state·joint_states·axis_map─ hr_udp_relay ◀══UDP STATE(324 B, 20 Hz)══ hr_udp_bridge.py ◀─/tmp/hr_controller_state.json(100 ms)─ hr_controller
       enable/disable/clear_fault/emergency_stop/home_axis/set_run_mode 서비스 ══UDP SVC/ACK══ (IPC 그대로 전달)
```
패킷은 전부 이더넷 프레임 하나(IP 조각 없음), 유실되면 다음 것이 온다. 미연결 축(머리 90)과 enable 안 된 축엔 명령을 보내지 않는다. `command_mode` 는 1(POSITION) 만.

**미니PC (한 번만)**: Thor 홈에 올린 파일을 가져온다 — `scp sejong@192.168.10.1:~/workspaces/isaac_ros-dev/src/quest_arm_ik/tools/hr_udp_bridge.py ~/hr_controller/tools/`. 그 뒤 매번:
```bash
# tty1: ./run_hr_controller.sh > ~/hr.log 2>&1          (전과 같음)
# tty2: 삼현 ros2_state_bridge.py 는 띄우지 않는다. 대신
python3 ~/hr_controller/tools/hr_udp_bridge.py         # 5 s 마다 상태 한 줄: peer=…  cmd 50/s  ipc 700/s  state 20/s  fsm=…  enabled 14
```
`peer` 는 Thor 가 보내는 HELLO 로 자동 등록된다(고정하려면 `--peer 192.168.10.1:7002`). `net.unix.max_dgram_qlen` 이 10 이면 명령마다 IPC 14개가 몰려 큐가 차므로 재시도로 버티지만, `echo 'net.unix.max_dgram_qlen = 512' | sudo tee /etc/sysctl.d/99-hr.conf && sudo sysctl --system` 으로 영구 설정해 두는 것이 좋다. 부팅 때 자동으로 띄우려면 `tools/install_hr_systemd_services.sh` 의 브리지 서비스 ExecStart 를 이 스크립트로 바꾼다.

**Thor**: `env.sh` 가 `HR_LINK=udp` 와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`(DDS 를 Thor 안에만) 를 기본으로 잡는다. `launch … hr:=true` 는 `hr_udp_relay` 를 같이 띄우고(`sim:=true` 면 불필요), motor_calib 를 쓸 땐 터미널 하나에서 먼저:
```bash
ros2 run quest_arm_ik hr_udp_relay                       # 5 s 마다: cmd … Hz → 미니PC, state … Hz ← 미니PC (마지막 … s 전, fsm …)
ros2 topic echo /upper_body/system_state --once          # Thor 안의 토픽이라 QoS 걱정 없이 바로 됨
```
미니PC 주소가 다르면 `--ros-args -p peer_host:=…` 또는 config `hr_udp_relay.peer_host`. 삼현 DDS 브리지로 되돌리려면 `HR_LINK=dds` 로 env.sh 를 소싱하고 launch 에 `hr_link:=dds`.

**검증(ROS 없이)**: `SH_REPO=$SH $QPY test/test_udp_bridge.py` — 가짜 HRController(IPC 소켓 + 상태 JSON) + 실제 `hr_udp_bridge.py` 서브프로세스 + 가짜 rclpy 위의 릴레이로, 상태 50 Hz 수신·enable→ENABLED·명령 50 Hz → IPC MoveToPos 축당 1건·set_run_mode/clear_fault/disable 까지 한 번에 돈다(2026-10-21 통과).

**삼현에 알릴 것**: 미니PC 에서 ros2_state_bridge.py 대신 hr_udp_bridge.py 를 쓴다(제어기·IPC·상태 파일은 그대로, 수정 없음). 대시보드(rosbridge)가 필요하면 삼현 브리지를 **상태 발행 전용**으로(명령 구독을 끄고) 같이 띄울 수 있다.

### 6-1e. 미니PC 프로그램 v2 — hr_controller 패치 (`tools/hr_controller_v2/hr_controller_demo.patch`, 권장)
미니PC 의 삼현 스택을 통째로 살펴본 결론(2026-10-22). 구성은 ① `hr_controller`(C++ 7천 줄, iRobe 런타임 위: 코디네이터 5 ms + 축 유닛 1 kHz + CANET200 TCP I/O 1 ms) ② 파이썬 도구들(`tools/ros2_state_bridge.py` = DDS 브리지, sine 시험기, axis 도구, rosbridge 대시보드) ③ systemd 유닛 3개. 시연에 필요한 것은 ① 뿐이고, 문제는 전부 ①↔Thor 사이의 **경로**(DDS 브리지 → JSON 파일 → IPC → 코디네이터)였다. 그래서 제어기 본체(CAN 프로토콜·FSM·축 유닛·안전 로직)는 손대지 않고, 경로를 제어기 안으로 끌어들이는 패치를 만들었다 — 7파일, `git apply` 한 번.

**기준 소스에 대한 교정(2026-10-22)**: 삼현이 준 zip(main, 7/1)과 달리 **미니PC 에 배포된 것은 `feature/100hz-control` 브랜치(f29a434)** 다. 거기서는 `CAN_CTRL_HZ = 200` 이고 축 유닛의 지령 슬롯이 `PERIODIC_TX_HZ = CAN_CTRL_HZ` 라 **loc_ref 를 이미 200 Hz 로 쓴다**(idle 폴링도 200 Hz), `[IPC] MoveToPos` 로그도 이미 없고, 코디네이터가 10 s 마다 `[HR] CAN frames(10s) ch1 tx=… (+N) rx=… | ch2 …` 를 찍으며, config 의 축 한계는 전부 ±180° 다(6-1c 의 axis2/axis9 비대칭 걱정은 해당 없음). 6-1c 의 "50 Hz 상한" 은 zip 기준의 분석이었고, 그래서 처음 만든 패치(zip 기준)는 세 파일이 맞지 않아 적용되지 않았다 → 미니PC 소스(`minipc_src.tgz`)를 받아 다시 만든 것이 지금 파일이다. 지령 주기·로그 변경은 뺐다.

| # | 바뀌는 것 | 효과 |
|---|---|---|
| 1 | **Thor 직결 UDP 링크 내장** (`hr/udp/hr_udp_link.hpp/.cpp` 새 파일, `main.cpp`): 0.0.0.0:7001 에서 `hr_udp_relay` 의 CMD/SVC/HELLO 를 받고 STATE 를 **100 Hz** 로 보낸다. 링크 명령은 기존 `IpcCommandPacket` 으로 바꿔 `dispatch_ipc_command()` 로 넘긴다 | 미니PC 에 파이썬·ROS 프로세스가 **하나도 필요 없다**. JSON(100 ms)·IPC 큐(`max_dgram_qlen`)·DDS 가 경로에서 빠진다. 측정값 지연 100~200 ms → 10 ms 이하 |
| 2 | 메인 루프 `sleep 5 ms` → `poll(IPC, UDP, 1 ms)` | 명령이 도착하는 즉시 코디네이터로 (최대 5 ms 대기 제거). IPC 소켓·JSON 은 그대로 → 삼현 도구·대시보드도 그대로 돈다 |
| 3 | enable 때 위치 루프 OD 튜닝 쓰기(config `position_tuning`, 0 = 유지) + **네 값 읽어 로그** `[Axis n] OD 0x7017 limit_spd = …` | 드라이버의 현재 `limit_spd`·`loc_kp`·`vel_max_pp`·`acc_set_pp` 를 처음으로 볼 수 있고, 필요하면 config 로 바꾼다 |

**안 바꾼 것(일부러)**: 지령 슬롯 200 Hz(이미 그렇게 돌고 있고 CAN 부하 ≈ 40 %, `[HR-WARN] ethernet anomaly` 없이 돌았음), 코디네이터 주기 5 ms, CAN 프로토콜·enable 시퀀스·안전 홀드·boot scan, `config/hr_controller.yaml`(생략된 키는 기본값: 링크 켜짐·7001·100 Hz·튜닝 0). 미니PC 의 빌드 환경은 그대로 있다(`/usr/bin/c++`·ninja·cmake, iRobe SDK `/opt/irobe/0.1.0`, yaml-cpp — `build/release/CMakeCache.txt`), 인터넷 불필요.

**검증(여기서 한 것)**: 링크 모듈은 SDK 없이 컴파일되는 순수 C++ 라 (a) `test/test_udp_bridge.py --cpp` 가 같은 소스를 g++ 로 빌드한 가짜 제어기를 띄워 Thor 쪽 `hr_udp_relay` 와 실제 프로토콜로 상태 100 Hz·enable·명령 33개 → 14축 462건·set_run_mode·disable 까지 돈다(2026-10-22 통과, `--all` 이면 파이썬 브리지 경로도). (b) 미니PC 소스 전체(패치 적용본)를 iRobe SDK 의 사용 부분만 흉내낸 스텁 헤더로 `-fsyntax-only -Wall -Wextra -Wpedantic` 통과(9개 .cpp, 경고 0), 패치는 미니PC 소스 사본에 `git apply --check` / `git apply -R` 왕복 확인. 실제 SDK 로 링크·실행은 미니PC 에서 처음 하므로, `cmake --build` 가 오류를 내면 그 출력을 그대로 보내 주면 된다.

**미니PC 절차** (패치 파일 머리에도 있음):
```bash
scp sejong@192.168.10.1:~/workspaces/isaac_ros-dev/src/quest_arm_ik/tools/hr_controller_v2/hr_controller_demo.patch ~/
cd ~/hr_controller && git status --short src include CMakeLists.txt   # 아무것도 안 나와야 함
git apply --check ~/hr_controller_demo.patch           # 조용하면 OK
git apply ~/hr_controller_demo.patch && git diff --stat   # 5 files changed (+ 새 파일 2개는 ??)
cmake --build build/release                            # 수십 초
# tty1: Ctrl-C 로 제어기 종료 → ./run_hr_controller.sh > ~/hr.log 2>&1     (tty2 의 파이썬 브리지는 띄우지 않는다)
tail -F ~/hr.log                                       # [HR-UDP] listening on 0.0.0.0:7001, STATE 100 Hz → peer (none)
```
되돌리기: `git apply -R ~/hr_controller_demo.patch && cmake --build build/release`(config 는 건드리지 않음). 파이썬 브리지(6-1d)로 돌아가려면 config 에 `udp_link: {enabled: false}` 만 넣어도 된다(7001 포트를 제어기가 놓는다). Thor 와 직결 링크가 "No route to host" 면 Thor 호스트의 직결 포트에 192.168.10.1 이 없는 것 — `sudo nmcli con add type ethernet ifname <포트> con-name hr-link ipv4.method manual ipv4.addresses 192.168.10.1/24 ipv6.method ignore && sudo nmcli con up hr-link` 로 고정.

**Thor 절차**: 바뀌는 것 없음 — `ros2 run quest_arm_ik hr_udp_relay`(peer 192.168.10.2:7001) 가 그대로 붙고 "state 100 Hz" 가 찍힌다. `config/sh_humanoid_v2.yaml` 의 `upper_body_streamer.rate: 100.0`(IK 와 1:1), verify 는 `--rate 100`; 슬롯이 200 Hz 이므로 `--rate 200` 도 verify 로 시험해 볼 수 있다(코디네이터 5 ms 와 같은 주기라 목표가 슬롯에 0/2개씩 걸리는 맥놀이는 있음). 확인 순서: ① hr.log 에 `peer 192.168.10.1:7002 registered (HELLO)` ② `ros2 topic hz /upper_body/motor_states` ≈ 100 ③ `ros2 service call /upper_body/enable_motors …` → hr.log 에 `[HR-UDP] svc 1 … → ok`, `[HR] EnableAll: 14 axes ON → …`, 축마다 `OD 0x7017 limit_spd = …` 4줄 ④ `motor_calib -- verify --config /workspaces/isaac_ros-dev/src/quest_arm_ik/config/sh_humanoid_v2.yaml --joint elbow --delta 0.3 --rate 100` → 정지 구간 없음, 5 s 마다 hr.log 의 `[HR-UDP] … cmd 100/s (entries 1400/s, dispatched 1400/s, skipped_disabled 0, unknown_motor 0)` ⑤ `[HR] CAN frames(10s)` 의 (+N) 은 7축 채널에서 idle 이든 명령 중이든 +14000(= 7 × 200 × 10) — 변하면 축이 빠지거나 CAN 이 막힌 것.

**실측 (2026-10-22, 패치 적용 후 `verify --joint elbow --delta 0.3 --rate 100`)**: 송신 간격 평균 10.0 / 최대 10.1 ms, motor_states 수신 103 Hz, 첫 명령 후 움찔 1.9 mrad, **정지 구간 없음·공백 없음**(어제의 0.3~2.5 s 정지가 사라짐), 측정 속도 최대 0.230 rad/s = 명령 0.226, 끝 오차 1.1 mrad. 추종 지연은 드라이버 위치 루프 몫으로 ≈ 0.12 s(0.22 rad/s 에서 최대 31 mrad 뒤짐) — `position_tuning`(loc_kp 등) 으로 줄일 수 있는 부분. CANET200 의 CAN package time 을 1 ms 로 바꾼 전후: 최대 오차 43 → 31 mrad, 끝 오차 3.0 → 1.1 mrad (피드백 지연 감소) — 1 ms 유지. 요약의 "motor_states 갱신 ≈ 33 Hz" 는 **motor_calib 의 집계 오류**였다(rclpy.spin_once 가 호출당 콜백 하나만 처리해 100 Hz 토픽 셋 중 하나씩만 밀려 들어옴; v0.7.28 `spin_pending` 으로 수정) — 실제 상태는 100 Hz 로 오고 있었다.

**부하**(채널당 7축, OD write 마다 0x02 ACK 1프레임 = 축당 왕복 2프레임, 확장 프레임 ≈ 0.14 ms @ 1 Mbps): 200 Hz 2800/s ≈ 40 % — 지금 돌고 있는 값. CANET200 은 8000 프레임/s 까지(제조사 사양), 제어기 I/O 스레드는 1 ms 에 50 프레임 → 한계 아님. 축별 `tx2fb=…ms`(송신→피드백) 가 30 ms 를 넘으면 `[HR-WARN] … jitter` — CANET200 의 CAN→이더넷 패키징 시간이 기본 10 ms 라 10 ms 안팎이 정상(변환기 웹 설정 192.168.0.7 에서 package time 을 1 ms 로 줄이면 피드백 지연도 준다 — 제어기 수정 아님).

**드라이버 튜닝값 실측(2026-10-26 enable read-back, v2.2 패치 적용 후)** — 축마다 다르고 왼팔 yaw 두 축은 사실상 꺼져 있다:

| 축 | 관절 | loc_kp | 지연 ≈ 1/kp | acc_set_pp | | 축 | 관절 | loc_kp | 지연 | acc_set_pp |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | L 어깨 pitch | 17.6 | 57 ms | **1.7** | | 8 | R 어깨 pitch | 14.6 | 68 ms | **1.7** |
| 2 | L 어깨 roll | 14.6 | 68 ms | **2.0** | | 9 | R 어깨 roll | 11.7 | 85 ms | **2.0** |
| 3 | L 어깨 yaw | **0.57** | **1.8 s** | 5.0 | | 10 | R 어깨 yaw | 4.5 | 220 ms | 5.0 |
| 4 | L 팔꿈치 | 8.8 | 114 ms | 5.0 | | 11 | R 팔꿈치 | 6.8 | 146 ms | 5.0 |
| 5 | L 손목 yaw | **0.57** | **1.8 s** | 5.0 | | 12 | R 손목 yaw | 2.6 | 380 ms | 5.0 |
| 6 | L 손목 roll | 5.1 | 195 ms | 5.0 | | 13 | R 손목 roll | 6.8 | 146 ms | 5.0 |
| 7 | L 손목 pitch | 2.9 | 340 ms | 5.0 | | 14 | R 손목 pitch | 4.9 | 205 ms | 5.0 |

`limit_spd` = `vel_max_pp` = 18.64 rad/s(그대로 둠). diag_latency 의 "드라이브 지연 50~60 ms" 는 어깨가 지배한 평균이고, 손목·yaw 는 0.2~0.4 s, 왼팔 yaw 두 축은 거의 안 따라온다(왼팔이 오른팔과 다르게 움직이던 원인 후보 1순위).
어깨의 acc 1.7~2.0 rad/s² 는 1 rad/s 명령 변화에 0.5 s 가 걸려 v_max 1.0 에서 추종 오차 0.27 rad·재동기화 반복("불안정") 을 만든 원인.

조정은 `tools/hr_controller_v2/tune_axes.py`(미니PC `~/` 에 복사) 로 단계별로 config 의 축 항목에 키를 넣는다(원본 `.bak`, `--stage off` 로 복귀):
```bash
python3 ~/tune_axes.py --stage acc     # 1단계: 가속만 — 어깨 1.7~2.0 → 4.0, 나머지 5.0 → 8.0 → 재시작·enable·read-back → verify --config … --delta2 0.5 --velocity 1.0 (6-1f D) → V_MAX=0.7 diag
python3 ~/tune_axes.py --stage kp1     # 2단계: + loc_kp 1차 (×1.5, 3·5번 축은 5.0/4.0) → 같은 검사 → V_MAX=1.0 (diag 는 -p v_max:=1.0)
python3 ~/tune_axes.py --stage kp2     # 3단계: 어깨 acc 6.0·나머지 10.0, loc_kp 바닥 6~10·어깨 22~24 — 2단계가 떨림·소음 없을 때만
```
판정: 떨림·웅웅 소리 → 그 축만 `--set 축=값` 으로 25 % 낮춤; verify 의 과도(측정 속도 ≫ 명령)·오버슈트 10 % 초과 → acc 를 한 단계 아래로. 목표: v_max 1.0 에서 드라이브 지연 ≤ 60 ms, 추종 오차 최대 ≤ 0.1 rad, 재동기화 없음.
삼현에 물을 것: 3·5번 축 loc_kp 0.57 이 의도인지(verify 로는 다른 축과 같은 50 ms 지연 — read-back 값이 실제 쓰이는 게인인지), 위치 루프에 적분항이 있는지(±0.5 rad 유지에서 중력 쪽으로 13~20 mrad 처짐), loc_kp·acc_set_pp 의 권장 상한.

**삼현에 알릴 것**: 배포 브랜치(feature/100hz-control) 위에 UDP 링크 모듈을 붙였다(패치 파일 전달, 7파일, FSM·CAN·지령 주기·안전 로직 무수정). 파이썬 브리지·rosbridge 가 필요하면 config `udp_link.enabled: false` 로 종전 구성이 그대로 된다. systemd 로 자동 기동하려면 `tools/install_hr_systemd_services.sh` 가 만든 유닛 중 `hr_controller.service` 만 켠다(`sudo systemctl enable --now hr_controller; sudo systemctl disable --now hr_ros2_state_bridge hr_rosbridge`).

### 6-1f. 드라이브 튜닝 — 단계별 상세 절차 (맥북 ssh + tmux 기준)
전제: v2.1+v2.2 패치 적용·빌드 완료, 미니PC `~/tune_axes.py`, Thor tmux `demo`(2 relay · 3 enable · 4 teleop · 5 diag), 미니PC tmux `hr`(1 ctrl · 2 log). 팔은 내린 자세, 팔 반경 비움, 비상정지 담당.
각 단계는 같은 7걸음이다 — **A 값 넣기 → B 제어기 재시작 → C enable·read-back → D verify → E 텔레옵 diag → F 판정 → G 다음 단계 또는 되돌리기**.

**A. 값 넣기 (미니PC, 창 2)** `python3 ~/tune_axes.py --stage acc` → "14개 변경" 과 축별 값이 찍히고 `config/hr_controller.yaml` 이 바뀐다(처음 한 번 `.bak` 생성). 넣기 전에 보려면 `--dry-run`.
**B. 제어기 재시작 (미니PC, 창 1)** `Ctrl-C` → 팔이 그대로 서 있는지 확인(탈력되면 받칠 것) → `Enter`(입력돼 있는 `./run_hr_controller.sh > ~/hr.log 2>&1`) → 창 2 에서 `grep "position_tuning on enable" ~/hr.log` 에 축마다 `acc_set_pp=4`/`8`(kp 단계에선 `loc_kp=…`) 가 보여야 한다. 안 보이면 yaml 들여쓰기 확인(`--stage off` 로 되돌리고 다시).
**C-0. read-back 주의(2026-10-26 실측)**: v2.2 까지는 OD 쓰기 직후 같은 묶음으로 읽어, 어깨 pitch 드라이브(축 1·8)는 **이전 값**을 돌려준다(4.0 을 쓰고 1.7 로 보이고, 다음에 2.0 을 쓰면 4.0 으로 보임 — 쓰기는 된다).
`hr_controller_tuning_readback.patch`(v2.3, 2파일) 를 덧붙이면 enable 0.5 s 뒤에 읽어 실제 값이 찍힌다. 패치 전이라면 disable → enable 을 한 번 더 하고 마지막 read-back 을 본다.
**C. enable·read-back (Thor 창 3 → 미니PC 창 2)** `ros2 service call /upper_body/enable_motors …` 뒤 `grep -h "OD 0x701E\|OD 0x7025" ~/hr.log | sort -u` — 드라이브가 **실제로 받은 값**이 14줄씩 보여야 한다(이 줄은 enable 때 읽어 온 값). 한 축이라도 옛 값이면 그 축 드라이브가 쓰기를 거부한 것 → 삼현에 확인.
**D. verify (Thor 창 5; teleop launch 는 반드시 내려 둠 — 명령 발행자가 둘이면 안 됨)** `ros2 run quest_arm_ik motor_calib -- verify --config /workspaces/isaac_ros-dev/src/quest_arm_ik/config/sh_humanoid_v2.yaml --delta2 0.5 --delta 0.3 --velocity 1.0 --rate 100` — 관절마다 Enter 로 진행(`--config` 는 v0.7.43 부터 생략 가능 — 설치된 share 의 같은 파일을 쓴다; 그 전 버전은 `--config <sh_humanoid_v2.yaml> 필요` 로 멈춘다). 관절마다 요약 세 줄을 본다:
  `명령-측정 차: 최대 X mrad, 끝에서 Y mrad; 측정 속도 최대 V rad/s (명령 최대 속도 ≈ W)` → **X ÷ W ≈ 추종 지연**(mrad 끼리 비교하지 말 것 — 속도가 다르면 mrad 도 다르다; 10/22 의 31 mrad 는 0.22 rad/s 에서라 140 ms 였다). 목표 지연 ≤ 60 ms = 1.0 rad/s 에서 X ≤ 60 mrad(14축 실측 36~60 ms — 아래 표). **V 가 W 의 1.5배를 넘으면 오버슈트**(드라이브가 자체 프로파일로 뛰는 것) → 그 축 acc 를 한 단계 낮춤. `정지 구간 없음` 유지. 눈과 귀: 목표를 지나쳤다 돌아오면 오버슈트, 멈춘 뒤 **웅웅·지잉 소리/미세 진동**이면 kp 과다 → 그 축만 낮춤.
  `0.5 s 구간 끝 [명령 → 측정]` 줄에서 **유지 구간(같은 명령이 두 번 이어지는 곳)의 차**도 본다 — 1번 축 실측(2026-10-26, acc 4.0, loc_kp 17.6): 최대 47.6 mrad @ 1.0 rad/s = 48 ms, 위치 오버슈트 없음, 100 Hz·공백·정지 없음, 측정 속도 최대 1.38(따라잡기 구간; acc 가 2.0 이었다면 0.2 rad 가까이 뒤처졌을 것 → 4.0 이 들어갔다는 증거). 그런데 **±0.5 rad 유지 중 14~15 mrad 가 중력 쪽으로 모자라고 0 rad 에서는 0** — 위치 루프가 P 만이라 하중에 비례해 처지는 것(손끝 ≈ 9 mm). loc_kp 에 반비례해 줄 뿐 없어지지 않는다(왼쪽 15 mrad @ kp 17.6 ↔ 오른쪽 20 mrad @ 14.6); 더 줄이려면 스트리머에 느린 외부 적분(측정−명령, 시정수 1~2 s)을 넣는 방법이 있다 — 시연엔 불필요. `틱당 변화 최대` 가 명령의 2배 정도는 피드백 프레임이 한 틱에 둘 들어온 타이밍 지터일 수 있으니 3배 기준만 본다.
**D 실측 — acc 단계 14축 (2026-10-26, `--delta2 0.5 --delta 0.3 --velocity 1.0 --rate 100`; 지연 = 최대 오차 ÷ 1.0 rad/s, 잔차 = 1 s 유지 구간 끝의 명령−측정)**:

| 축 | 관절 | loc_kp(read) | acc | 최대 오차 → 지연 | 측정 v / 명령 v | 유지 잔차 | 비고 |
|---|---|---|---|---|---|---|---|
| 1 | L 어깨 pitch | 17.6 | 4.0 | 47.6 mrad → 48 ms | 1.38 | 15 mrad (±0.5, 중력 쪽) | 0.5 rad 구간 가속 5.3 rad/s² > 4.0 → 따라잡기 |
| 2 | L 어깨 roll | 14.6 | 4.0 | 52.9 → 53 | 0.91 | 12 (−0.3, 중력 쪽) | 0.3 rad 구간 가속 8.9 > 4.0 |
| 3 | L 어깨 yaw | **0.57** | 8.0 | 58.6 → 59 | 0.86 | 17 (+0.5) / 4 (−0.5) | read-back 0.57 인데 지연은 다른 축과 같음 |
| 4 | L 팔꿈치 | 8.8 | 8.0 | 38.2 → 38 | 0.73 | 16 (굽힘 0.5, 중력 쪽) | |
| 5 | L 손목 yaw | **0.57** | 8.0 | 55.4 → 55 | 0.84 | 10 / 15→5 (0.5 s 에 걸쳐 수렴) | 잔차가 더디게 줄어듦 |
| 6 | L 손목 roll | 5.1 | 8.0 | 45.0 → 45 | 0.94 | 8 / 11 | |
| 7 | L 손목 pitch | 2.9 | 8.0 | (71.6: 몸통 접촉) | 1.21 | 2 (바깥 0.45) | 안쪽 0.49 rad 에서 **몸통 접촉** → 정지 구간 2회 (손상 없음) |
| 8 | R 어깨 pitch | 14.6 | 4.0 | 49.6 → 50 | 0.78 | 20 (±0.5, 중력 쪽) | 1번과 같음 → 양쪽 acc 쓰기 확인 |
| 9 | R 어깨 roll | 11.7 | 4.0 | 60.3 → 60 | 1.08 | 13 (0.33, 중력 쪽) | |
| 10 | R 어깨 yaw | 4.5 | 8.0 | 39.7 → 40 | 0.81 | 1~2 | 가장 좋음 |
| 11 | R 팔꿈치 | 6.8 | 8.0 | 45.9 → 46 | 0.82 | 16 (굽힘 0.53, 중력 쪽) | |
| 12 | R 손목 yaw | 2.6 | 8.0 | 41.2 → 41 | 0.95 | 0~1 | |
| 13 | R 손목 roll | 6.8 | 8.0 | 35.5 → 36 | 0.81 | 4~5 | |
| 14 | R 손목 pitch | 4.9 | 8.0 | 39.1 → 39 | 1.09 | 4 (바깥 0.54) / 23 (안쪽 0.46 = 몸통 접촉) | |

읽기: ① 14축 모두 100 Hz·공백 없음·방향 맞음. ② **지연 36~60 ms 가 loc_kp 와 무관** — read-back 0.57(3·5번)과 17.6(1번)이 같은 50 ms 대. 따라서 '지연 ≈ 1/loc_kp' 는 틀렸고, 남은 지연의 큰 몫은 경로(Thor→미니PC→5 ms 슬롯→CAN→피드백→CANET200→100 Hz 상태) ≈ 15~25 ms + 드라이브 속도 루프이며, **kp 를 올려도 지연은 거의 안 준다**. ③ loc_kp 가 바꾸는 것은 유지 잔차(중력 처짐 ∝ 1/kp, 왼쪽 yaw 두 축의 더딘 수렴)뿐 → kp1·kp2 는 선택 사항이 됐고, 할 가치가 있는 것은 **3·5번만 오른팔 수준으로**(`--stage acc --set 3=5.0 --set 5=4.0`). ④ 어깨 네 축은 짧고 빠른 구간(필요 가속 5~9 rad/s²)에서 acc 4.0 이 걸려 50~60 mrad·속도 따라잡기(1.08~1.38) — v_max 1.0 텔레옵에서 어깨 재동기화가 나면 `--stage acc2`(어깨 6.0). ⑤ 손목 pitch 는 팔 내린 자세에서 안쪽 0.44~0.49 rad 에 몸통이 있다 — verify 는 v0.7.44 부터 안쪽 구간을 0.3 으로 제한(`--cap wrist_pitch=…`); 텔레옵은 손목 클러치(트리거)라 평소 잠겨 있음.
**E. 텔레옵 diag (Thor 창 4·5)** v0.7.47 이상(스트리머 제동 거리 법칙)이어야 한다 — 그 전 스트리머는 v_max ≥ 0.7 에서 멈춘 목표 둘레를 2 Hz 로 진동했다(6-6 6차). 스트리머 `a_max` 는 드라이브 acc_set_pp 의 최솟값(어깨 4.0) 이하(config 4.0; `ros2 param get /upper_body_streamer a_max`). 창 4 에서 ↑ 로 launch 명령을 불러 `v_max:=0.7`(통과하면 1.0) 로 고쳐 Enter → Quest 그립 → 창 5 `ros2 run quest_arm_ik diag_latency --ros-args -p v_max:=0.7` (v_max 를 맞춰야 포화율·점프 계산이 맞다). 창 4 로그에 `재동기화`·`HOLD` 가 없어야 한다.
**F. 판정** 통과 = 드라이브 지연 ≤ 60 ms, 추종 오차 최대 ≤ 0.10 rad, 재동기화 점프 0, 떨림 분석 '측정에 2~10 Hz 떨림 없음', D 에서 오버슈트·소음 없음. 실패 축이 있으면 `python3 ~/tune_axes.py --stage <같은 단계> --set <축>=<낮춘 kp>` (예 `--set 12=3.0`) 로 그 축만 25 % 낮춰 B 부터 반복.
**H. 이동 중 리플 스윕 (v0.7.48 verify 의 `이동 중 리플` 줄로; 7차에서 남은 3~5 Hz 드라이브 진동의 원인 가르기)** 오른쪽 어깨 pitch(축 8)와 팔꿈치(축 11) 두 관절만 돌린다. 한 조건에 1분:
  미니PC 창 2 `python3 ~/tune_axes.py --stage acc --acc 8=<acc> --acc 11=<acc> [--set 8=<kp> --set 11=<kp>]` → 창 1 Ctrl-C·Enter → Thor 창 3 enable → 창 5 `ros2 run quest_arm_ik motor_calib -- verify --config … --joint right_shoulder_pitch --delta2 0.5 --velocity 0.7 --rate 100` 과 `--joint right_elbow`. 조건: ① acc 6/8(지금) ② acc 4/4 ③ acc 2/5(원래값) ④ ②에서 kp ×0.7(8=10, 11=5) ⑤ ②에서 kp ×1.4(8=20, 11=10). 관절마다 `이동 중 리플 σ`(목표 < 0.1 rad/s)와 `명령-측정 차 최대`(지연) 를 적어 **리플이 0.1 아래이면서 지연이 가장 작은 조건**을 고른다. 리플이 acc 에 따라 변하면 드라이브 프로파일(PP) 헌팅 — acc 를 그 값으로 두고 스트리머 a_max 도 같게; kp 에 따라 변하면 위치 루프 링잉 — kp 를 낮춘다. 둘 다 아니면(acc 2 에서도 리플) 기구 공진 → 삼현에 속도 루프 게인·필터 확인 요청.
  **H 실측 (2026-10-26 22:00, 오른쪽 어깨 pitch / 팔꿈치, `--delta2 0.5 --velocity 0.7`)**:

| 조건 | 어깨 pitch: 리플 σ / 오차 최대 / 측정 v 최대 | 팔꿈치: 리플 σ / 오차 최대 / 측정 v 최대 |
|---|---|---|
| ① acc 6/8 | 0.048 (3.8 Hz) / 35.2 mrad / **0.98** | 0.036 (3.4 Hz) / 33.0 / 0.69 |
| ② acc 4/4 | 0.047 (3.8 Hz) / 33.3 / 0.55 | 0.036 (3.0 Hz) / 35.6 / 0.65 |
| ③ acc 2/5 (원래값) | 0.051 (2.8 Hz) / 37.3 / 0.59 | 0.038 (3.0 Hz) / 39.8 / 0.69 |

  읽기: ① **리플은 acc 와 무관**(0.036~0.051, 어느 조건이든 '양호' < 0.1) — 매끈한 단일 관절 명령에서는 드라이브 고유 리플이 ±5 mrad·3~4 Hz 수준뿐이다. ② 지연도 무관(33~40 mrad @ 0.7 = 48~53 ms). ③ acc 6/8 만 어깨 측정 속도 0.98(명령 0.70)의 따라잡기 과도가 있다 → **acc 4/8(`--stage acc`) 로 확정**, 스트리머 a_max 4.0 과 짝. ④ 그런데 텔레옵(7차)의 측정 리플은 σ 0.13~0.3 으로 단일 관절의 3~5배 — 차이는 IK 출력에 실려 오는 2~10 Hz 성분(0.05~0.066 rad/s; 조작자 손 자체의 2~5 Hz 움직임 10~25 mm/s 가 필터 5/10 을 거의 그대로 통과)과 다관절 동시 구동이다. 어깨 pitch/roll 은 2.5~3.5 Hz 에서 스트리머→측정 이득 1.2~2.4(코히어런스 0.5~0.8) — 드라이브 위치 루프가 3~4 Hz 에서 덜 감쇠돼 그 대역을 키운다. ⑤ 다음 손: (a) 같은 verify 로 kp ④ ×0.7 / ⑤ ×1.4 를 돌려 리플 피크의 높이·주파수가 kp 를 따라가는지(링잉이면 kp 를 낮춘 쪽이 유리, 처짐은 커짐) (b) 텔레옵에서 `filter_min_cutoff 3.0, filter_beta 5.0`(+10 ms)·`task_gain 0.6` 으로 명령의 2~5 Hz 를 줄여 떨림 분석 비교 (c) 삼현에 '3~4 Hz 위치 루프 링잉 — 속도 루프 게인/필터, 100 Hz 스트리밍에 맞는 모드(CSP 또는 MIT Kp/Kd)' 문의. 드라이브 모드 변경(MIT + 중력 FF)은 시연 뒤.
**G. 다음 단계 (2026-10-26 D 결과로 수정)** acc(어깨 4·나머지 8) 에서 E 를 v_max 0.7 → 1.0 으로 올린다. 1.0 에서 어깨 재동기화·추종 오차 > 0.1 rad 이면 `--stage acc2`(어깨 6.0). loc_kp 는 지연을 바꾸지 않으므로 kp1·kp2 는 선택(중력 처짐 ∝ 1/kp 만 조금 줄고 떨림 위험) — 제어기를 재시작하는 김에 `--stage acc --set 3=5.0 --set 5=4.0`(왼팔 yaw 두 축만 오른팔 수준)은 해 둘 만하다. 되돌리기는 `--stage off`(키 제거 → 드라이브 기본값) 또는 `cp config/hr_controller.yaml.bak config/hr_controller.yaml` 뒤 B. 최종값은 이 런북 6-1e 표 옆에 적는다.

### 6-2. 연결·인터페이스 확인 (30분) — 아직 모터 enable 안 함
```bash
ping 192.168.10.2
ros2 topic list | grep upper_body                  # /upper_body/command, motor_state, system_state 가 보여야 함 (미니PC 드라이버 실행 상태)
ros2 service list | grep upper_body                # enable_motors, disable_motors, set_run_mode
ros2 topic hz /upper_body/motor_states             # 발행률 (실기체 ≈14 Hz(70 ms) 로 실측 — 추종 감시·로그 해상도가 이 값에 묶인다; 100 Hz 이상 요청)
ros2 topic echo /upper_body/system_state --once    # state 2(READY)?  run_mode?  is_connected 15개?
ros2 interface show ros2_interfaces/msg/UpperBodyCommand   # 미니PC 쪽과 한 글자도 다르지 않아야 함 (ros2 interface show 를 양쪽에서 비교)
ros2 run quest_arm_ik motor_calib -- watch         # 15 모터 표: id·pos·vel·effort·temp·en/rdy/fault·err  (Ctrl-C 로 종료)
```
- `topic list` 에 안 보이면: 같은 `ROS_DOMAIN_ID`? 같은 RMW? 방화벽(미니PC Windows 면 UDP 7400~7500 허용)? 멀티캐스트가 막힌 망이면 양쪽에 `ROS_STATIC_PEERS=<상대 IP>` (Jazzy Fast DDS).
  `ros2 daemon stop` 후 다시 `topic list`. 토픽은 보이는데 `echo` 가 안 되면 QoS 또는 타입 해시 불일치(`ros2 topic info -v /upper_body/motor_states`).
- `watch` 를 띄운 채 **전원 꺼진(또는 disable 된) 팔을 손으로 천천히 움직여** 본다: 어느 motor_id 가 어느 관절인지, 값이 rad 인지(어깨를 90° 들었을 때 ≈1.57 이면 rad), 부호가 URDF 와 같은지(URDF 양의 방향은 `check_model.py`/MuJoCo 뷰어에서 `ros2 topic pub /teleop/joint_targets` 로 확인 가능) 를 적어 둔다. **이 단계에서 motor_ids 와 joint_signs 가 거의 다 정해진다.**
- 단위가 rad 가 아니면 알려 달라 — `MotorMap` 에 스케일 한 줄 추가(현재는 `모터각 = sign·q + offset`).

### 6-2b. 실측 결과 (2026-10-20, 모터 disable 상태에서 손으로 움직이며 `motor_calib -- watch`)
영점 = 양팔을 곧게 내린 자세(사진) = URDF 0 → `joint_offsets` 전부 0. 모터는 disable 상태에서 손으로 돌아감(역구동 가능).

| 관절 (controlled_joints 순서) | UID | 모터 + 의 물리 방향 (실측) | URDF + 의 물리 방향 (모델 축 분석) | joint_sign |
|---|---|---|---|---|
| left_shoulder_pitch | 34 | 앞으로 듦 | 앞으로 듦 | +1 |
| left_shoulder_roll | 113 | 안쪽(내림) | 안쪽 | +1 |
| left_shoulder_yaw | 125 | 바깥 회전 | 바깥 회전 | +1 |
| left_elbow | 52 | 굽힘(듦) | 폄 (URDF −가 굽힘) | −1 |
| left_wrist_yaw | 105 | 바깥 회전 | 안쪽 회전 | −1 |
| left_wrist_roll (6축, 축 −x: 앞뒤 흔들림) | 115 | 듦(앞으로) | 뒤로 | −1 |
| left_wrist_pitch (7축, 축 ±y: 좌우 흔들림) | 57 | 안쪽 굽힘 | 바깥 굽힘 | −1 |
| right_shoulder_pitch | 51 | 내림 | 앞으로 듦 | −1 |
| right_shoulder_roll | 32 | 바깥(옆으로 듦) | 안쪽 | −1 |
| right_shoulder_yaw | 33 | 안쪽 회전 | 바깥 회전 | −1 |
| right_elbow | 31 | 폄(내림) | 폄 | +1 |
| right_wrist_yaw | 35 | 안쪽 회전 | 안쪽 회전 | +1 |
| right_wrist_roll (13축) | 36 | 내림(뒤로) | 뒤로 | +1 |
| right_wrist_pitch (14축) | 37 | 바깥 굽힘 | 바깥 굽힘 | +1 |

왼팔 `[+,+,+,−,−,−,−]`, 오른팔 `[−,−,−,+,+,+,+]` 로 정확히 반대칭(거울 장착)이라 신뢰도가 높다. verify 의 시험 방향은 관절마다 안전한 쪽으로 잡는다 — 어깨 roll 은 v0.7.29 부터 **바깥쪽(옆으로 벌어짐, URDF −)** 으로 시험(안쪽은 몸통에 닿을 수 있음): 왼팔 모터 113 → −0.3, 오른팔 모터 32 → +0.3. v0.7.31 부터 손목 yaw(5축)·roll(6축)도 첫 이동을 URDF − 쪽(전완 바깥 회전 / 손 앞쪽으로 휨)으로 한다(현장 요청). 남은 불확실성 둘은 `verify` 로 확정한다:
(1) **확정(2026-10-23, 실기체 verify)**: 손목은 yaw(5축) → **roll(6축, UID 115/36)** → **pitch(7축, UID 57/37)** 순이고, URDF 는 pitch→roll 이었다. URDF 를 실기체 순서로 고쳤다(`SH-Humanoid_wrist_order.zip`: 두 관절의 원점은 그대로, 위 모듈 = roll(축 −x), 아래 모듈 = pitch(축 y), 손은 `*_wrist_pitch_link` 에; 0 자세 손 위치 불변, 0.5 rad 손목에서 손 위치가 39~44 mm 달라짐 = 그동안의 모델 오차). 이 패키지의 `controlled_joints`·motor_ids 등 배열도 같은 순서(…, wrist_yaw, wrist_roll, wrist_pitch)로 바꿔 motor_ids 가 제어기 axis 1~14 순서와 그대로 같아졌다. verify 도 5→6→7축 순으로 움직인다.
(2) '손목 드는 것' 을 '손끝이 앞으로' 로 해석한 것.
```bash
ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true, motor_id_list: []}"
ros2 run quest_arm_ik motor_calib -- verify --config /workspaces/isaac_ros-dev/src/quest_arm_ik/config/sh_humanoid_v2.yaml --rate 100 [--joint wrist] [--delta 0.3]
#   --delta2 0.5     양방향: 어깨 roll 을 뺀 모든 관절을 현재 위치에서 +0.5 → −0.5 → 0 (URDF, 한계 0.05 rad 여유 안으로 자름 — 곧게 편 팔꿈치는 − 쪽만). 어깨 roll 은 --delta 만큼 바깥쪽 한 번
#   --velocity 0.5   이동 피크 속도 [rad/s] (0.3 / 0.5 / 1.0 …; 구간 시간 = 1.5·이동각/속도, 최소 0.2 s). 없으면 --ramp(2 s/구간)
#   예) 전 관절 ±0.5 rad 를 0.5 rad/s 로:  … verify --config … --rate 100 --delta2 0.5 --delta 0.3 --velocity 0.5
```
관절마다 "기대 동작"을 보여 주고 URDF + 방향으로 0.15 rad(`--delta`) 움직였다 돌아온다. 같으면 Enter, 반대면 r(부호 뒤집기), 다른 관절이면 o. 끝에 뒤집을 관절 목록이 나온다.
**명령 형식은 팀의 `trajectory_bridge_node`(MoveIt→HRController 브리지)와 동일하게** 맞췄다(v0.7.11): command_mode POSITION, 15 슬롯 전부(축 순서, 미연결 90 포함), position + velocity 피드포워드, effort/kp/kd 0, `duration` = 송신 주기, frame_id `upper_body_link`. 보간은 이쪽에서 `--rate` 로(smoothstep) 한다. 스트리머도 `position_only: true` 가 기본이고, MIT/임피던스 시험은 `--kp/--kd` 또는 `position_only: false` 로만.
**'튐'의 원인은 송신 주기였다(2026-10-20, 팔꿈치 UID 52 verify 로그 3종 + jog 1종)**:
- 200 Hz(1.3 mrad/틱 smoothstep): 명령이 0.5~2.4 s 씩 통째로 무시되다가 '그때(또는 0.3~0.5 s 전) 명령' 으로 한 번에 점프 — 틱당 측정 139~186 mrad, 2 rad/s, 램프 끝 오차 25~229 mrad. 속도 피드포워드 유무(`--no-velocity`)와 무관.
- 명령 1개(`jog 52 0.3 --once --duration 2.0`): 0.25 s 안에 실행(최대 4.5 rad/s, +0.36 까지 20 % 오버슈트, 1.2 s 정착, 잔차 −7 mrad) → `duration` 은 무시되고, 내부 위치 루프는 빠르고 약간 언더댐핑. 즉 멈춤은 모터가 아니라 **받는 쪽이 200 Hz 를 소화하지 못해** 생긴 것(삼현이 본 '첫 명령 0.15 rad' = 그 앞 200개를 처리 못 한 것).
- **50 Hz: 1회는 매끄럽게 추종**(측정 속도 최대 0.27 vs 명령 0.24 rad/s, 지연 ≈0.13 s, 끝 오차 4 mrad) — 그러나 **재시도(v0.7.14)에서는 다시 멈춤**: 명령이 0.25 까지 가도록 1.5 s 정지 → 3.0 rad/s 로 점프, 내려올 때 2.5 s 정지 → 끝에서 점프. 즉 주기를 낮추면 덜할 뿐, 근본 원인은 **미니PC 브리지의 간헐적 처리 정지**다. config 는 팀 브리지와 같은 `rate: 50` 으로 두고 삼현이 브리지를 고쳐야 한다.
- **정지의 위치를 가르는 진단(v0.7.15)**: verify/jog 요약이 `정지 구간`(명령은 가는데 측정이 0.3 s 이상 안 움직임)과 그동안 `motor_states` 갱신 횟수를 찍는다 — 갱신 0회면 브리지(상태+명령 루프) 전체가 멈춘 것, 계속 오면 명령 경로(수신 콜백/명령 전달)만 멈춘 것. 또 하나 의심: **미니PC 콘솔(tty1) 로그 출력** — HRController 가 명령마다 콘솔에 로그를 찍으면 콘솔 출력이 느려 프로세스가 막힌다(매끄러웠던 1회는 콘솔이 tty2 로 전환돼 있던 때일 수 있음). 미니PC 를 tty2 로 둔 채 / tty1 에서 각각 `verify … --rate 50` 을 돌려 비교하고, tty2 의 `top` 에서 브리지 CPU 를 본다. 로그가 원인이면 `./run_hr_controller.sh > ~/hr.log 2>&1` 로 띄우거나 로그 레벨을 낮춘다.
  (콘솔을 tty2 로 넘긴 채 돌린 50 Hz 도 1.5 s 정지 + 끝 오차 118 mrad — 콘솔 로그는 원인이 아니었다.)
- (소스를 받기 전의 가설 — 6-1c 로 대체) RELIABLE QoS 의 유실 복구 대기(v0.7.16 `--best-effort`). 명령 발행이 RELIABLE(팀 브리지와 같은 기본)이면 UDP 패킷 하나가 유실될 때 구독자는 그것이 재전송될 때까지 **뒤의 명령을 모두 붙들었다가**(순서 보장) 한꺼번에 넘긴다 → 정확히 '멈췄다 점프' 이고, 메시지가 많을수록(200 Hz) 자주 난다. 유실은 허브(CAN 제어기와 공유)나 미니PC 의 UDP 수신 버퍼 넘침에서 생길 수 있다. 가르는 시험: Thor `ros2 topic info -v /upper_body/command` 로 드라이버 구독의 Reliability 를 본 뒤, `verify … --rate 50 --best-effort` 2회 — 매번 `정지 구간 없음` 이면 확정이고 config `command_reliability: best_effort` 로 바꾼다(위치 스트림은 최신 명령만 중요하므로 BEST_EFFORT 가 맞는 QoS). 드라이버 구독이 RELIABLE 이면 BEST_EFFORT 발행과 짝이 안 맞아 움직이지 않으니, 그땐 삼현에 구독을 BEST_EFFORT(sensor data QoS)로 바꿔 달라고 한다. 유실 자체는 미니PC `cat /proc/net/snmp | grep Udp`(RcvbufErrors/InErrors 가 verify 중 늘면 수신 버퍼 넘침)와 `ip -s link show eno1`(RX errors/dropped) 로 확인한다.
  삼현에는 로그(`/tmp/motor_calib_52.csv`, 200 Hz·50 Hz 각각)를 보내 브리지의 명령 처리 방식(스레드·큐·주기·구독 QoS)과 CPU 여유를 묻는다.
- **팀 `trajectory_bridge_node` / `_blend_node` 와의 비교(2026-10-20 검토)**: 형식·QoS(RELIABLE depth 10)·50 Hz 는 우리 스트리머와 같다. blend 판이 고친 것은 *연속된 MoveIt 궤적 사이의 시작 점프*(새 궤적 첫 점 ≠ 현재 명령값 → 0.3 s 동안 오프셋을 감쇠)이고, 우리 스트림은 측정값에서 출발해 `v_max·dt` 로 잇기 때문에 그 문제가 애초에 없다(verify 첫 1 s 변화 1~2 mrad). 남은 차이는 셋뿐: ① 팀 브리지는 **궤적 중에만** 발행(유휴 시 무발행), 우리는 유지 명령을 계속 발행 ② duration 0.005 고정 vs 우리 1/rate ③ C++ 타이머 vs Python 루프(v0.7.17 부터 절대 시각 페이싱, 요약에 `송신 간격` 출력). 이 중 ①은 `jog 52 0.3 --seconds 4`(움직일 때만 50 Hz 송신) 3회로 가를 수 있다 — 이것만 매번 매끄러우면 '연속 유지 명령' 이 드라이버에 문제를 일으키는 것. 팀 브리지가 매끄러웠던 당시 **어디서 돌았는지**(미니PC 안 = 공유 메모리/루프백이라 유실 없음, 또는 다른 PC 에서 이더넷)가 핵심 질문. 허브 의심은 Thor↔미니PC eno1 을 **직결 케이블**로 바꿔 verify 3회로 확인(CAN 제어기는 enp1s0 쪽 허브에 그대로).
참고: URDF 팔꿈치 한계가 [−1.75, −0.05] 라 곧게 편 영점(0)은 모델 밖 — IK 는 팔꿈치를 최소 0.1 rad 굽힌 값으로 시작하므로 첫 그립 때 팔꿈치가 약 6° 굽는 것이 정상.

### 6-3. 매핑 실측 → config (1시간)
1. 팔을 **URDF 0 자세**(양팔 아래로 곧게, 손바닥 안쪽 — 시작 자세와 같음; 팔꿈치만 0) 에 두고:
   `ros2 run quest_arm_ik motor_calib -- zero` → 출력된 `position` 이 `joint_offsets` 후보(모터각 = q + offset 이므로 0 자세의 모터각이 그대로 오프셋).
2. 모터를 enable (삼현 절차: `ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true}"`, 필요하면 `set_run_mode`).
   `ros2 run quest_arm_ik motor_calib -- hold` : 현재 위치 유지 명령 2 s(position 만) — 드라이버가 명령을 받는지(`system_state.command_accepted`, 로그), 떨림·소음이 없는지.
3. 모터 하나씩: `ros2 run quest_arm_ik motor_calib -- jog 34 0.1` (UID 34 를 +0.1 rad, 2 s, position 만). **어느 관절이 어느 방향으로** 움직였는지 → `motor_ids`(controlled_joints 순서로 모터 ID), `joint_signs`(URDF + 방향과 같으면 +1, 반대면 −1). 14개 반복. 끝에 Δ 표가 나온다(명령한 모터만 움직여야 정상 — 다른 모터도 움직이면 ID 충돌).
4. `config/sh_humanoid_v2.yaml` 의 **`upper_body_streamer` 블록 한 곳**에 `motor_ids / joint_signs / joint_offsets / extra_motor_ids` 를 적는다(launch 가 IK·시뮬 에뮬레이션에도 같은 값을 넘김). `command_mode`/`kp`/`kd` 도 삼현 확인값으로.
   `$QPY test/test_upper_body.py` (config 길이·중복·부호 검사) → `colcon build --symlink-install --packages-select quest_arm_ik` 는 config 가 심볼릭 링크라 보통 불필요.
5. 검증: 팔을 0 자세에서 조금 벗어난 임의 자세에 두고 `ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock hr:=true sim:=false viewer:=false` 를 **enable 하지 않은 채**(드라이버 READY) 띄우면 명령은 안 나가고 IK 만 돈다 —
   `ros2 topic echo /teleop/joint_targets --once` 의 position 과 `motor_calib watch` 의 (pos − offset)·sign 이 14축 모두 같은 값이어야 한다(IK 가 측정값으로 초기화되므로). 다르면 매핑 오류.

### 6-4. 하드웨어 없는 리허설 (15분) — 실측 매핑을 넣고
```bash
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock hr:=true sim:=true          # 미니PC 드라이버는 꺼 둔다(같은 토픽 이름!) 또는 ROS_DOMAIN_ID 를 다르게
ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true}"   # 에뮬레이션도 enable 전엔 명령을 무시
ros2 topic hz /upper_body/command                                                             # 50 Hz (config rate)
```
뷰어의 로봇이 mock 궤적(들어 올리기 → 원 → 쥐기)을 그대로 따라오고 `upper_body_streamer` 로그가 `state TRACK, driver RUNNING`, `cmd 50 Hz, tracking err 0.0xx rad` 이면 경로 전체가 맞다.
여기서 `joint_signs` 하나를 일부러 −1 로 바꿔 보면 실기체에서 매핑이 틀렸을 때의 증상(엉뚱한 관절이 반대로 가고 IK 재동기화 반복 → 10 s 안에 자동 정지)을 미리 볼 수 있다.

### 6-5. 실기체 — mock 으로 먼저 (1시간)
```bash
# 미니PC tty1: hr_controller (패치판, 6-1e) 실행 중
# 터미널 A (Thor): ros2 run quest_arm_ik hr_udp_relay            ← 서비스 제공용 단독 릴레이
# 터미널 B (Thor): ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true}"   # 먼저 enable (팔은 그 자리)
# 터미널 A: Ctrl-C 로 단독 릴레이 종료 → pgrep -af hr_udp_relay 가 비어야 함 (launch 가 자기 릴레이를 띄우며 7002 포트를 쓴다)
# 터미널 2 (Thor): 비상정지 담당 준비 확인 후
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock hr:=true sim:=false viewer:=false v_max:=0.2 mock_radius:=0.05
#   (단독 릴레이를 계속 쓰려면 relay:=false 를 붙인다 — v0.7.32)
# 터미널 3: ros2 run quest_arm_ik diag_teleop   /   ros2 topic hz /upper_body/command   /   ros2 run quest_arm_ik motor_calib -- watch
```
- **enable 을 launch 보다 먼저** 하는 이유: mock 의 타이머는 enable 이 아니라 IK 첫 출력 시점부터 돈다. launch 뒤 한참 있다가 enable 하면 mock 은 이미 원을 돌고 실제 팔은 서 있어 IK 가 재동기화를 반복하다(10 s 에 3회) `/teleop/hold true` 를 쏴 스트리머가 HOLD 로 들어간다(그땐 `ros2 topic pub -1 /teleop/hold std_msgs/msg/Bool "{data: false}"`).
- 순서: launch 가 뜨면 스트리머가 ENABLED 를 보고 즉시 **현재 자세 유지 명령**부터 시작(움직임 없음) → IK 가 측정 각도로 초기화 → 2 s 뒤 mock 이 그립을 쥐고 **양손을 2 s 동안 앞·위로 6 cm/15 cm 들어 올린 뒤** 지름 ~8 cm 원을 천천히 그린다(launch 후 4~5 s 뒤부터 움직임). v_max 0.2 rad/s 라 아주 느리다.
- **실측(2026-10-23, 제어기 내장 링크 + 손목 순서 수정 모델)**: `relay:=false v_max:=0.5 mock_radius:=0.08 mock_period:=3.0` → `state HOLD (no ik target), driver ENABLED` → IK `initialized from measured joint positions` → 그립 → `state TRACK`, ik/motor_state/cmd 전부 100 Hz, tracking err 0.03~0.05 rad 로 일정, 재동기화·HOLD 없음, Ctrl-C 깨끗이 종료. `mock_period:=2.0`(손 0.19 m/s) 에서는 err 0.05~0.085 rad — 드라이버 위치 루프 지연(≈0.12 s × 관절 속도) 이며 통신 문제 아님; 줄이려면 미니PC config `position_tuning`(loc_kp).
- **아무것도 안 움직이면** launch 출력에서 `[hr_udp_relay] … Address already in use` 와 스트리머의 `motor_state 0 Hz` 를 찾는다(2026-10-23 첫 시도가 이것): 릴레이가 둘이라 launch 의 릴레이가 죽은 것 — 컨테이너에서 `pgrep -af hr_udp_relay` 로 남은 것을 `pkill -f hr_udp_relay` 한 뒤 다시 launch 하거나, 그 릴레이를 두고 `relay:=false` 로 launch. 움직이지 않은 것은 안전한 실패(상태를 못 받으면 스트리머가 명령을 내지 않음).
- 봐야 할 것: 두 팔이 같은 모양으로 움직이는가(한쪽만 다르면 그쪽 매핑), 소음·떨림(kd ↑ 또는 kp ↓), `tracking err` 가 0.05 rad 안인가(크면 kp ↑ 또는 v_max ↓), 온도.
- **정지 시험 3종**을 여기서 한다: ① 터미널 2 에서 `dual_arm_ik` 만 죽이기(`pkill -f dual_arm_ik`) → 0.2 s 안에 `HOLD (ik timeout)` 로 멈추고 그 자세 유지 ② `ros2 topic pub -1 /teleop/hold std_msgs/msg/Bool "{data: true}"` → 멈춤, `false` → 그 자리에서 재추종 ③ 팔을 손으로 살짝 막아(장갑, 저게인) 명령-측정 차가 0.35 rad 를 넘으면 `HOLD (tracking error …)` → 손을 떼면 자동 복귀. 비상정지 스위치도 한 번 눌러 드라이버가 어떻게 서는지 본다.
- 문제없으면 속도를 올린다 — v_max 는 상한일 뿐이라 궤적을 빠르게 해야 차이가 난다(v0.7.33): `mock_radius:=0.08 mock_period:=3.0 v_max:=0.5`(손 0.13 m/s) → `mock_radius:=0.08 mock_period:=2.0 v_max:=1.0`(0.19 m/s). 기본 mock(r 0.05, 6 s)은 손 0.04 m/s 라 v_max 0.2 도 안 걸린다.
- 끝낼 때: launch Ctrl-C → 스트리머 명령이 끊긴다. **드라이버가 명령 끊김에 어떻게 반응하는지**(유지? 브레이크? 탈력?) 를 여기서 확인해 두어야 한다. 탈력이면 팔이 떨어지니 팔을 받칠 준비.

### 6-6. 실기체 — Quest (1시간)
```bash
# 터미널 A: ros2 launch isaac_ros_teleop ... (2-2 와 같음), Quest Connect
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=false viewer:=false v_max:=0.5 operator_arm:=0.56 grasp:=false
```
- 조작자는 로봇과 같은 방향을 보고, **팔을 옆으로 내린 채**(자연스럽게, 미리 굽힐 필요 없음) 한쪽 그립만 쥔다 → 그 팔만 움직임(다른 팔은 IK 가 현재 자세를 유지). 작은 동작(10 cm) → 들어 올리기 → 양손. 손을 놓으면 즉시 멈추는지.
- 그립을 쥔 자리가 기준이므로 큰 동작 전엔 놓고 다시 쥔다. 스케일 = 로봇 팔 0.45 m / 조작자 어깨→손바닥(`operator_arm:=`, 기본 0.60) — 사람 30 cm ≈ 로봇 22 cm.
- **버튼 (v0.7.35, config `orientation_clutch: trigger`)**: 측면 그립 = 팔(손 위치) 클러치, **앞쪽 검지 트리거 = 손목 클러치**. 트리거를 당기지 않으면 손목 3관절은 잠겨 전혀 움직이지 않고
  손 방향은 전완을 따라간다(위치만 추종). 당긴 순간의 컨트롤러 자세가 기준이 되어 그 뒤 손목 회전만 따라가고, 놓으면 손목이 그 자리에 멈춘다. 그립을 놓으면 둘 다 끝.
  이렇게 한 이유: 로봇(상완 0.19·전완 0.27 m)과 사람의 팔 비율이 달라 손 방향을 늘 맞추면 사람은 손목을 안 움직였는데 로봇 손목이 37~60° 꺾인다(IK_TUNING.md §2-2).
  Dex3 손을 쓸 때는 트리거가 손 쥐기와 겹치므로 `orientation_clutch: always` 로 돌리거나 손 쥐기를 다른 입력으로.
  dual_arm_ik 로그 `right wrist tracking (trigger)` / `right wrist locked` 로 전환을 확인한다.
- 그립만 쥐고 가만히 있어도 팔꿈치가 조금 굽던 것(v0.7.34 까지, 3 s 에 13°)은 `nominal_posture` 를 내린 팔(팔꿈치 −0.3, 손목 0)로 바꿔 없앴다(v0.7.35). 남는 2.9° 는 팔꿈치 한계 0 + `limit_margin` 0.05.
- 재동기화 경고가 가끔 뜨면 정상(놓고 다시 쥠). 반복되면 로봇이 못 따라오는 것: v_max ↓, kp ↑, 또는 충돌/한계.
- 이후 `v_max:=0.5` → `0.8`. IK 쪽 `collision_d_min` 은 실기체에서는 0.04 로(시뮬 0.03).
- **팔꿈치 힌트(v0.7.38, config `elbow_hint_cost: 0.5`)**: 손 위치만 맞추면 팔꿈치 모양은 자세 과제가 정해 사람과 다르게 접히던 것을, 그립 순간 로봇 전완 방향을 기준으로
  컨트롤러 회전 변화만큼 돌린 방향(≈ 조작자 전완 방향)을 팔꿈치 위치 과제로 줘서 **팔꿈치가 사람 전완 방향을 따라가게** 했다(모델 검증: 손을 가슴 앞에 두고 팔꿈치만 70° 들기 —
  전완 방향 오차 최대 45° → 21°, 손 오차 +4 mm). 트리거로 손목을 돌리는 동안은 끄고 놓을 때 기준을 다시 잡는다. 전제는 "팔을 내린 채 그립" 과 "컨트롤러를 전완 방향으로 자연스럽게 쥠";
  손목을 무의식적으로 꺾으면 그만큼 팔꿈치가 따라 돈다 — 거슬리면 `elbow_hint_cost: 0`. 로그 `right elbow hint on/off`.
- **지연·떨림 측정(v0.7.37)**: 텔레옵이 도는 채로 `ros2 run quest_arm_ik diag_latency`(diag 창; 기본 오른팔, 정지 3 s + 동작 20 s) → 입력률, 정지 떨림[mm], **필터 지연**(target_raw→IK 목표),
  **드라이브 지연**(IK 출력→측정), 속도 포화율(v_max 가 자르는 틱 %), 추종 오차를 표로. 튜닝 전후로 한 번씩 찍어 비교한다. 목표: 필터 ≤ 50 ms, 드라이브 ≤ 60 ms, 떨림 σ ≤ 2 mm, 포화 ≤ 5 %.
  가짜 체인 실측: One-Euro 기본(min_cutoff 1.5, beta 0.5) 80 ms → `filter_min_cutoff 4.0, filter_beta 10.0, task_gain 0.8` 30 ms. 지연 줄이는 순서는 IK_TUNING.md §8.
  **2차 측정(컨트롤러를 가슴 앞에 완전히 정지)**: 점프 0·끊김 0 인데 정지 σ 70 mm·최대 196 mm — 튐이 아니라 **매끈한 흐름**. 광학 추적을 놓쳐 IMU·팔 모델로 추정될 때의 전형(헤드셋을 쓴 사람 기준으로
  컨트롤러 위치를 지어내므로 책상 헤드셋에서는 수십 cm 흐른다). 분리 시험: ① 컨트롤러를 헤드셋 0.5 m 앞 책상에 놓고 측정(σ ≤ 5 mm 여야 함) ② 잠깐 써 보고 측정(기준값) ③ 거치 거리·각도·컨트롤러 윗면(LED)이 카메라를 보게.
  **3차 측정(v0.7.41, 추적 주파수 60 Hz 고정, 책상 거치, 튜닝 전)** — 신뢰할 수 있는 첫 기준값: 가장 조용한 2 s σ **0.36 mm**(최대 0.56; 추적은 매우 좋음, 앞의 15 cm 는 자세 잡는 이동이었음), 점프·끊김 0,
  **필터 지연 110 ms**(상관 0.92; One-Euro 1.5 Hz 이론값과 일치), **드라이브 지연 50 ms**(0.93), 속도 포화율 40 %, 추종 오차 최대 0.06 rad, 입력 50 Hz. → 다음: 필터(4~5 Hz) → v_max 1.0 → 드라이브 튜닝.
  **실기체 첫 측정(2026-10-26, 책상 헤드셋, v_max 0.5, 튜닝 전)**: 입력 50.6 Hz, 정지 떨림 σ 169 mm(!), 필터 지연 200 ms(상관 0.53 — 신뢰 낮음), 속도 포화율 46 %, 드라이브 지연 60 ms(상관 0.92),
  추종 오차 최대 0.07 rad. 정지 중 17 cm 는 손 떨림이 아니라 **추적 점프**(헤드셋이 컨트롤러를 놓쳤다 다시 잡음)이고, 그 점프를 `max_step`(틱당 3 cm) 이 따라가며 생기는 지연이 200 ms 로 잡힌 것.
  v0.7.41 부터 `입력 추적` 줄(5 cm 넘는 점프·150 ms 넘는 끊김 횟수), `가장 조용한 2 s` 의 σ(사람이 언제 멈췄든 추적 잡음만), 처음 3 s 궤적이 찍히고, 시각은 메시지 header.stamp, 필터 지연 상관은 그립을 쥔 구간만·요 90° 축 맞춤 뒤 계산한다 — 이것이 0 이 될 때까지는 헤드셋 거치(거리 0.7 m, 가슴 높이, 조명)를 먼저 고친다. 드라이브 60 ms 는 예상(120)보다 좋음; 포화 46 % 는 v_max 가 동작 절반을 자르고 있다는 뜻.
  **4차(필터 5 Hz·beta 10·task_gain 0.8, v_max 0.5)**: 필터 지연 **20 ms**(110 에서), 나머지 3차와 같음. 같은 날 v_max 1.0 은 불안정 — 드라이브 acc 2.0(어깨) 때문(6-1e).
  **5차(E 단계: acc 4/8 적용 후 v_max 0.7, 2026-10-26 저녁)** — '불안정' 재발, 그러나 원인이 드라이브가 아니다: 입력 50 Hz, **가장 조용한 2 s σ 8.6 mm·최대 34 mm**(3차 0.36 의 24배), **점프 2회(120 mm/샘플)**, **필터 지연 100 ms**(= One-Euro 1.5 Hz 기본값 — 새 zip 을 풀면서 config 가 패키지 기본값으로 돌아간 것), 드라이브 60 ms(0.88), 포화 34 %, 추종 오차 최대 0.144 → 보정 후 0.103 rad, 재동기화 1회. 12 cm 점프를 100 ms 필터·max_step 이 따라가며 팔이 '휙' 움직인 것이 불안정의 실체.
  조치 순서: ① 헤드셋 설정 → 추적 주파수 60 Hz 확인(입력률 50 Hz 는 CloudXR 스트림의 정상값으로 확인됨 — 3·6차 모두 50 Hz), 조명 밝게, 헤드셋이 컨트롤러 윗면(LED 링)을 보도록 거치(0.5~0.7 m, 가슴 높이) ② config 복구(filter_min_cutoff 5.0, filter_beta 10.0, task_gain 0.8; 튜닝 중엔 elbow_hint_cost 0) ③ 처음 3 s 는 정말 가만히 두고 다시 측정.
  **6차(5차 조치 뒤 다시 v_max 0.7, 2026-10-26)** — 추적 회복(가장 조용한 2 s σ **0.48 mm**), 필터 **20 ms** 회복, 그런데 **'심하게 덜덜'**: 드라이브 지연 130 ms(상관 0.75), 추종 오차 최대 0.201 rad. v0.7.46 `diag_latency` 의 npz(`diag_right_211857.npz`)를 열어 보니 **원인은 스트리머였다**(그림 `diag_211857_shoulder_pitch.png`): 오른쪽 어깨 pitch 에서 IK 출력(검정)은 매끈하게 멈추는데 스트리머 출력(파랑, /upper_body/command)이 그 둘레를 **±40 mrad·2 Hz 삼각파**로 진동하고 측정(빨강)은 그걸 40 ms 늦게 충실히 따라갔다(2~10 Hz 속도 RMS: IK 0.053 → 스트리머 0.096 → 측정 0.131 rad/s; 스트리머→측정 지연 40 ms·상관 0.92, 즉 드라이브는 멀쩡). 구법 스트리머는 `(목표−명령)/dt 를 v_max 로 자른 속도` 에 가속 제한을 걸어, 멈춘 목표를 향해 전속으로 달리다 지나치고(v²/2a) 반대로 전속… 을 반복하는 뱅뱅 제어였다. v_max 0.5 에선 진폭(∝ v²)이 작아 안 보였고 0.7·1.0 에서 드러난 것 — **v_max 1.0 '불안정' 도 드라이브 acc 2.0 만의 탓이 아니었다.** → **v0.7.47: 스트리머를 제동 거리 법칙으로 교체**(속도 지령 = 목표 속도 피드포워드 + sign(e)·min(√(2·a_max·|e|), |e|/dt), a_max 로 감속해 목표에 딱 멈춤). 같은 IK 출력을 다시 돌리면(replay) 진동 없이 한 번 지나쳤다(v²/2a ≤ 70 mrad @ a_max 4) 돌아온다; 회귀 시험 `test_upper_body.py::test_streamer_no_limit_cycle_after_abrupt_stop`.
  남는 것: IK 출력이 속도 한계(v_max)로 달리다 **즉시 멈추는** 모양(task_gain 0.8 의 데드비트) — 멈출 때마다 v²/2a 만큼 지나쳤다 돌아오는 건 드라이브 가속 한계(어깨 4.0)의 물리라 피할 수 없고, 거슬리면 task_gain 0.5~0.6 으로 IK 가 미리 감속하게 하거나 acc2 단계(어깨 6.0) 뒤 a_max 6.0.
  **7차(v0.7.47 스트리머 + 미니PC acc2(어깨 6)·3/5번 kp 5/4·v2.3, v_max 0.7; `diag_right_213529.npz`)** — 스트리머 출력이 IK 출력 위에 정확히 얹힘(극한 사이클 소멸, IK→측정 지연 70 ms·상관 0.89, 오차 최대 0.051 rad). 사용자 체감 '진동이 조금 줄었다'. 남은 것은 **측정에만 있는 3~5 Hz 리플**(그림 `diag_213529_shoulder_elbow.png`): 어깨 pitch/roll/yaw·팔꿈치 모두 이동 중 측정 속도가 명령 둘레로 σ 0.13~0.3 rad/s 흔들리고(위치 ±10~25 mrad), 스트리머→측정 코히어런스가 2.5 Hz 위에서 0.5 아래 — 즉 명령에 있는 성분이 증폭된 게 아니라 **드라이브/기구가 스스로 내는 진동**이다(어깨 roll 3.7 Hz 가 가장 뾰족). 3차(acc 2.0/5.0, v_max 0.5)에서는 없던 것이라 acc 를 올린 대가로 보이지만 kp 의 몫일 수도 있다 → H 스윕으로 가른다.
  **7차 뒤 할 일 = 6-1f H.**
  **8차(2026-10-28 텔레옵 현장 확정: 필터 1.5/0.5·v_max 0.4·elbow_hint 0, v0.7.55 스트리머, 100 Hz)** — "가장 안정적", 단 **중간에 한 번씩 움직임이 뚝뚝 끊김**(잠시 뒤 비슷한 자세에서는 괜찮음). 창 4 로그(3분): IK·motor_state·cmd 모두 100 Hz, 추종 오차 0.02~0.07 rad, HOLD·재동기화 없음, 스트리머 접근(catch-up) 로그 없음 — 제어 쪽은 깨끗하다. 유일한 흔적: 한 5 s 창에서 **motor_state 84 Hz**(≈0.8 s 분량의 미니PC 상태가 안 옴). 상태가 끊기는 동안은 명령도 미니PC 에서 적용되지 않다가 한꺼번에 따라가므로 '뚝 끊김' 과 맞는다 — 6-2b 에서 본 **미니PC 쪽 간헐 정지**(브리지 0.5~2.4 s 멈춤)의 짧은 형태로 보이며, 자세와 무관하게 간헐적인 것도 같다. 다만 로그만으로는 Quest 입력 끊김(헤드셋이 컨트롤러를 놓치면 IK 가 멈췄다 따라감)과 구별이 안 돼 **v0.7.57 에 끊김 감지를 넣었다**: 창 4 스트리머가 `motor_states 끊김 N ms` / `IK 목표 끊김` / `스트리머 틱 지연` 을 그 순간 WARN 으로 찍고 5 s 보고 줄에 횟수를 붙이며, 창 2 릴레이가 `상태 패킷 끊김 N ms: 패킷 n개 유실(네트워크) / 미니PC 정지(제어기 시각도 비었음) / 늦게 도착(링크 지연·Thor 릴레이 폴 지연)` 으로 **원인을 갈라** 찍고, quest_bridge 가 `Quest 입력 끊김 L/R N ms` 를 찍는다. diag_latency 요약에도 '끊김(>100 ms)' 스트림별 표가 붙는다. **다음 실행**: 창 2·4 를 보이게 두고 텔레옵, 창 5 에 `ros2 run quest_arm_ik diag_latency --ros-args -p v_max:=0.4` — 끊기는 순간 어느 창에 어떤 줄이 뜨는지(아래 8장 행)와 npz 를 보내 줄 것.

### 6-7. 손(Dex3-1)과 그 외
- 우리 쪽 손 출력은 `/teleop/{left,right}_hand/grasp`(Float32 0~1, 60 Hz) 와 `/teleop/hand_joint_targets`(JointState 14 손가락 관절 rad, 50 Hz) 로 이미 나온다. Dex3 가 HRController 15 모터와 별도(Unitree SDK2 DDS `rt/dex3/{left,right}/cmd`)라면 작은 브리지 노드가 필요 — 관절 이름·열림/쥠 각은 config `hand_grasp` 와 같다. 손 전원·DDS 도메인은 삼현과 확인.
- `command_mode: 5(IMPEDANCE)` 로 가면 중력토크를 `effort_feedforward: true` 로 실어 보낼 수 있다(IK 가 `/teleop/joint_targets.effort` 에 N·m 로 넣음). 모터 토크 단위·부호 확인 후에만.
- `duration` 은 드라이버가 무시한다(실측). 매끄러움은 송신 주기(50 Hz)와 틱당 변화량(`v_max·dt`)이 정한다.
- 튠은 재시작 없이: `ros2 param set /upper_body_streamer kp 30.0` (kp, kd, extra_kp/kd, v_max, a_max, max_tracking_error, ik_timeout, v_catchup, hold_recover_time, max_tracking_retries, command_mode, effort_feedforward 가 다음 틱부터 반영). IK 쪽 `max_joint_velocity` 는 launch `v_max:=` 로 다시 띄운다.
- 스트리머 상태는 `/upper_body/streamer_state`(String 10 Hz: `state=TRACK|reason=|driver=ENABLED|commanding=1|catchup=0|err=0.012|dist=0.004|dup=0`) 로 보인다 — `ros2 topic echo /upper_body/streamer_state`. motion_show 가 이것으로 시작·일시정지를 정한다.
- 정지 → 추종 재개(enable, hold 해제, 발행자 복구, 추종 오차 회복)는 언제나 **측정값에서 시작해 목표까지 `v_catchup`(0.3 rad/s)** 으로 간다(v0.7.55; 2026-10-28 enable 순간 1 rad 떨어진 쇼 목표로 v_max 2.5 로 튄 사고). 추종 오차 정지(`HOLD (tracking error …)`)는 명령을 **측정값으로 되돌려** 드라이브가 더 당기지 않게 하고 1 s 뒤 다시 천천히 접근하며, 목표에 닿지 못한 채 3회면 `정지 유지` — `/teleop/hold` true→false, disable→enable, 또는 목표가 명령 자리로 돌아오면(텔레옵 IK 재동기화) 해제.

### 6-8. 확인 항목별 방법 (삼현·그린큐브에 묻는 것 + 현장에서 직접 재는 것)
HRController 는 그린큐브(광운대) 측이 만든 네이티브 프로세스로 보인다(저장소의 `integration/ros2_interfaces` 빌드 산출물이 macOS·ROS 2 **Humble**(RoboStack) 에서 `SamHyun/HRController/ros2_interfaces` 경로로 빌드됨; `integration/tools/ros2_axis_scan.py` 가 미니PC 로컬 소켓 `/tmp/hr_controller_cmd.sock`(magic `HRCM`, 명령 `RESCAN_UID=13`) 로 명령을 보냄). 따라서 **미니PC 가 Humble 일 가능성**이 있고, 축 재스캔·enable 등은 미니PC 의 대시보드/IPC 로도 한다. 모터는 삼현 자체 액추에이터(OD 0x7005 run_mode) 라 **삼현 액추에이터 통신 규격서(OD 표)** 를 받으면 단위·모드·게인 범위가 그 문서에 있다 — 스마트 액추에이터 과제 자료에 있을 수 있다.

| # | 항목 | 묻는 말 (삼현/그린큐브) | 현장에서 직접 확인 | 들어가는 곳 |
|---|---|---|---|---|
| 1 | ROS 배포판·RMW·DOMAIN·IP | "미니PC OS/ROS 배포판(Humble? Jazzy?), `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, 고정 IP, `ROS_LOCALHOST_ONLY`·`ROS_AUTOMATIC_DISCOVERY_RANGE` 설정값" | 미니PC: `printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; ip -4 addr`. Thor 컨테이너에서 같은 값으로 맞춘 뒤 `ros2 topic pub /ping std_msgs/msg/String "{data: hi}"` ↔ 상대에서 `ros2 topic echo /ping`(양방향). `ROS_LOCALHOST_ONLY=1` 이면 통신 불가(해제). Humble↔Jazzy 는 Fast DDS 끼리 메시지 정의가 같으면 통신됨; 안 보이면 Thor 쪽 `export ROS_STATIC_PEERS=<미니PC IP>` 후 `ros2 daemon stop` | `env.sh` |
| 2 | `ros2_interfaces` 동일 여부 | "HRController 가 쓰는 ros2_interfaces 커밋/파일을 보내 달라" | 양쪽에서 `md5sum $(ros2 pkg prefix ros2_interfaces)/share/ros2_interfaces/msg/*.msg …/srv/*.srv` 비교, 또는 `ros2 interface show ros2_interfaces/msg/UpperBodyCommand` 출력 diff. 실제 검증은 Thor 에서 `ros2 topic echo /upper_body/system_state --once` 가 디코드되면 끝. `ros2 topic info -v /upper_body/motor_states` 로 상대 엔드포인트·QoS 확인 | Thor 쪽 체크아웃 |
| 3 | position 단위·부호·영점 | "position/velocity/effort 단위(rad, rad/s, N·m? 출력축 기준? 감속비 반영?), 영점 설정 방법, 다회전 랩 여부" | 모터 disable 상태에서 `motor_calib -- watch` 켜고 관절을 손으로 90° 들기 → 값이 ±1.57 이면 rad, ±90 이면 deg, ±0.25 면 turn. 방향은 시뮬에서 URDF + 방향을 보며(`ros2 topic pub /teleop/joint_targets` 로 +0.3) 비교 → `joint_signs`. 팔을 곧게 내린 자세에서 `zero` → 값이 0 근처면 드라이버 영점이 URDF 0 과 같음, 아니면 그 값이 `joint_offsets`. 손목 롤을 한 바퀴 돌려 ±π 에서 값이 뛰는지(랩) | `joint_signs`, `joint_offsets`, (단위가 rad 아니면 `MotorMap` 스케일 추가) |
| 4 | motor_id ↔ 관절, 15번 정체 | "모터 ID 배치표(1~15 가 어느 관절인지), 15번은 무엇인지(허리? 목? 예비?)" | `watch` 켜고 관절을 하나씩 손으로 움직여 어느 ID 가 변하는지 → `motor_ids`. 15번이 어떤 관절에도 안 묶이면 `is_connected[14]`·`is_ready` 확인; 미니PC 에서 `python3 integration/tools/ros2_axis_scan.py` 가 연결된 축 목록을 보여 줌. 없으면 `extra_motor_ids: []` | `motor_ids`, `extra_motor_ids` |
| 5 | 지원 `command_mode`, kp·kd 단위·권장값 | "command_mode 1/4/5 중 구현된 것, kp/kd 의 단위(N·m/rad, N·m·s/rad?)·허용 범위·팔 무게를 버티는 권장값, POSITION 모드의 내부 게인/속도 제한" (실측 2026-10-20: POSITION 내부 루프는 0.3 rad 스텝에 4.5 rad/s, 20 % 오버슈트, 잔차 7 mrad; 명령 주기는 50 Hz 가 상한 — 6-2b) | 저게인부터: `motor_calib -- hold --mode 1 --kp 5 --kd 0.2` → `system_state.command_accepted` 와 로그. `--mode 4`, `--mode 5` 도 각각(거부되면 message 에 뜸). `stiffness 3 --mode 5 --kp 20 --kd 0.5` 로 관절을 손으로 0.1~0.2 rad 밀면 effort/Δ = 실효 강성 → kp 와 같으면 MIT 식(τ = kp·Δq + kd·Δv + τ_ff, N·m/rad). 중력에 처지지 않는 kp 를 올려 가며 찾는다(20→30→50) | `command_mode`, `kp`, `kd`, `extra_kp/kd` |
| 6 | `duration` 의미 — **확인됨: 무시됨**(2026-10-20, `jog 52 0.3 --once --duration 2.0` 이 0.25 s 안에 도착) | "duration 을 쓰는 모드가 따로 있는지" | `motor_calib -- jog 3 0.2 --once --duration 2.0 --seconds 3` : 명령 1개만 보내고 0.25 s 마다 위치 출력 — 2 s 에 걸쳐 천천히 가면 보간, 바로 가면 무시/즉시. 보간이면 스트리머 `duration: 0.004` | `duration` |
| 7 | 명령 끊김 시 동작 (워치독) | "명령이 N ms 이상 안 오면 어떻게 되나(마지막 위치 유지 / 브레이크 / 탈력 / FAULT), N 값, 복구 절차" | `motor_calib -- watchdog --seconds 3 --watch 10` : 3 s 유지 명령 뒤 송신을 끊고 state 변화·위치 변화 시각을 찍는다. **팔을 받칠 사람 배치**, 팔은 낮은 자세. 결과에 따라 `publish_when_idle`(기본 true = 정지 중에도 명령 유지) 유지 | `publish_when_idle`, 운영 절차 |
| 8 | `run_mode`(OD 0x7005) 와 `command_mode` 관계 | "set_run_mode 로 모터 run_mode 를 바꿔야 command_mode 가 먹는지(예: command 5 는 run_mode 0 에서만?), 기본 run_mode, 바꿀 때 enable 해제 필요 여부" | `ros2 service call /upper_body/set_run_mode ros2_interfaces/srv/SetRunMode "{mode: 1, motor_id_list: []}"` → 응답 `applied_motor_ids`/`message`, `system_state.run_mode` 변화. 그 상태에서 5번 항목의 hold 를 mode 1/4/5 로 반복해 어느 조합이 움직이는지 표로 | 운영 절차(enable 순서), `command_mode` |
| 9 | `system_state.state` 전이 조건 | "ENABLED 와 RUNNING 의 차이(명령 수신 중?), READY→ENABLED 조건, FAULT 해제 방법" | `ros2 topic echo /upper_body/system_state` 를 켜 두고 enable → hold 명령 → 중단 순서로 state 가 어떻게 바뀌는지 기록. ENABLED/RUNNING 이 아닌 값에서 명령을 받는 구현이면 `require_enabled: false` | `require_enabled` |
| 10 | 비상정지·disable 동작 | "E-stop 이 전원 차단인지(팔 낙하) 브레이크인지, disable_motors 뒤 모터가 탈력하는지, 복구(재enable) 절차와 영점 유지 여부" | 팔을 낮게 두고 받칠 사람 배치 후 E-stop 1회, `disable_motors` 1회 — 떨어지는지·멈추는지·`state`/`has_fault`/`error_code` 기록. 복구 후 `zero` 값이 그대로인지(영점 유지) | 운영 절차, 6-5 끝낼 때 순서 |
| 11 | 온도·전류 한계, 연속 운전 시간 | "온도/전류 경고·차단값, 전시 중 연속 운전 가능 시간" | `watch` 의 temp/current 를 시연 1회 길이만큼 돌려 보며 기록 | 운영 체크리스트 |

## 7. (8단계) 시연 운영 체크리스트

### 7-0. 시연 최종 설정 (2026-10-26 확정, v0.7.49 패키지 기본값) — 우선순위: ① 떨림 없음(지연은 감수) ② 포즈 유사성 ③ 손 위치 정확도(중요하지 않음)
| 곳 | 항목 | 값 | 이유 |
|---|---|---|---|
| Thor yaml `dual_arm_ik` | `filter_min_cutoff` / `filter_beta` | **1.5 / 0.5** (2026-10-28 현장 확정; v0.7.49 안 2.0/1.0) | 조작자 손의 2~10 Hz 성분이 드라이브 3~4 Hz 링잉을 키운다(7차). 1.5/0.5 는 지연 ≈ 100 ms 대신 가장 차분. 더 민첩히 2.0/1.0(60~80 ms)·3.0/2.0(40 ms) |
| | `task_gain` | **0.5** | 0.8 은 속도 한계로 달리다 즉시 멈춰 스트리머가 v²/2a 지나침. 0.5 는 지수적 접근(+10 ms) |
| | `elbow_hint_cost` / `forearm_filter_alpha` | **0.0** / 0.1 (2026-10-28 현장 확정; v0.7.49 안 0.5) | 켜면 팔꿈치가 조작자 전완 방향을 따라가 포즈는 닮지만(시뮬 45° → 21°) 실기체에서 덜 안정했다. 포즈 우선이면 0.5 |
| | `orientation_clutch` / `orientation_cost` | trigger / 0.15 | 손목은 트리거를 당길 때만 — 평소 잠겨 있어 손목 떨림·꺾임 없음 |
| | `translation_scale` | launch `operator_arm:=<어깨→손바닥 m>` | 조작자 팔 길이에 맞춰야 '뻗으면 뻗고 접으면 접는' 포즈가 맞는다 (0.60 m 이면 0.75) |
| | `nominal_posture` / `posture_weights` | [0,0,0,−0.3,0,0,0] / [0.1,1,1,0.3,1.5,1.5,1.5] | 그대로 — 포즈는 힌트가 맡고 자세 과제는 바탕만 |
| launch | `v_max` | **0.4** (2026-10-28 현장 확정; config 기본도 0.4, v0.7.49 안 0.5) | 드라이브 리플은 속도와 함께 커진다(틱당 과도 1.0 rad/s 에서 2배, 0.7 에서 1.5배). 포즈 모사에 0.4~0.5 면 충분 |
| | `a_max` | 4.0 (config) | 드라이브 어깨 acc 와 같게 — 명령이 드라이브가 못 내는 가속을 요구하지 않게 |
| Thor `upper_body_streamer` | `v_catchup` | 0.3 (config, v0.7.55) | enable·hold 해제·추종 오차 회복 뒤 멀리 있는 목표로는 이 속도로 접근 — 2026-10-28 쇼 enable 순간 2.5 rad/s 로 튐 |
| Thor `upper_body_streamer` | 제동 거리 법칙 (v0.7.47+) | — | 구법은 v_max ≥ 0.7 에서 2 Hz 로 진동(6차). 필수 |
| 미니PC `hr_controller.yaml` | 드라이브 acc | `--stage acc` (어깨 4.0·나머지 8.0) | 리플·지연은 acc 와 무관(H 스윕), 6/8 은 어깨 따라잡기 과도 → 4/8 |
| | loc_kp | 드라이브 기본값 (3·5번 5.0/4.0 은 왼팔이 웅웅거리지 않으면 유지) | kp1·kp2 는 안 함 — 지연은 kp 와 무관, 처짐(≤ 20 mrad)은 시연에 안 보임 |
| | 패치 | v2.1 + v2.2 + v2.3 | 링크 내장·축별 튜닝·read-back 0.5 s 지연 |
| Quest | 추적 주파수 60 Hz, 책상 거치 0.5~0.7 m, 조명 밝게 | | 추적 σ 0.4 mm 유지 |
| 조작자 | 팔 내린 채 그립 → 천천히(관절 0.5 rad/s 안쪽) → 손목이 필요할 때만 트리거 | | 속도 한계 안에서 움직이면 지연 ≈ 필터 70 + 스트리머 30 + 드라이브 50 ≈ 150 ms 뿐 |

판정은 `diag_latency` 로: 떨림 분석 측정 2~10 Hz RMS 가 7차(0.09~0.12) 보다 작고 '측정에 2~10 Hz 떨림 없음' 에 가까워야 한다. 그래도 떨리면 (1) `v_max:=0.4` (2) 필터 1.5/0.5 (3) 삼현에 속도 루프 게인·필터(3~4 Hz 링잉) 문의 — 이 순서.

### 7-2. 동작 쇼 (텔레옵 외 연속 동작: 하트 → 양손 인사(와이퍼) → 달리기 → 손 키스, v0.7.56)
`input:=show` 로 launch 하면 quest_bridge·dual_arm_ik 대신 `motion_show` 노드가 관절 공간 안무(`quest_arm_ik/choreography.py`)를 `/teleop/joint_targets` 로 내고, 스트리머·릴레이·시뮬은 텔레옵과 똑같이 돈다. 전체 ≈ 55 s, 끝나면 팔 내린 자세를 유지(Ctrl-C 로 종료).
```bash
# 1) 시뮬 리허설 — 실기체와 같은 경로(스트리머 → HR 에뮬레이션). 모니터가 있으면 viewer:=true 로 MuJoCo 창. 이때 창 2 의 hr_udp_relay 는 **꺼 둔다**
#    (같은 /upper_body 토픽 — 에뮬레이션은 다른 발행자가 보이면 스스로 종료한다). enable 은 실기체처럼 창 3 명령으로.
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=show hr:=true sim:=true viewer:=true
# 2) 모니터 없이(맥북 ssh) 미리 보기 — ROS 없이 GIF 렌더 (안무 검사도 같이 찍힘: 속도·한계 여유·자기충돌 0)
MUJOCO_GL=egl SH_REPO=$SH $QPY src/quest_arm_ik/tools/render_show.py ~/show.gif ~/show_keys.png     # 맥북: scp thor:~/workspaces/isaac_ros-dev/show.gif ~/Downloads/
# 3) 실기체 (팔 반경 비우고, 비상정지 담당) — tmux 창 6 `show` 에 입력돼 있음. 텔레옵 launch(창 4)는 내리고, 릴레이(창 2)는 켠 채로
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=show hr:=true sim:=false viewer:=false relay:=false grasp:=false show_speed:=0.5
#    → 로그 `motion_show v0.7.56 …`(버전 ≥ 0.7.55) 와 `시작 대기: 스트리머가 명령을 내지 않음 (driver READY)` 확인 → 창 3 enable → `시작: 1 s 뒤 3.0 s 동안 시작 자세로` → 쇼.
#    괜찮으면 show_speed:=1.0 (v_max 는 show 기본 2.0: 달리기 ±60° = 1.64 rad/s). 쇼 도중 멈추려면 Ctrl-C(마지막 자세 유지) 또는 창 3 disable(→ 쇼 일시정지, enable 하면 천천히 이어감)
ros2 launch … input:=show hr:=true sim:=false relay:=false show_acts:=heart,wave show_speed:=0.7 show_loop:=true   # 일부만 / 느리게 / 반복
```
**2026-10-28 사고 ①과 보강(v0.7.54)**: `input:=show hr:=true` 만 주고 띄웠더니 launch 의 `sim` 기본값이 true 라 MuJoCo HR 에뮬레이션이 창 2 의 실기체 릴레이와 **함께** 떠서 `/upper_body/motor_states`·`system_state` 발행자가 둘이 됐다. 스트리머는 섞인 측정값(에뮬레이션의 초기 자세 vs 실기체 자세)으로 첫 명령을 내 실기체가 튀었고, 0.45 rad 추종 오차로 HOLD → 에뮬레이션의 READY 상태를 보고 명령 중단. 보강: ① launch `sim` 기본값이 hr:=true 면 false ② 스트리머가 motor_states/system_state 발행자가 둘 이상이면 명령을 내지 않음(ERROR 로그) ③ 에뮬레이션은 motor_states 발행자가 이미 있으면 켜지지 않고 종료 ④ motion_show 는 드라이버 ENABLED 를 본 뒤 그 순간의 측정 자세에서 시작(`wait_enabled`).

**2026-10-28 사고 ②와 보강(v0.7.55)**: 위 명령(sim:=false relay:=false)으로 show_speed 0.5 는 "enable 때 조금 튀고 그 뒤 괜찮음", 1.0 은 "enable 하니 너무 빠르게 튀고 하트 시작 자세까지 너무 빠르게 가며 불안정". 로그: motion_show 가 launch **10 ms 뒤** `시작: 1 s 뒤 3.0 s 동안 시작 자세로` 를 찍고 쇼 시계가 돌기 시작(v0.7.54 의 motion_show 라면 `enable 을 기다림` 을 찍으며 섰을 것 — 실행된 코드가 v0.7.53 이었다: zip 을 덮어쓴 뒤 `colcon build` 가 안 됐거나 install 이 symlink 가 아님). 6 s 뒤 enable 한 순간 쇼 목표는 이미 하트 전환 끝(어깨 roll −1.2·팔꿈치 −1.0 …, 측정에서 1 rad 넘게 떨어짐)이라 스트리머가 측정값에서 그 목표로 **v_max 2.5 rad/s** 로 달렸고, 0.71 rad 추종 오차로 HOLD — 그런데 구 HOLD 는 명령을 0.7 rad **앞에 세워 둔 채** 멈추므로 드라이브는 거기까지 전속으로 갔고, 오차가 0.35 아래로 내려오면 TRACK 으로 돌아가 다시 달리기를 반복(0.36 → 2.40 rad; 팔이 흔들리다 e-stop/READY). 0.5 에서 '조금 튄 것' 도 같은 원인(목표가 덜 멀었을 뿐).
보강: ① motion_show 는 **스트리머가 `/upper_body/streamer_state` 로 `commanding=1`(발행자 하나·드라이버 ENABLED·측정 있음)을 알릴 때까지** 아무 목표도 내지 않고 `시작 대기: …` 를 5 s 마다 찍는다(시간이 지나도 혼자 시작하지 않음; 순수 시뮬 `hr:=false` 만 즉시 시작). ② 쇼 도중 스트리머가 명령을 못 내거나(disable·발행자 2개·hold·추종 오차) 목표까지 접근 중이면 **쇼 시계를 멈추고 마지막 목표를 유지**(`쇼 일시정지: …`), 스트리머가 다시 목표에 닿으면 `쇼 재개`. ③ 스트리머는 정지에서 추종을 (다시) 시작할 때 목표까지 **`v_catchup` 0.3 rad/s** 로 접근하고 닿은 뒤에만 v_max — enable 을 언제 하든 튀지 않는다. ④ 추종 오차 정지는 명령을 **측정값으로 되돌리고**(드라이브가 더 당기지 않음) 1 s 뒤 천천히 다시 접근, 3회 연속이면 정지 유지. ⑤ 노드가 시작 로그에 버전을 찍는다(`motion_show v0.7.55`, `upper_body_streamer v0.7.55`) — 로그에 버전이 없으면 옛 코드가 돌고 있는 것: `ros2 pkg xml quest_arm_ik | grep version` 과 `colcon build --packages-select quest_arm_ik` 뒤 창마다 `source install/setup.bash`(env.sh). 회귀 시험 `test_motion_show.py::test_chain`(disable → 팔을 0.4 rad 밀어냄 → enable: 접근 0.30 rad/s, 쇼 재개), `test_upper_body.py::test_streamer_tracking_error_and_external_hold`·`test_streamer_catchup_after_restart`.

**2026-10-28 ③ 하트에서 오른팔 손목 pitch 가 '혼자' 꺾임 (v0.7.56)**: 시작은 안전해졌는데 하트 자세에서 오른팔 마지막 관절이 이상하게 꺾였다. 부호 문제가 아니다 — 6-2b 표와 6-1f D 결과(양쪽 손목 pitch 모두 **−(안쪽) 구간에서 몸통 접촉**)로 양팔 부호는 맞고, 모델 FK 로도 양팔이 거울 대칭이다. 원인은 **하트 자세의 두 손 사이가 모델에서 30 mm** 뿐이었던 것(손목 roll −0.5·pitch −0.3 으로 손끝을 모았음). 실기체는 관절마다 중력 처짐 10~20 mrad 와 영점 오차가 쌓여 손 위치가 수 cm 틀어지므로 두 손(손가락)이 서로 눌렸고, 가장 무른 손목 pitch(loc_kp 3~5) 가 밀려 꺾인 것. (손 키스의 '입 앞' 자세도 모델에서 엄지가 2 mm 겹쳐 있었다.) 조치: 하트 `both(-0.2, -1.90, 1.40, -1.60, 0, -0.30, -0.15)` 로 손 사이 **124 mm**(손끝 높이 1.40 m, 하트 모양은 팔이 만들고 손은 위 중앙에서 마주 보기만), 입 앞 자세 `both(1.02, -0.15, -0.12, -1.60)` 로 **92 mm**; `test_motion_show.py::test_self_collision` 이 손가락 geom(contype 0 이라 ncon 에 안 잡힘)까지 `mj_geomDistance` 로 재어 쇼 전 구간 손-손 ≥ 60 mm·손-반대팔 ≥ 40 mm·손-몸통(팔 든 동안) ≥ 25 mm 를 검사한다(현재 124/228/90 mm). **머리**: 모델에 머리가 없다 — 하트에서 손 링크는 z 1.31 m(몸통 윗면 +21 cm)·앞 17 cm 를 지나므로 실제 머리가 그보다 높거나 앞으로 나와 있으면 손이 스친다. 첫 실행에서 손이 머리를 스치면 머리 높이·앞뒤 치수를 알려줄 것(어깨 roll 을 −1.95 로 올리거나 어깨 pitch 를 −0.3 으로 젖혀 올린다).
동작(2026-10-27 MuJoCo 로 설계·검사: 자기충돌 0, 관절 한계 여유 ≥ 0.10 rad, 관절 속도 run 1.64·나머지 ≤ 1.0 rad/s):

| 동작 | 길이 | 내용 |
|---|---|---|
| heart | 13 s | 팔을 옆으로 들어(roll −1.85) 팔꿈치를 굽혀 손끝이 머리 위 중앙에서 만남(손 링크 간격 0.36 m), 1 Hz 로 두 번 '두근', 옆으로 내림 |
| wave | 10 s | 와이퍼: 팔 앞·위(pitch 1.4)에 전완을 세우고(팔꿈치 80°, **손목 yaw +1.0 으로 손바닥이 정면**) 팔꿈치 위치는 고정, 어깨 yaw 만 ±0.5 로 돌려 두 전완이 **같은 방향**으로 좌우 흔들림(왼팔 +·오른팔 − 가 같은 방향), 0.3 Hz × 3회 |
| run | 8 s | 팔꿈치 60° 고정, 한 팔이 앞으로 가면 반대 팔은 뒤로 — 어깨 pitch **±60°**(반대 위상), 0.25 Hz × 2회. 관절 속도 1.64 rad/s·가속 4.1 rad/s² 라 쇼의 v_max 는 2.0(스트리머 2.5, a_max 4) |
| kiss | 11 s | 양손을 '입'(몸통 위 앞 중앙, IK 로 푼 자세) 앞에 모았다가 앞·바깥으로 던지기 × 2 |

안전 장치: ① motion_show 는 드라이버 ENABLED 를 확인한 뒤 그 순간의 측정 관절각(motor_states)에서 최소 저크로 시작 자세에 들어가고(blend_time 3 s, 더 멀면 전환 속도 1.0 으로 늘림), 틱마다 v_max·dt·관절 한계로 자른다 ② 스트리머가 v_max(1.25×)·a_max 4 로 한 번 더 지키고 명령-측정 차 0.35 rad 이면 그 자리 정지 ③ 동작 사이 전환은 최대 관절 속도가 v_max 를 넘지 않는 시간으로 자동 계산. 안무를 바꾸면 `SH_REPO=$SH $QPY test/test_motion_show.py`(속도·한계·MuJoCo 접촉·가짜 ROS 체인) 를 돌린다. 모델에 머리가 없으므로 '머리 위' 는 몸통 상자(z 1.10) 위를 뜻한다 — 실기체 머리가 있으면 hands z 1.27 이 머리와 닿지 않는지 첫 실행 때 `show_speed:=0.5` 로 확인(달리기도 0.82 rad/s 로 돌아 드라이브 반응을 먼저 보는 셈).

### 7-1. 맥북만으로 운영 (모니터 없이 ssh + tmux)
전시장에서는 Thor·미니PC 에 모니터를 붙이지 않고 맥북 한 대로 둘 다 다룬다. 터미널은 몇 개든 ssh 로 열 수 있지만, **ssh 가 끊기면(맥북 절전·Wi-Fi 흔들림) 그 셸에서 띄운
launch 가 같이 죽으므로** 반드시 **tmux** 안에서 띄운다 — 세션은 머신에 남고, 다시 붙으면(`tmux attach`) 화면이 그대로다.

**네트워크**: 맥북을 Quest 공유기에 붙인다(5 GHz Wi-Fi 또는 유선). Thor 는 공유기 쪽 IP(`hostname -I`, 공유기에서 DHCP 고정 예약)로, 미니PC(192.168.10.2)는 Thor 를 거쳐(ProxyJump) 들어간다.
미니PC 에 sshd 가 있는지 **모니터가 있을 때** 확인: `systemctl is-active ssh`(inactive/없음이면 `sudo apt install openssh-server`, 인터넷 필요 → 연구실에서).

**맥북 `~/.ssh/config`** (한 번) + 키 복사로 비밀번호 없이:
```
Host thor
  HostName 192.168.50.10          # Thor 의 공유기 쪽 IP
  User sejong
  ServerAliveInterval 15
Host minipc
  HostName 192.168.10.2
  User samhyun
  ProxyJump thor
  ServerAliveInterval 15
```
```bash
ssh-copy-id thor && ssh-copy-id minipc        # 이후 ssh thor / ssh minipc
```

**Thor** (호스트에 `sudo apt install tmux`, 연구실에서 미리): `scripts/thor_demo_tmux.sh` 를 `~/` 에 두고 맥북에서 `ssh -t thor 'bash ~/thor_demo_tmux.sh'` → tmux 세션 `demo` 에 창 5개
(1 cloudxr · 2 relay · 3 enable · 4 teleop · 5 diag)가 생기고 각 창은 컨테이너 셸 + `env.sh` 상태로 명령이 **입력만** 되어 있다 → 순서대로 Enter: ① cloudxr(Quest Connect) ② relay ③ enable ④ teleop(`relay:=false` 라 relay 창은 그대로 둔다) ⑤ diag.
속도·팔 길이는 `V_MAX=0.6 OPERATOR_ARM=0.58 bash ~/thor_demo_tmux.sh` 처럼 환경변수로. 다시 붙을 땐 같은 명령(이미 있으면 attach). 창 이동 `Ctrl-b 숫자`, 떼기 `Ctrl-b d`, 스크롤은 마우스 휠, 그 창의 명령만 멈춤 `Ctrl-C`.

**미니PC**: `scripts/minipc_hr_tmux.sh` 를 `~/` 에 두고 맥북에서 `ssh -t minipc 'bash ~/minipc_hr_tmux.sh'` → 세션 `hr` 에 창 2개(1 ctrl: `./run_hr_controller.sh > ~/hr.log 2>&1` 입력만, 2 log: `tail -F ~/hr.log` 실행 중).
모터 전원을 켠 뒤 창 1 에서 Enter(전원 전에 띄우면 boot scan 0/15 → Ctrl-C 후 다시 Enter). tmux 가 없으면 `nohup ./run_hr_controller.sh > ~/hr.log 2>&1 &` + `tail -F ~/hr.log`.
CANET200 웹 UI 가 필요하면 맥북에서 `ssh -N -L 8007:192.168.0.7:80 minipc` 후 브라우저 `http://localhost:8007`(Thor 의 Firefox 불필요).

**Thor 재부팅 후**: `isaac-ros activate` 가 컨테이너를 다시 만들 수 있으므로(1-2 절) 첫 창이 뜨는 데 시간이 걸린다 — 스크립트가 창마다 2 s 간격으로 들어가지만 늦으면 `env.sh` 줄이 호스트 셸에 찍힐 수 있다. 그때는 그 창에서 `isaac-ros activate` → `source …/env.sh` 를 손으로.

- 전원·네트워크: Thor 유선, Quest 5 GHz 전용 공유기, IP 고정, Quest 충전·펌웨어. 인증서는 Thor IP 가 바뀌면 재수락.
- 실행 순서: 드라이버(또는 시뮬) → 터미널 A `ros2 launch isaac_ros_teleop ...` → Quest Connect(로봇과 같은 방향, 정지 상태) → `ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop [sim:=false]` → `engaged` 로그 확인 → 조작자 그립.
- 조작자: 로봇과 같은 방향을 보고(passthrough), **로봇처럼 팔을 옆으로 내린 채** 그립을 쥔 순간이 기준. 큰 동작 전에는 놓았다 다시 쥐어 기준 리셋. 조작 중 몸을 돌리지 않는다(돌렸으면 Disconnect/Connect). 손 쥐기는 검지 트리거(당긴 만큼).
- 이상 시: 그립 놓기(즉시 정지) → 안 되면 비상정지. 복구는 `reset_arms.sh`(RViz/ros2_control) 또는 드라이버 disable.
- 예비: 헤드셋 고장 시 `input:=mock` 로 데모 동작; 맥북 Chrome 에서 같은 클라이언트 URL 을 열면 IWER 가 Quest 3 를 에뮬레이션(점검용).

## 8. 막혔을 때
먼저 `ros2 run quest_arm_ik diag_teleop` 를 띄워 두면 1 Hz 로 노드 중복·발행자 수·IK 명령 vs 측정 팔꿈치각·엔게이지 토글·입력 수신률을 찍어 준다. launch 를 바꿀 때는 모든 터미널을 Ctrl-C 하고 `ros2 node list` 가 비어 있는지 확인(남아 있으면 `pkill -f quest_arm_ik`).

| 증상 | 확인 |
|---|---|
| 헤드셋이 Connect 안 됨 | Video Codec H.264, 인증서 수락(https://<ip>:48322/), 같은 서브넷·포트(47998/udp, 49100·48322/tcp), Quest 펌웨어, 터미널 A 가 살아 있는지 |
| `ee_poses` 가 0 Hz / (0,0,0) | 헤드셋이 대기(근접센서) 상태, 컨트롤러가 꺼짐·추적 밖, TeleopSession 로그 확인 |
| `engaged` 가 안 뜸 | 그립 버튼(측면)인지 트리거인지 — 트리거를 쓰려면 `left/right_engage_key: *_trigger_value` |
| 손이 엉뚱한 방향 | Connect 순간 머리 방향(재연결) 또는 `yaw_offset:=`; SH 는 90° 자동이 들어갔는지 로그 확인 |
| 뷰어가 안 뜸 | 호스트 `xhost +local:` + 컨테이너 `export DISPLAY=:1`; 또는 `viewer:=false` + Foxglove |
| 팔이 준비 자세로 안 감 / 시작하자마자 점프 | `/joint_states` 가 들어오는지(`init_from_joint_states`), 컨트롤러가 forward 모드인지 |
| QP infeasible 경고 반복 | 목표가 스탠드 안쪽; `collision_d_min` ↓ 또는 `collision_shapes_yaml` 치수 재확인 |
| 원을 그리다 멈추고 이상한 동작 / 관절각이 명령 범위를 벗어남 | 이전 launch 의 노드가 살아 있어 같은 토픽에 두 발행자가 있는 경우가 대부분 (`diag_teleop` 의 ★ 표시, `ros2 node list` 중복). 전부 내리고 하나만 실행. v0.5.3 부터 IK 는 내장 시뮬의 `joint_states`(frame_id=base) 만 받음 |
| 동작이 점점 밀려 팔꿈치가 과하게 접힘(−1.8 이하) / `dual_arm_ik` 에 `재동기화` 경고 반복 | 시뮬이 실시간보다 느린 것(소프트웨어 GL 뷰어). v0.5.2 부터 뷰어가 느려도 물리는 실시간을 따라가며, 그래도 `mujoco_sim` 이 `실시간보다 느림` 경고를 내면 `viewer:=false` + Foxglove |
| mock 원이 한쪽으로 치우쳐 팔꿈치가 한계(−1.75)에 붙음 | IK 가 뜨기 전에 mock 이 원을 돌고 있으면 IK 가 올라온 순간의 위상이 클러치 기준이 됨(v0.5.4 부터 mock 이 IK 출력을 기다렸다가 시작). 실제 Quest 도 같은 원리: 그립을 쥔 자리가 기준이므로 팔을 편한 위치에 두고 쥔다 |
| 팔꿈치가 과하게 접혀 보임 (SH) | `ros2 topic echo /joint_states --field position` 의 4번째·11번째(elbow)가 −1.8 아래로 안 내려가면 IK 는 정상(관절부 메시가 겹쳐 보이는 것). IK 팔꿈치 상한은 `joint_limits_yaml` (기본 −1.8, 메시 기준 −1.9 부터 전완이 어깨에 닿음) |
| mock 원의 일부만 그리고 되돌아감 / 한 방향으로 안 움직임 | 시작 자세가 작업공간 가장자리(팔이 거의 펴짐·접힘). `check_model.py` [4] 작업공간 여유가 FAIL 이면 `nominal_posture`·`initial_posture` 를 팔 길이 중간(손이 어깨 앞 절반 거리)으로 |
| `libtinyxml.so` ImportError (NVIDIA 노드 또는 우리 노드) | pinocchio 두 벌(~/.local 과 /usr/local) 충돌. `env.sh` 가 `PYTHONNOUSERSITE=1` 로 ~/.local 을 차단한다 — `pip --user` 로 설치하지 말 것 |
| 레이어 빌드 중 `ImportError: libtinyxml2.so.11` (또는 `.so.10`) | pinocchio 바이너리와 cmeel-tinyxml2 버전이 어긋난 것(pin 4 ↔ tinyxml2 10, pin 3 ↔ tinyxml2 11). v0.6.2 부터 우리 스택은 시스템 사이트가 아닌 `/opt/quest_py` venv 에 `cmeel-urdfdom<5`, `cmeel-tinyxml2<11` 캡으로 설치 — 시스템 python 의 `pin` 은 절대 올리지 말 것(NVIDIA 노드가 pin 2.7.0 에 묶여 있음) |
| 우리 노드에서 `ModuleNotFoundError: pink/mujoco` 또는 pinocchio 2.7 API 오류 | 우리 노드가 시스템 python 으로 떴다. `echo $QPY` → `/opt/quest_py/bin/python3` 인지, `ls /opt/quest_py/bin` 이 있는지(없으면 레이어가 안 빌드된 컨테이너 → `setup_container.sh`). IK·시뮬 노드를 `ros2 run` 으로 직접 띄우면 시스템 python 이 쓰이므로 launch 를 쓰거나 `$QPY -m quest_arm_ik.dual_arm_ik_node` 처럼 실행 |
| 레이어 빌드 `exit code: 127` (apt 단계) | 베이스 이미지에 `ROS_DISTRO` ENV 가 없어 `${ROS_DISTRO:?}` 검사가 실패한 것. v0.6.0 Dockerfile 은 `ARG ROS_DISTRO=jazzy` 기본값 사용 |
| 트리거를 당겨도 손가락이 안 움직임 | `ros2 topic echo /teleop/right_hand/grasp` 가 0→1 로 바뀌는지(안 바뀌면 xr_dump 로 `right_trigger_value` 키 확인) → `ros2 topic hz /teleop/hand_joint_targets`(hand_grasp 노드, 50 Hz) → mujoco_sim 로그에 `hand_joint_targets: 모델에 없는 관절` 이 있으면 손가락 관절이 없는 모델(SH 는 launch 가 `dex3_fingers:=revolute` 를 넣음; `hand:=box` 나 `grasp:=false` 면 없음, G1 은 `$G1H`) |
| 시작하자마자 팔이 위로 올라감 (mock) | 정상 — mock 은 팔을 내린 시작 자세에서 2초간 손을 들어 올린 뒤 원을 그린다 |
| 실행 직후 입력이 없는데 팔(주로 팔꿈치)이 조금 움직임 | v0.7.3 까지: IK 가 뜨자마자 영공간에서 `nominal_posture`(teleop_ready) 쪽으로 자세를 정렬하던 것(5 s 에 팔꿈치 ~12°). v0.7.4 부터 **첫 그립 전에는 IK 를 돌리지 않고 시작 자세를 그대로 낸다**(`dual_arm_ik.freeze_until_engaged: true`, 실기체에서도 enable 직후 저절로 움직이지 않게). 그립을 놓은 뒤 손은 고정된 채 팔꿈치가 아주 조금 자리를 잡는 것은 정상 |
| (Quest) 팔을 높이 들수록 로봇이 사람과 다르게 움직임 — 팔이 먼저 다 펴지며 손목이 꺾이거나(스케일 큼), 팔꿈치가 굽은 채 남음(스케일 작음) | 스케일 = 로봇 팔 / 조작자 팔. `operator_arm:=<어깨→손바닥 m>` 로 띄우거나 `ros2 param set /dual_arm_ik operator_arm_length 0.58`. 그립은 반드시 팔을 내린 자세에서 쥔다(기준점). v0.7.3 에서 SH 기본 0.85→0.75, 시작 팔꿈치 −0.35→−0.15 |
| (mock, SH) 들어 올린 뒤 팔꿈치 아래 관절이 돌아가고 손목이 양팔 다 같은 쪽으로 꺾임 | v0.7.1 까지의 mock 버그: '손끝 들어 올리기' 피치를 항상 ROS y 축으로 돌려서 정면이 +y 인 SH 에서는 롤이 됐다(손목 요/피치가 ±1 rad). v0.7.2 에서 정면 축 기준으로 수정 — 실제 Quest 경로(xr_teleop)는 요 오프셋이 자세까지 같이 돌리므로 영향 없음 |
| `source $ISAAC_ROS_WS/src/quest_arm_ik/scripts/env.sh` → `No such file or directory` (재부팅 후마다) | 호스트 `~/.bashrc` 가 컨테이너에 마운트되어 호스트용 `ISAAC_ROS_WS` 가 컨테이너 값을 덮어쓴 것. `source /workspaces/isaac_ros-dev/src/quest_arm_ik/scripts/env.sh` 처럼 **절대 경로**로 source(v0.7.1 env.sh 가 이후 `ISAAC_ROS_WS` 를 바로잡음). 근본 해결은 호스트 `.bashrc` 의 `export ISAAC_ROS_WS=…` 줄을 컨테이너 안(`/workspaces/isaac_ros-dev` 가 있을 때)에서는 건너뛰게 조건문으로 감싸는 것 |
| `ros2 run quest_arm_ik xr_dump` (또는 motor_calib, hand_grasp) → `No executable found` | setup.py 에 새로 추가된 콘솔 스크립트는 `colcon build --symlink-install --packages-select quest_arm_ik` 후에 생긴다; 급하면 `python3 -m quest_arm_ik.xr_dump` |
| (hr) 스트리머가 `driver READY — 명령을 내지 않음` 만 반복 | 드라이버 `system_state.state` 가 4/5 가 아님: enable_motors 를 했는지, 드라이버가 enable 뒤에도 READY 를 유지하는 구현이면 config `require_enabled: false` |
| (hr) `motor_state 에 motor_ids … 가 다 없음` | config `motor_ids` 에 적은 ID 가 드라이버 메시지에 없음 — `motor_calib watch` 의 ID 목록으로 수정 |
| (hr) 명령은 50 Hz 로 나가는데 로봇이 안 움직임 | 드라이버 run_mode/command_mode 조합, kp 가 0 또는 너무 작음, 모터 미enable(`is_enabled`), `command_accepted` false |
| (hr) 한 관절이 반대로/엉뚱한 관절이 움직이고 IK `재동기화` 반복 → `/teleop/hold` 자동 | `joint_signs`/`motor_ids` 오류. jog 로 다시 확인. 해제는 `ros2 topic pub -1 /teleop/hold std_msgs/msg/Bool "{data: false}"` |
| (hr, udp) 릴레이가 `미니PC 상태 아직 없음` / `… s 이상 안 옴` | 미니PC 에 `hr_udp_bridge.py` 가 떠 있는지, 그 로그의 `peer=` 가 Thor 주소인지(HELLO 가 가야 함 — 포트 7001/7002, 방화벽), `ip -br link` 로 링크 |
| (hr, udp) 브리지 로그에 `ipc … drop N` 또는 `IPC 소켓이 가득 참` | `net.unix.max_dgram_qlen` 을 512 로(/etc/sysctl.d/99-hr.conf). hr_controller 메인 루프가 멈췄다면 `~/hr.log` |
| (hr, udp) 브리지 로그에 `상태 파일 없음` / `IPC 전송 실패` | hr_controller 가 안 떠 있거나 경로가 다름 (`--state-file`, `--ipc`) |
| (hr) hr_controller 콘솔에 `[HR] event: command_commit_failed` 가 계속 찍힘 | 미연결 축(머리 90)에 명령이 가고 있음 — v0.7.19 미만이거나 `extra_motor_ids` 에 90 이 있음. `extra_motor_ids: [0]`, `empty_slot_id: 255`; motor_calib 는 `--include-not-ready` 를 주지 않았는지 |
| (hr) launch 직후 팔이 엉뚱한 자세로 튀고 스트리머가 `HOLD (tracking error …)` → `driver READY` | 측정 발행자가 둘: launch 의 MuJoCo HR 에뮬레이션(`sim` 이 켜짐)과 실기체 릴레이가 같이 떠 있음. v0.7.54 부터 스트리머가 `motor_states 발행자 2개` ERROR 를 내고 명령을 멈추며, 에뮬레이션은 스스로 종료. 실기체는 `hr:=true sim:=false relay:=false`(창 2 릴레이) — `ros2 topic info -v /upper_body/motor_states` 로 발행자 수 확인 |
| (hr) verify/텔레옵에서 움직임이 '튐' — 멈췄다가 한 번에 뛰고, 램프 끝에서 수십~수백 mrad 모자람 | 미니PC 브리지가 명령 처리를 0.5~2.5 s 씩 멈추는 것(2026-10-20 실측; 200 Hz 에서 항상, 50 Hz 에서도 간헐). 6-1c: verify 중 hr_controller 콘솔의 `[HR-WARN] ethernet anomaly`/`TCP send error`, 미니PC `ss -ti dst 192.168.0.7` 의 retrans 증가 → CANET200 TCP 링크를 허브에서 분리(직결). `max_dgram_qlen` 10 → 512. 삼현에 MoveToPos 로그 제거·브리지 정리 요청. 속도 피드포워드·duration·kp·송신 주기(50 상한)·콘솔 전환은 원인이 아님(실측) |
| (hr) 텔레옵 중 간헐적으로 움직임이 '뚝뚝' 끊김 (자세와 무관) | v0.7.57 부터 끊기는 순간 로그가 가른다: 창 2 `상태 패킷 끊김 … 미니PC 정지` → 미니PC 제어기/CAN 부하(명령 주기 `rate: 50` 으로 낮춰 비교, 미니PC CPU·hr_controller 로그 확인); `… 패킷 유실` → 이더넷/스위치; `… 늦게 도착` + `릴레이 폴 지연` → Thor 쪽 부하(CloudXR 과 같은 호스트 — `top`); 창 4 `IK 목표 끊김` 만 → dual_arm_ik(QP) 지연; quest_bridge `Quest 입력 끊김` → 헤드셋 추적(조명·거치 거리, 6-6 5차). 6-6 8차 참고 |
| (hr) `HOLD (tracking error …)` 가 자주 뜸 | kp 가 낮아 중력에 처짐(kp ↑ 또는 IMPEDANCE + effort_feedforward), v_max 가 모터 능력보다 큼, 충돌. `max_tracking_error` 를 올리는 것은 마지막. v0.7.55 부터는 정지 때 명령을 측정값으로 되돌리고 1 s 뒤 `v_catchup` 으로 다시 접근, 3회면 `정지 유지`(hold 토글·disable→enable 로 해제) |
| (show) enable 하는 순간 팔이 빠르게 튀고 하트 자세까지 너무 빨리 감 | v0.7.53 motion_show: 쇼 시계가 launch 직후부터 돌아 enable 때 목표가 멀리 있음(2026-10-28 사고 ②). 로그 첫 줄에 `motion_show v0.7.55` 가 없으면 옛 코드 — zip 덮어쓴 뒤 `colcon build --packages-select quest_arm_ik` + 창마다 env.sh 다시. v0.7.55 는 enable 전엔 `시작 대기: …` 만 찍고, 재개는 `v_catchup` 0.3 rad/s |
| (show) 하트·손 키스에서 손목(마지막 관절)이 혼자 꺾임 | 두 손이 서로(또는 머리와) 닿아 가장 무른 손목 pitch 가 밀린 것 — v0.7.56 하트·입 앞 자세를 손 사이 ≥ 90 mm 로 넓힘. 그래도 닿으면 실기체 영점(6-2b `zero`)·중력 처짐을 의심하고 머리 치수를 알려줄 것. 부호는 아니다(6-2b 표·6-1f D: 양쪽 손목 pitch 모두 − 구간에서 몸통 접촉) |
| (show) 쇼가 시작하지 않고 `시작 대기: 스트리머가 명령을 내지 않음 (driver READY)` 만 5 s 마다 | 정상 — 창 3 에서 enable. `… 발행자가 둘` 이면 sim/릴레이 중복(위 행), `스트리머 상태(/upper_body/streamer_state)가 아직 안 옴` 이 계속되면 스트리머가 옛 버전(v0.7.54 이하)이거나 안 떠 있음 |
| (show) `쇼 일시정지: …` 뒤 멈춰 있음 | 사유가 로그에 있다: driver READY(disable/e-stop → enable 하면 천천히 이어감), 추종 오차 정지(팔이 걸렸는지 확인; 3회면 `정지 유지` → `ros2 topic pub -1 /teleop/hold std_msgs/msg/Bool "{data: true}"` 뒤 `false`), hold 토픽 ON, 접근 중(기다림) |
| (hr) 미니PC 와 토픽이 안 보임 | `ROS_DOMAIN_ID`·RMW 일치, 방화벽(UDP 7400~), 멀티캐스트 차단 시 `ROS_STATIC_PEERS=<IP>`, `ros2 daemon stop`; 보이는데 echo 안 되면 메시지 정의 불일치(`ros2 topic info -v`) |
| Dex3 zip 을 풀고 빌드했는데 뷰어의 손이 여전히 박스 | 시뮬/IK 가 **다른 xacro** 를 읽은 것. v0.7.1 부터 두 노드가 시작 로그에 `model=<경로> … hand=dex3 또는 box` 를 찍으니 그 줄부터 본다. 확인: `P=$(ros2 pkg prefix SH_Humanoid_v2_description)/share/SH_Humanoid_v2_description/urdf; ls -la $P` (dex3_hand.xacro 가 있고 항목이 src 로 가는 심볼릭 링크인지), `grep -c dex3 $P/SH_Humanoid_v2.xacro` (0 이면 옛 xacro 가 설치된 것), `grep -c dex3 /tmp/quest_arm_ik/SH_Humanoid_v2.sim.prepared.urdf` (시뮬이 마지막에 실제로 읽은 모델). 원인은 (a) zip 을 `src/SH-Humanoid/` 가 아닌 곳에 푼 것(`find /workspaces -name dex3_hand.xacro`), (b) 워크스페이스 루트가 아닌 곳에서 colcon 을 돌려 `install/` 이 갱신되지 않은 것, (c) 예전 `colcon build`(심볼릭 링크 없이) 의 복사본이 남은 것 — `rm -rf build/SH_Humanoid_v2_description install/SH_Humanoid_v2_description` 후 루트에서 `--symlink-install` 재빌드. 우회: `model_path:=$SH/urdf_moveit/SH_Humanoid_v2_description/urdf/SH_Humanoid_v2.xacro package_dirs:=$SH/urdf_moveit` |
| `xacro.XacroException: No such file or directory: …/install/SH_Humanoid_v2_description/share/…/urdf/dex3_hand.xacro` | `install/` 에 Dex3 파일이 없음 — zip 을 푼 뒤 **워크스페이스 루트에서** `colcon build --symlink-install --packages-select SH_Humanoid_v2_description` 을 안 했거나 다른 디렉터리에서 돌린 것(`src/SH-Humanoid/{build,install,log}` 가 생겼으면 삭제). 급하면 `model_path:=$SH/.../SH_Humanoid_v2.xacro package_dirs:=$SH/urdf_moveit` 로 소스 트리 직접 사용 |
| 재부팅 후 `Package 'isaac_ros_teleop' not found` / import 오류 / `pip: command not found` | 컨테이너가 새로 만들어진 것. 1-1 의 이미지 레이어가 등록되어 있으면 생기지 않음; 임시로는 `bash $ISAAC_ROS_WS/src/quest_arm_ik/scripts/setup_container.sh` |

## 부록 A. 경로 A (팀 Unity 앱, SHPOSE UDP 15000) 로 대체할 때
Windows PC + Quest Link 에서 `meta_quest/metaquest_unity` 씬 `MetaQuestTest` 의 PoseLogger `Ros2 Computer Address` = Thor IP 로 Play (또는 APK).
Thor 에서 `python3 -c "import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',15000));print(s.recvfrom(1024)[0])"` 로 패킷 확인 후
`ros2 launch quest_arm_ik teleop.launch.py robot:=sh` (기본 `input:=shpose_udp`, 요 오프셋 0 — Unity 가 base_link 축으로 보냄). 축 확인은 `/teleop/right_hand/target_raw` 에서 앞 y↑·위 z↑·오른쪽 x↑, 틀리면 Unity `RobotBaseLinkCoordinates.cs` 부호 수정.
`mapping_mode: absolute`(SHABS, 절대 포즈)로 쓸 때 두 가지를 맞춰야 한다 — ① 팀 `quest_ik.yaml` 의 NEUTRAL 손 위치는 구 모델 기준이라 현재 모델(Dex3·손목 순서 수정)의 teleop_ready 손 위치와 **23 mm** 차이(그립 순간 그만큼 이동), ② 패킷 회전이 그대로 손 프레임 회전 목표가 되므로 Unity 중립 회전 = 모델 손 프레임 회전(teleop_ready 에서 x축 68°)이어야 한다 — 단위 회전을 보내면 손목 한계에 걸려 위치가 2.7 cm 밀린다(`test/test_sh_chain.py` 참고). 경로 B(relative)에는 해당 없음.

