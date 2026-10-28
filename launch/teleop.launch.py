"""Quest → quest_bridge → dual_arm_ik (→ mujoco_sim) 실행.

삼현 2세대 (SH-Humanoid 저장소가 워크스페이스에 있을 때; 모델·SRDF·메시 경로 자동):
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh                       # Unity SHPOSE UDP 15000 → MuJoCo
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=mock           # 헤드셋 없이
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=team_topics    # 팀 udp_pose_bridge 토픽 사용
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh sim:=false            # 팀 demo.launch.py(RViz/ros2_control) 또는 실기체로 출력
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop      # Isaac ROS isaac_ros_teleop (/xr_teleop/*) — 요 90° 자동
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh hand:=box             # 기존 박스 손 모델 (기본은 G1 Dex3-1 3지 핸드)
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh grasp:=false          # 손 쥐기(트리거) 끄기 — 손가락 관절 고정
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=true    # 실기체 경로 리허설 (시뮬이 HRController 흉내)
  ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=false   # 실기체 (삼현 미니PC HRController)
G1 (MuJoCo Menagerie):
  ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=/abs/g1.xml input:=mock
  ros2 launch quest_arm_ik teleop.launch.py robot:=g1 model_path:=/abs/g1.xml input:=xr_teleop   # Isaac ROS /xr_teleop/*
xr_teleop 공통: engage:=grip(기본)|always, yaw_offset:=<deg> (기본 = forward_axis 가 ROS x 축에서 돌아간 각: g1 0, sh 90)
스케일: operator_arm:=0.58 (조작자 어깨→손바닥 [m] → translation_scale = 로봇 팔 길이 / 이 값) 또는 scale:=0.7 (직접). 실행 중 ros2 param set /dual_arm_ik operator_arm_length 0.58
인터프리터: 환경변수 QPY (scripts/env.sh 가 /opt/quest_py/bin/python3 로 설정) 가 있으면 우리 노드는 그 python 으로 실행된다.
임의 모델: robot:=custom config:=/abs/my.yaml model_path:=/abs/robot.urdf package_dirs:=/abs/dir srdf_path:=/abs/x.srdf
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

INPUT_MODES = ("xr_teleop", "shpose_udp", "shabs_udp", "team_topics", "pose_joy", "mock", "show")   # show = 안무 재생(motion_show, IK 대신)
QPY_DEFAULT = "/opt/quest_py/bin/python3"   # docker/Dockerfile.zz_quest_teleop 가 만드는 quest_arm_ik 전용 venv


def _python_prefix():
    """우리 노드를 실행할 인터프리터. Thor 이미지의 시스템 python 은 NVIDIA isaac_ros_teleop 용 pinocchio 2.7/numpy 1.26 이라
    우리 스택(pin 3.x, pink, mujoco)은 /opt/quest_py venv 에 따로 있다. 환경변수 QPY(env.sh) 또는 기본 경로가 있으면 그걸로 실행,
    없으면(맥북 등 한 사이트에 전부 있는 환경) 콘솔 스크립트의 python3 그대로."""
    py = os.environ.get("QPY", "")
    if py in ("", "python3") and os.path.exists(QPY_DEFAULT):
        py = QPY_DEFAULT
    return py if py and py != "python3" and os.path.exists(py) else None


def _sh_defaults():
    """SH-Humanoid 저장소의 description/moveit_config 패키지가 빌드·소싱되어 있으면 경로를 찾는다."""
    try:
        desc = get_package_share_directory("SH_Humanoid_v2_description")
    except Exception:  # noqa: BLE001
        return {}
    out = {"model_path": os.path.join(desc, "urdf", "SH_Humanoid_v2.xacro"),
           "package_dirs": os.path.dirname(desc), "srdf_path": ""}
    try:
        mc = get_package_share_directory("SH_Humanoid_v2_moveit_config")
        out["srdf_path"] = os.path.join(mc, "config", "SH_Humanoid_v2.srdf")
    except Exception:  # noqa: BLE001
        pass
    return out


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("quest_arm_ik")
    lc = lambda k: context.launch_configurations.get(k, "")  # noqa: E731
    robot = lc("robot")
    inp = lc("input")
    if inp not in INPUT_MODES:
        raise RuntimeError(f"input 은 {INPUT_MODES} 중 하나")

    config = lc("config") or os.path.join(share, "config",
                                           {"sh": "sh_humanoid_v2.yaml", "g1": "dual_arm_ik.yaml"}.get(robot, "dual_arm_ik.yaml"))
    model_path, package_dirs, srdf_path = lc("model_path"), lc("package_dirs"), lc("srdf_path")
    if robot == "sh":
        d = _sh_defaults()
        model_path = model_path or d.get("model_path", "")
        package_dirs = package_dirs or d.get("package_dirs", "")
        srdf_path = srdf_path or d.get("srdf_path", "")
        if not model_path:
            raise RuntimeError("robot:=sh 인데 SH_Humanoid_v2_description 패키지를 찾지 못함 — "
                               "SH-Humanoid/urdf_moveit 를 워크스페이스에 빌드·소싱하거나 model_path:= 를 직접 지정")
    if not model_path:
        raise RuntimeError("model_path:= 가 필요함 (.urdf / .xacro / .xml)")
    pkg_list = [d for d in package_dirs.split(":") if d] or [""]
    use_sim_time = lc("use_sim_time") in ("true", "True", "1")
    viewer = lc("viewer") in ("true", "True", "1")
    hr_flag = lc("hr") in ("true", "True", "1")
    # sim 기본값: hr:=true(실기체 경로)면 꺼짐, 아니면 켜짐. 2026-10-28 사고: `input:=show hr:=true` 에 sim 이 기본 true 라 MuJoCo HR 에뮬레이션이
    # 실기체 릴레이와 함께 떠서 /upper_body/motor_states·system_state 발행자가 둘이 됐고, 스트리머가 섞인 측정값으로 첫 명령을 내 실기체가 튀었다.
    sim = (lc("sim") or ("false" if hr_flag else "true")) in ("true", "True", "1")
    grasp = lc("grasp") in ("true", "True", "1")
    fwd = [float(v) for v in lc("forward_axis").split(",")] if lc("forward_axis") else ([0.0, 1.0, 0.0] if robot == "sh" else [1.0, 0.0, 0.0])

    bridge_params = {"use_sim_time": use_sim_time}
    ik_extra = {}
    sim_extra = {}
    if lc("xacro_args"):
        ik_extra["xacro_args"] = sim_extra["xacro_args"] = lc("xacro_args")
    elif robot == "sh":
        # 삼현 모델의 손 선택 (SH_Humanoid_v2.xacro 의 hand 인자): dex3 = G1 Dex3-1 3지 핸드(전시 사양, xacro 기본), box = 기존 박스 손.
        # grasp 가 켜져 있으면 Dex3 손가락 관절을 활성(revolute)으로 — mujoco_sim 이 /teleop/hand_joint_targets 로 손가락을 움직인다.
        parts = ([f"hand:={lc('hand')}"] if lc("hand") else []) + (["dex3_fingers:=revolute"] if grasp and lc("hand") != "box" else [])
        if parts:
            ik_extra["xacro_args"] = sim_extra["xacro_args"] = " ".join(parts)
    if inp == "mock":
        bridge_params.update({"input_mode": "pose_joy", "input_convention": "ros", "engage_mode": "grip",
                              "left_grasp_axis": 0, "right_grasp_axis": 1})     # quest_mock 의 Joy.axes[0,1] = 쥐기 패턴
    else:
        bridge_params["input_mode"] = inp
        if lc("input_convention"):
            bridge_params["input_convention"] = lc("input_convention")
    if inp == "xr_teleop":
        # Isaac ROS /xr_teleop/ee_poses 는 ROS 축(x 정면)·그립 값은 controller_data → 클러치 grip, 요 오프셋은 forward_axis 로 자동
        bridge_params.update({"input_convention": "ros", "engage_mode": lc("engage") or "grip"})
        yaw = lc("yaw_offset")
        ik_extra["quest_yaw_offset_deg"] = float(yaw) if yaw else math.degrees(math.atan2(fwd[1], fwd[0]))
    elif lc("engage"):
        bridge_params["engage_mode"] = lc("engage")
    if lc("yaw_offset") and inp != "xr_teleop":
        ik_extra["quest_yaw_offset_deg"] = float(lc("yaw_offset"))
    if lc("operator_arm"):
        ik_extra["operator_arm_length"] = float(lc("operator_arm"))        # 조작자 어깨→손바닥 [m] → scale = 로봇 팔 / 조작자 팔
    elif lc("scale"):
        ik_extra["translation_scale"] = float(lc("scale"))
    hr = lc("hr") in ("true", "True", "1")
    if hr:
        # 실기체(삼현 HRController) 경로: IK 는 관절 목표(/teleop/joint_targets)만 내고 upper_body_streamer 가 250 Hz 로 UpperBodyCommand 를 보낸다.
        # 측정값은 /upper_body/motor_state. sim:=true 이면 mujoco_sim 이 HRController 를 흉내 내어(hr_emulation) 하드웨어 없이 같은 경로를 리허설한다.
        ik_extra.update({"output_mode": "joint_state", "joint_states_source": "upper_motor_state", "init_from_joint_states": True})
        # 모터 매핑(motor_ids/joint_signs/joint_offsets/extra_motor_ids)은 config 의 upper_body_streamer 블록 하나만 고치면 되도록
        # 여기서 읽어 dual_arm_ik(측정값 변환) 와 mujoco_sim(HR 에뮬레이션) 에도 같은 값을 넘긴다.
        m = {}
        try:
            import yaml
            m = yaml.safe_load(open(config))["upper_body_streamer"]["ros__parameters"]
            ik_extra.update({k: m[k] for k in ("motor_ids", "joint_signs", "joint_offsets") if k in m})
            sim_extra.update({"hr_" + k: m[k] for k in ("motor_ids", "joint_signs", "joint_offsets") if k in m})   # hr_extra_motor_ids 는 config 의 mujoco_sim 값(실기체처럼 90 을 not-ready 로 보고)
        except Exception:  # noqa: BLE001
            pass
        # 속도 상한 하나(v_max)로 IK 와 스트리머를 함께 맞춘다: IK max_joint_velocity = v_max, 스트리머 v_max = 1.25·v_max (뒤에 선 안전장치).
        # IK 가 스트리머보다 빠르면 실기체가 뒤처져 IK 가 재동기화(클러치 해제)를 반복하므로, IK 쪽을 느리게 두는 것이 원칙.
        v_ik = float(lc("v_max")) if lc("v_max") else (2.0 if inp == "show" else float(m.get("v_max", 0.5)))   # 쇼는 안무 설계 상한 2.0 (스트리머 2.5)
        ik_extra["max_joint_velocity"] = v_ik
        streamer_extra = {"v_max": 1.25 * v_ik}
        if lc("a_max"):                                   # 스트리머 가속 상한 — 드라이브 acc_set_pp 이하로 (런북 6-1f)
            streamer_extra["a_max"] = float(lc("a_max"))
        if sim:
            sim_extra["hr_emulation"] = True
    elif sim:
        # 내장 시뮬의 joint_states 만 받도록 (같은 관절 이름을 쓰는 다른 시뮬/launch 가 떠 있어도 섞이지 않게)
        try:
            import yaml
            ik_extra["joint_states_frame_id"] = str(yaml.safe_load(open(config))["dual_arm_ik"]["ros__parameters"].get("base_frame", ""))
        except Exception:  # noqa: BLE001
            pass

    py = _python_prefix()                      # 콘솔 스크립트를 이 인터프리터로 실행 (None 이면 스크립트의 #! 그대로)
    if inp == "show":
        # 동작 쇼: quest_bridge·dual_arm_ik 대신 motion_show 가 /teleop/joint_targets 를 낸다 (스트리머·시뮬·릴레이는 그대로)
        show_params = {"use_sim_time": use_sim_time, "config": config,
                       "v_max": float(lc("v_max")) if lc("v_max") else 2.0,          # choreography.SHOW_V_MAX (run ±60° = 1.64 rad/s)
                       **({"acts": lc("show_acts")} if lc("show_acts") else {}),
                       **({"speed": float(lc("show_speed"))} if lc("show_speed") else {}),
                       **({"loop": lc("show_loop").lower() in ("true", "1", "yes")} if lc("show_loop") else {})}
        nodes = [Node(package="quest_arm_ik", executable="motion_show", name="motion_show", output="screen", prefix=py, parameters=[show_params])]
    else:
        nodes = [
            Node(package="quest_arm_ik", executable="quest_bridge", name="quest_bridge", output="screen", prefix=py,
                 parameters=[config, bridge_params]),
            Node(package="quest_arm_ik", executable="dual_arm_ik", name="dual_arm_ik", output="screen", prefix=py,
                 parameters=[config, {"model_path": model_path, "package_dirs": pkg_list, "srdf_path": srdf_path,
                                      "use_sim_time": use_sim_time, **ik_extra}]),
        ]
    if sim:
        nodes.append(Node(package="quest_arm_ik", executable="mujoco_sim", name="mujoco_sim", output="screen", prefix=py,
                          parameters=[config, {"model_path": lc("sim_model_path") or model_path, "package_dirs": pkg_list,
                                               "srdf_path": srdf_path, "viewer": viewer, "use_sim_time": use_sim_time,
                                               "forward_axis": fwd, "viewer_view": lc("viewer_view") or "behind", **sim_extra}]))
    if grasp:
        nodes.append(Node(package="quest_arm_ik", executable="hand_grasp", name="hand_grasp", output="screen", prefix=py,
                          parameters=[config, {"use_sim_time": use_sim_time}]))
    if hr:
        nodes.append(Node(package="quest_arm_ik", executable="upper_body_streamer", name="upper_body_streamer", output="screen", prefix=py,
                          parameters=[config, {"use_sim_time": use_sim_time, **streamer_extra}]))
        # 실기체 링크: udp(기본) = Thor 의 hr_udp_relay ↔ 미니PC 의 hr_udp_bridge.py (DDS 를 링크에 태우지 않음, 2026-10-21 Fast DDS 블로킹 대응)
        #            dds = 삼현 ros2_state_bridge.py 와 직접 DDS. sim:=true 면 mujoco_sim 이 토픽을 내므로 릴레이 불필요.
        hr_link = (lc("hr_link") or os.environ.get("HR_LINK", "udp")).lower()
        want_relay = (lc("relay") or "true").lower() in ("true", "1", "yes")
        if hr_link == "udp" and not sim and want_relay:
            nodes.append(Node(package="quest_arm_ik", executable="hr_udp_relay", name="hr_udp_relay", output="screen", prefix=py,
                              parameters=[config, {"use_sim_time": use_sim_time, **({"peer_host": lc("peer_host")} if lc("peer_host") else {})}]))
    if inp == "mock":
        try:
            import yaml
            ready_topic = str(yaml.safe_load(open(config))["dual_arm_ik"]["ros__parameters"].get("joint_state_out_topic", "/quest_ik/joint_states"))
        except Exception:  # noqa: BLE001
            ready_topic = "/quest_ik/joint_states"
        nodes.append(Node(package="quest_arm_ik", executable="quest_mock", name="quest_mock", output="screen", prefix=py,
                          parameters=[{"use_sim_time": use_sim_time, "forward_axis": fwd, "wait_for_topic": ready_topic,
                                       **({"radius": float(lc("mock_radius"))} if lc("mock_radius") else {}),
                                       **({"period": float(lc("mock_period"))} if lc("mock_period") else {}),
                                       **({"lift_time": float(lc("mock_lift_time"))} if lc("mock_lift_time") else {})}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="sh", description="sh (삼현 2세대) | g1 | custom"),
        DeclareLaunchArgument("model_path", default_value="", description=".urdf / .xacro / MJCF .xml (sh 는 자동)"),
        DeclareLaunchArgument("package_dirs", default_value="", description="URDF package:// 해석 디렉터리 (':' 구분, sh 는 자동)"),
        DeclareLaunchArgument("srdf_path", default_value="", description="MoveIt SRDF (sh 는 자동)"),
        DeclareLaunchArgument("hand", default_value="", description="sh 모델의 손: dex3 (G1 3지 핸드, xacro 기본) | box (기존 박스 손)"),
        DeclareLaunchArgument("grasp", default_value="true", description="컨트롤러 트리거(앞쪽 버튼) → 손 쥐기 (hand_grasp 노드, 손가락 관절 활성)"),
        DeclareLaunchArgument("hr", default_value="false", description="실기체 HRController 경로: upper_body_streamer(250 Hz) + motor_state 피드백. sim:=true 면 시뮬이 HRController 를 흉내"),
        DeclareLaunchArgument("hr_link", default_value="", description="hr: udp(기본, hr_udp_relay ↔ 미니PC hr_udp_bridge.py) | dds(삼현 ros2_state_bridge 직결). 환경변수 HR_LINK 로도"),
        DeclareLaunchArgument("peer_host", default_value="", description="hr_link=udp: 미니PC 주소 (기본 config hr_udp_relay.peer_host = 192.168.10.2)"),
        DeclareLaunchArgument("relay", default_value="true", description="hr_link=udp: launch 가 hr_udp_relay 를 같이 띄움. 단독 `ros2 run quest_arm_ik hr_udp_relay` 를 그대로 쓰려면 false (둘 다 띄우면 포트 7002 충돌)"),
        DeclareLaunchArgument("v_max", default_value="", description="hr: 관절 속도 상한 [rad/s] — IK max_joint_velocity 이자 스트리머 한도(×1.25). 기본 config upper_body_streamer.v_max(0.4, 2026-10-28 현장 확정); input:=show 는 2.0"),
        DeclareLaunchArgument("show_acts", default_value="", description="input:=show — 동작 목록 (기본 heart,wave,run,kiss)"),
        DeclareLaunchArgument("show_speed", default_value="", description="input:=show — 시간 배율 (0.7 = 느리게, 기본 1.0)"),
        DeclareLaunchArgument("show_loop", default_value="", description="input:=show — true 면 끝나고 다시 처음부터"),
        DeclareLaunchArgument("a_max", default_value="", description="hr: 스트리머 가속 상한 [rad/s²]. 드라이브 acc_set_pp(어깨 4.0) 이하여야 드라이브가 뒤처졌다 따라잡는 떨림이 없다. 기본 config upper_body_streamer.a_max(4.0). 실행 중엔 ros2 param set /upper_body_streamer a_max <값>"),
        DeclareLaunchArgument("xacro_args", default_value="", description='xacro 인자 "name:=value …" (hand:= 보다 우선)'),
        DeclareLaunchArgument("sim_model_path", default_value="", description="mujoco_sim 에 다른 모델을 쓸 때"),
        DeclareLaunchArgument("config", default_value="", description="파라미터 yaml (기본: robot 에 따라 선택)"),
        DeclareLaunchArgument("input", default_value="shpose_udp",
                              description=" | ".join(INPUT_MODES)),
        DeclareLaunchArgument("input_convention", default_value="", description="pose_joy 좌표계: unity | openxr | unity_sh | ros"),
        DeclareLaunchArgument("sim", default_value="", description="내장 MuJoCo 시뮬 실행. 기본: hr:=true 면 false(실기체), 아니면 true. hr:=true sim:=true = HR 에뮬레이션 리허설 — 이때 실기체 릴레이(hr_udp_relay)가 떠 있으면 안 된다"),
        DeclareLaunchArgument("viewer", default_value="true", description="MuJoCo 뷰어 (Thor 데스크톱 필요)"),
        DeclareLaunchArgument("forward_axis", default_value="", description="베이스의 정면 방향 'x,y,z' (sh: 0,1,0 / g1: 1,0,0 자동)"),
        DeclareLaunchArgument("engage", default_value="", description="클러치: grip | always | on_stream (xr_teleop 기본 grip)"),
        DeclareLaunchArgument("yaw_offset", default_value="", description="입력 정면 → 로봇 정면 요 각 [deg] (xr_teleop 은 자동)"),
        DeclareLaunchArgument("operator_arm", default_value="", description="조작자 어깨→손바닥 길이 [m] (예 0.58). 주면 translation_scale = 로봇 팔 길이 / 이 값 으로 자동"),
        DeclareLaunchArgument("scale", default_value="", description="translation_scale 직접 지정 (operator_arm 이 있으면 무시). 기본은 config"),
        DeclareLaunchArgument("viewer_view", default_value="behind", description="MuJoCo 카메라: behind | front | default"),
        DeclareLaunchArgument("mock_radius", default_value="", description="mock 원 반지름 [m] (기본 0.05; 손 원 = ×translation_scale)"),
        DeclareLaunchArgument("mock_period", default_value="", description="mock 원 한 바퀴 시간 [s] (기본 6.0). 손 속도 = 2π·반지름·scale/period — 속도 시험은 이 값을 줄인다 (v_max 는 상한일 뿐)"),
        DeclareLaunchArgument("mock_lift_time", default_value="", description="mock 그립 직후 들어 올리는 시간 [s] (기본 2.0)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
