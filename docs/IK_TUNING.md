# Quest 텔레옵 IK 동작 튜닝 가이드 — `config/sh_humanoid_v2.yaml`

작성 2026-10-23, 갱신 2026-10-25 · quest_arm_ik v0.7.38 · 대상: 삼현 2세대 SH-Humanoid 실기체(HRController 내장 UDP 링크, 100 Hz)

증상: 방향은 맞는데 **종종 조작자와 영 다르게 움직인다** — 팔꿈치가 많이 꺾이거나, 손목을 움직이지 않았는데 손목이 과하게 꺾인다.

---

## 0. 결론

| 질문 | 답 |
|---|---|
| 프로그램(파이썬 코드)을 고쳐야 하나? | **아니오.** 두 증상 모두 config 파라미터로 잡는 범위다. 코드는 §6 의 경우에만. |
| 어느 파일? | **`config/sh_humanoid_v2.yaml` 의 `dual_arm_ik:` 블록.** `config/dual_arm_ik.yaml` 은 **G1 용**이라 `robot:=sh` 에서는 읽히지 않는다. |
| 적용 방법 | 파일 저장 → launch **재실행**. 재빌드 불필요(`--symlink-install`). `translation_scale`·`operator_arm_length` 두 개만 실행 중 `ros2 param set /dual_arm_ik …` 로 즉시 변경 가능. |
| `operator_arm` 은 어디에? | **launch 인자**(`operator_arm:=0.56`)다. yaml 에는 이 이름의 키가 **없는 것이 정상** — launch 가 노드 파라미터 `operator_arm_length` 로 바꿔 넘긴다. yaml 에 적어 두려면 `dual_arm_ik: ros__parameters:` 에 **`operator_arm_length: 0.56`** 을 추가(`translation_scale` 옆). 우선순위: launch `operator_arm:=` > yaml `operator_arm_length` > `translation_scale`(둘 다 없거나 0 일 때만). `operator_arm_length` 가 어디서든 0 보다 크면 `scale:=`/`translation_scale` 은 무시된다. |
| 1차로 바꿀 것 | **v0.7.35 config 를 그대로 쓰면** `nominal_posture`(내린 팔)·손목 `posture_weights 1.5`·`orientation_clutch: trigger` 는 이미 들어 있다. 남는 것: `operator_arm:=<조작자 어깨→손바닥 m>`, `v_max 0.3→0.6`, (always 로 쓸 때만) `orientation_cost`·`orientation_filter_alpha` (§3) |

컨테이너 안 경로: `/workspaces/isaac_ros-dev/src/quest_arm_ik/config/sh_humanoid_v2.yaml`
(호스트에서 열려면 `/home/sejong/workspaces/isaac_ros-dev/src/quest_arm_ik/config/sh_humanoid_v2.yaml`)

---

## 1. 지금 동작 원리 (왜 그렇게 움직이는지)

IK 노드(`dual_arm_ik`) 는 100 Hz 로 다음을 푼다.

1. **매핑**(relative): 그립을 쥔 순간의 로봇 손 자세 `hand0` 를 기준으로
   - 위치 목표 = `hand0.p + translation_scale × (컨트롤러 이동량)`
   - 회전 목표 = `hand0.R` 에 **컨트롤러 회전 변화량**을 그대로 곱한 것
2. **IK**(pink, 가중 최소자승): 손마다 `FrameTask` (위치 cost 1.0, **회전 cost 0.15**) + `PostureTask`(cost 0.02 × 관절별 `posture_weights`, 목표 `nominal_posture`) + 관절 한계·자기충돌 제약 → 관절 속도 상한 `max_joint_velocity`(= launch `v_max`).

따라서

- **팔꿈치 굽힘 정도**는 거의 전적으로 "손 목표가 어깨에서 얼마나 떨어져 있나" = **스케일**과 **그립 순간의 기준 자세**가 정한다.
- **손목 각도**는 "컨트롤러 회전을 얼마나 따라가나"(`orientation_cost`) 와 "중립 자세로 돌아가려는 힘"(`posture_weights` 손목 3개) 의 **경쟁**이 정한다. 팔을 들거나 뻗으면 손에 든 컨트롤러도 자연히 기울고, 사람과 로봇의 관절 배치가 다르니 그 회전 차이를 로봇은 **손목으로만** 메운다 → "손목을 안 움직였는데 손목이 꺾이는" 현상.
- 7축 팔에 6-DoF 과제(위치 3 + 회전 3)를 주면 여유 자유도가 1 뿐이라 자세 과제는 거의 힘을 못 쓴다. **회전 cost 를 낮추거나 0 으로 하면** 여유 자유도가 늘어 `nominal_posture`/`posture_weights` 가 손목·팔꿈치 자세를 실제로 좌우하기 시작한다(§2-2 와 §5 의 연관 항목).

---

## 2. 증상별 원인과 손잡이

### 2-1. 팔꿈치가 많이 꺾인다 → 스케일 · 기준 자세

**원인.** `translation_scale 0.75` 는 "조작자 어깨→손바닥 0.62 m" 가정값(로봇 팔 ≈ 0.47 m / 0.62). 팔이 그보다 짧은 사람이 쓰면 조작자가 팔을 반쯤 뻗어도 로봇 손 목표는 어깨 가까이에 머물러 **팔꿈치가 과하게 굽는다.** 반대로 스케일이 크면 팔을 높이 들 때 로봇 팔이 먼저 다 펴지고 손목이 꺾인다(이전 0.85 때 증상).

또 상대 매핑이라 **그립을 쥔 순간의 조작자 팔 자세가 로봇 팔 자세와 다르면** 그 뒤의 이동이 다른 출발점에서 재생된다. 로봇이 팔을 늘어뜨리고 있는데 조작자가 팔을 굽힌 채 쥐면 이후 "앞으로 뻗기"가 로봇에서는 어깨 앞쪽으로 짧게만 나간다.

**손잡이.**

| 할 일 | 방법 |
|---|---|
| 조작자 팔 길이 실측 | 어깨 관절(견봉 바로 아래 회전 중심) → 손바닥 중앙, 팔을 곧게 뻗어 잰다. 대략 키 × 0.33 (175 cm → 0.58 m). |
| launch 에 반영 | `operator_arm:=0.56` → IK 가 `translation_scale = 로봇 팔 / 0.56` 을 자동 계산하고 로그에 `translation_scale = 로봇 팔 0.4xx m / 조작자 팔 0.560 m = 0.8xx` 로 찍는다. |
| 실행 중 바꾸기 | `ros2 param set /dual_arm_ik operator_arm_length 0.56` (즉시 반영; 쥔 채면 손 목표가 조금 옮겨지니 **손을 놓은 상태**에서 바꾼다). 직접 비율을 주려면 `ros2 param set /dual_arm_ik translation_scale 0.85` (0.1~3.0). |
| 그립 습관 | **팔을 옆으로 내린 채** 그립을 쥔다(사람 어깨 = 로봇 어깨 기준점). 큰 동작 전엔 놓았다가 다시 쥔다. |
| 그래도 많이 굽으면 | 스케일을 0.05 씩 더 올려 본다(0.85 → 0.9). 너무 올리면 팔을 멀리 뻗을 때 로봇이 먼저 다 펴지며 손목이 꺾이는 반대 증상. |

> 팔꿈치가 **그립 직후부터** 로봇만 굽어 있다면 스케일이 아니라 시작 자세 문제다. IK 는 측정 관절각으로 초기화하므로 로봇 팔이 굽은 자세에서 시작하면 그 자세가 기준이 된다 — enable 전에 팔을 늘어뜨린 자세로 두고 시작한다.

### 2-2. 손목을 안 움직였는데 손목이 꺾인다 → 회전 추종 약화

**원인.** `orientation_cost 0.15` 로 컨트롤러 회전을 따라가기 때문. 팔을 들면 컨트롤러가 함께 기울고, 로봇은 그 회전을 손목(yaw·roll·pitch) 으로 보상한다. 손목 한계가 좁아(±1.4~1.6 rad) 조금만 틀어져도 "과하게 꺾인" 것으로 보인다.

**v0.7.35 의 해법 — 회전 클러치 `orientation_clutch: trigger` (SH config 기본):** 앞쪽 **검지 트리거**를 당긴 동안만 컨트롤러 회전을 따라가고(당긴 순간이 기준),
당기지 않은 동안은 회전 cost 0 + **손목 3관절 잠금**(속도 한계 ~0) 으로 위치만 추종한다 → 손목은 조작자가 의도할 때만 움직이고, 그 외에는 손 방향이 전완을 따라간다.
측면 그립 = 팔 클러치, 검지 트리거 = 손목 클러치. 그립을 놓으면 둘 다 끝. Dex3 손 쥐기와 같은 버튼이라 손을 쓰는 날은 `always` 로.
오프라인 검증(`test/test_orient_clutch.py`): 잠금 중 손을 앞·위 20 cm 옮겨도 손목 Δ ≤ 0.0002°, 손 오차 0.2 mm; 트리거 후 가로축 30° 회전은 손목이 따라감(오차 2.2°).

아래 세 손잡이는 `always` 로 쓸 때(또는 트리거를 당긴 동안의 추종 특성)를 위한 것이다 — 한 번에 다 바꾸지 말고 ①→②→③ 순으로:

| # | 파라미터 | 현재 | 1차 | 2차 | 효과 / 부작용 |
|---|---|---|---|---|---|
| ① | `orientation_cost` | 0.15 | **0.05** | **0.0** | 회전을 약하게만 따라감 → 손목이 덜 움직임. 0.0 이면 **위치만 추종**: 손목은 아래 ②의 중립 자세 쪽에 가만히 있고, 손바닥 뒤집기 같은 의도적 손목 동작은 전달되지 않는다(시연용으로 가장 안정적). |
| ② | `posture_weights` 손목 3개 (5·6·7번째, 12·13·14번째) | 0.5 | **1.5** | 2.0 | 손목이 `nominal_posture` 의 중립(yaw 0, roll 0, pitch 0.3) 을 더 세게 지킴. 순서는 v0.7.34 부터 `…, wrist_yaw, wrist_roll, wrist_pitch`. |
| ③ | `orientation_filter_alpha` | 0.3 | **0.15** | 0.1 | 회전 입력 저역통과를 더 세게(1차 slerp, 1.0 = 필터 없음). 손 떨림·순간 기울임이 손목으로 안 감. 너무 낮추면 의도적 손목 회전이 늦게 따라옴(100 Hz 에서 0.15 → 시상수 ≈ 10 ms/0.15 ≈ 70 ms, 0.3 → 33 ms). |

`orientation_cost` 를 0.05 이하로 내리면 여유 자유도가 늘어 `nominal_posture` 와 `posture_weights` 가 자세를 실제로 좌우한다 — 그래서 **§2-3 의 `nominal_posture` 수정과 반드시 함께** 한다(현재 nominal 은 손목 pitch 0.3·팔꿈치 −1.2 라 회전 추종을 약하게 하면 손목이 그쪽으로 저절로 돌아간다).

**모델 계산으로 본 "손목이 저절로 꺾이는" 진짜 원인 (2026-10-24, `ik_core` 오프라인):** 로봇의 상완 0.19 m·전완+플랜지 0.27 m 는 사람(≈0.30/0.35)과 비율이 달라, 같은(스케일된) 손 위치에서 로봇 전완이 가리키는 방향이 사람과 다르다. 컨트롤러(= 사람 손) 자세를 따라가려면 그 차이를 **손목이 떠안는다.**

| 동작 (내린 팔에서 그립 후) | 현재 config | `orientation_cost 0.05` | `0.0` (위치만) |
|---|---|---|---|
| 앞 20·위 12 cm, 컨트롤러 수평 유지 | 팔꿈치 −9°, 어깨 44°, **손목 roll 37°** | 팔꿈치 −3°, 어깨 46°, 손목 27° | 팔꿈치 −44°, 어깨 13°, 손목 0° (사람 팔 모양) |
| 앞 20·위 12 cm, 컨트롤러 30° 기움 | 팔꿈치 −44°, 어깨 18°, 손목 18° | 손목 9° | 손목 0° |
| 가슴 앞으로 들기(앞 10·위 30), 컨트롤러 전완과 함께 기움 | 팔꿈치 −97°, **상완 뒤로 34°**, 손목 roll −60° | 손목 −50° | 손목 −49° (손 프레임 위치 때문) |
| 옆으로 12 cm | 팔꿈치 −25°, 어깨 roll −19°, 손목 16°/−17° | 손목 11°/−12° | 손목 0° |

즉 `0.0` 이면 팔 모양이 사람과 가장 비슷하고 손목은 가만히 있지만 손 방향은 전완을 따라 돌아간다(의도적 손목 동작 전달 안 됨); `0.05` 는 그 중간. 시연 성격(팔 동작 모사 vs 손 방향)에 맞춰 고른다.

### 2-3. 그립만 쥐어도(가만히 있어도) 로봇 팔꿈치가 조금 굽는다 → `nominal_posture`

**현상.** 팔을 쭉 펴고 내린 채 그립을 쥐면 로봇이 입력 없이 팔꿈치를 굽힌다. 조작자가 미리 조금 굽혀도 마찬가지(−14° 로 시작해도 7° 더 굽음).

**원인 두 가지** (모델 계산, 현재 config):
1. `joint_limits_yaml` 팔꿈치 상한 0.0 + `limit_margin 0.05` → IK 는 팔꿈치를 **−2.9° 보다 펴지 못한다.** 측정값 0° 로 시작하면 첫 스텝에 −2.9° 로 간다(이것은 하드 한계라 어느 config 에서도 남음; `limit_margin 0.02` 면 −1.1°).
2. 자세 과제가 `nominal_posture` 팔꿈치 **−1.2 rad(69° 굽힘, 팀 teleop_ready)** 쪽으로 계속 끌고, 어깨 pitch 가 반대로 돌아 손 위치는 거의 그대로(≤5 mm) 둔다 → 손은 안 움직이는데 팔꿈치만 굽어 보인다. 3 s 에 −2.9° → −15.8°(어깨 −6°, 손목 roll +6°), 자연스럽게 내린 −14° 시작은 −21°.

**텔레옵 어긋남의 원인인가?** 그 자체로는 아니다 — 손 위치가 안 변하니 상대 매핑의 기준(`hand0`)도 안 변한다. 하지만 같은 끌림이 조작 중에도 계속 작용해 "팔꿈치 굽힘 + 어깨 뒤로 + 손목 0.3 쪽" 편향을 만들고, §2-2 처럼 `orientation_cost` 를 낮추면 그 편향이 더 커진다. 조작자는 **미리 굽힐 필요 없다**(자연스럽게 내리면 됨).

**수정.** 중립 자세를 "조작자가 실제로 시작하는 내린 팔" 로:

| | 현재 (nominal 팔꿈치 −1.2, 손목 pitch 0.3) | nominal 팔꿈치 −0.3, 손목 0, 손목 w1.5 |
|---|---|---|
| 팔 쭉 펴고 그립, 3 s | −2.9° → −15.8°, 손 오차 5 mm | −2.9° → −6.3°, 1 mm |
| 자연스럽게 내리고(−14°) 그립, 10 s | → −21.4°, 손목 roll +3.4° | → −14.7° (움직임 없음) |
| 조작 중 자세(§2-2 표) | — | 현재와 같음(±3°) — 동작은 바꾸지 않고 드리프트만 없앤다 |

(−0.6 은 자연 시작에서 2.6° 드리프트 — −0.3 추천.)

### 2-4. 빠르게 움직일 때 "영 다른" 경로로 따라붙는다 → 속도 상한

**원인.** `v_max:=0.3` 이면 IK 관절 속도 상한이 0.3 rad/s. 조작자가 빠르면 로봇이 뒤처진 뒤 **관절 공간의 최단 경로**로 따라붙기 때문에 경로가 사람과 달라 보인다(팔꿈치가 먼저 굽고 어깨가 나중에 가는 식). mock 실측에서 추종이 멀쩡했으니(100 Hz, err 0.03~0.085 rad) 상한을 올려도 안전하다.

**손잡이.** launch `v_max:=0.6` (→ 0.8). 스트리머 한도는 ×1.25 자동. 재동기화 경고(`resync`)가 늘거나 `/teleop/hold` 가 뜨면 다시 내린다.

---

## 3. 1차 추천값 (복사용)

`config/sh_humanoid_v2.yaml` 의 `dual_arm_ik:` → `ros__parameters:` 에서 **해당 줄만** 바꾼다.

```yaml
dual_arm_ik:
  ros__parameters:
    # ---- 로봇 ----
    nominal_posture: [0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0]     # 팔꿈치 -1.2 → -0.3, 손목 pitch 0.3 → 0: 조작자가 시작하는 '내린 팔' (§2-3, 그립 드리프트 제거)
    posture_weights: [0.1, 1.0, 1.0, 0.3, 1.5, 1.5, 1.5,
                      0.1, 1.0, 1.0, 0.3, 1.5, 1.5, 1.5]     # 손목 0.5 → 1.5 (순서: sh_pitch, sh_roll, sh_yaw, elbow, wr_yaw, wr_roll, wr_pitch)

    # ---- IK ----
    orientation_clutch: trigger        # v0.7.35 기본. 검지 트리거를 당긴 동안만 손목 추종 (§2-2). 늘 따라가게 하려면 always
    orientation_cost: 0.15             # 트리거를 당긴 동안의 회전 가중치. always 로 쓸 때 손목이 과하면 0.05 (팔 모양 모사 우선이면 0.0, §2-2 표)
    limit_margin: 0.02                 # (선택) 0.05 → 0.02: 그립 순간 팔꿈치 -2.9° → -1.1°

    # ---- 매핑·필터 ----
    operator_arm_length: 0.56          # (새 줄, translation_scale 옆) 조작자 어깨→손바닥 [m] → scale = 로봇 팔 / 이 값. launch operator_arm:= 를 매번 안 치려면 여기에
    orientation_filter_alpha: 0.15     # 0.3 → 0.15 (회전 입력 저역통과 강화)

    # ---- 자기충돌 ----
    collision_d_min: 0.04              # 실기체는 0.04 (시뮬 0.03)
```

launch (Quest, 실기체):

```bash
# 미니PC: 제어기 실행 중, 모터 전원 ON
# 터미널 A (Thor): ros2 run quest_arm_ik hr_udp_relay   →  enable 뒤 Ctrl-C (또는 아래에 relay:=false)
# 터미널 B (Thor): ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true}"
# 터미널 1 (Thor): ros2 launch isaac_ros_teleop ...  (Quest Connect)
ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=false viewer:=false \
     v_max:=0.6 operator_arm:=0.56          # ← 조작자 어깨→손바닥 실측값 [m]
```

실행 중 미세 조정(손을 놓은 상태에서):

```bash
ros2 param set /dual_arm_ik operator_arm_length 0.58     # 스케일 재계산 (로봇 팔 / 0.58)
ros2 param set /dual_arm_ik translation_scale 0.85       # 비율을 직접
ros2 param set /upper_body_streamer v_max 1.0            # 스트리머 한도만 (IK 상한은 launch v_max 로 재실행)
```

---

## 4. 시험 절차 (한 변수씩, 5분)

1. **기준 자세**: 로봇 팔을 늘어뜨린 상태에서 enable → launch. 조작자도 팔을 옆으로 내린 채 **오른쪽 그립만** 쥔다.
2. **천천히 네 동작**: 앞으로 10 cm → 위로 20 cm → 옆으로 10 cm → 제자리. 각 동작마다 2~3 s.
   - 팔꿈치 굽힘이 자기 팔과 비슷한가(과굽힘 → §2-1, 먼저 다 펴짐 → 스케일 ↓).
   - 손목이 가만히 있는가(꺾임 → §2-2 ①②③ 순서로).
3. **손목 의도 동작**: 팔을 그대로 두고 손바닥만 안/밖으로 30° 돌린다 → 로봇 손목이 따라오는지. `orientation_cost 0.0` 이면 안 따라오는 것이 정상 — 시연에 손목 동작이 필요하면 0.05~0.1 로 타협.
4. **속도**: 같은 네 동작을 1 s 안에. 경로가 사람과 다르면 `v_max` ↑(§2-3). launch 출력(`upper_body_streamer`)의 `tracking err` 가 0.1 rad 를 넘기거나 `resync`/`HOLD` 가 뜨면 ↓.
5. **양손**: 두 그립을 쥐고 2~4 반복. 한쪽만 다르면 그쪽 `joint_signs`/`joint_offsets`(config `upper_body_streamer`) 문제, 양쪽이 같으면 IK 파라미터.
6. 값이 정해지면 yaml 에 주석으로 날짜와 함께 기록(§7 표).

관찰 창: 터미널 3 에서 `ros2 run quest_arm_ik diag_teleop`(1 Hz: 노드 중복·발행자 수·IK 명령 vs 측정 팔꿈치각·엔게이지 토글·입력 수신률) 와 launch 출력의 `ik 100 Hz, motor_state 100 Hz, cmd 100 Hz, tracking err 0.0xx rad`.

---

## 5. 파라미터 표 (`dual_arm_ik` 블록, 이번 증상과 관련된 것)

| 파라미터 | 현재 | 뜻 | 올리면 | 내리면 | 실행 중 변경 |
|---|---|---|---|---|---|
| `translation_scale` | 0.75 | 컨트롤러 이동 → 손 목표 이동 비율 (= 로봇 팔 / 조작자 팔) | 같은 동작에 로봇이 더 멀리 → 팔이 먼저 펴짐·손목 꺾임 | 로봇이 덜 움직임 → 팔꿈치 과굽힘 | ✔ `ros2 param set` |
| `operator_arm_length` | (launch `operator_arm:=`) | 조작자 어깨→손바닥 [m]; 주면 scale 자동 | scale ↓ | scale ↑ | ✔ |
| `orientation_cost` | 0.15 | 손 회전 추종 가중치 (위치 1.0 대비) | 손목이 컨트롤러 회전을 더 따라감 | 손목 움직임 ↓, 0 = 위치만 | ✘ 재실행 |
| `position_cost` | 1.0 | 손 위치 추종 가중치 | (기준값, 보통 그대로) | | ✘ |
| `posture_cost` | 0.02 | 자세 과제 전체 가중치 | 모든 관절이 중립 쪽으로 더 끌림(추종 정확도 ↓) | 자세 과제 약화 | ✘ |
| `posture_weights` | `[0.1,1,1,0.3,0.5,0.5,0.5]×2` | 관절별 자세 가중치(sh_pitch, sh_roll, sh_yaw, elbow, wr_yaw, wr_roll, wr_pitch) | 그 관절이 `nominal_posture` 를 더 지킴 | 자유롭게 움직임 | ✘ |
| `elbow_hint_cost` | 0.5 (v0.7.49 시연 최종 — 포즈 유사성 우선; 거슬리면 0) | 팔꿈치 위치 과제 가중 (손 1.0 대비): 손 목표 − 전완 길이 × 조작자 전완 방향(컨트롤러 회전 변화로 추정) | 팔꿈치가 사람을 더 따라감, 손 오차 ↑(1.0 → 10 mm) | 0 = 끔(자세 과제가 팔꿈치를 정함) | ✘ |
| `forearm_filter_alpha` | 0.3 | 힌트 방향 저역통과 (1 = 없음) | 반응 ↑ | 떨림 ↓ | ✘ |
| `nominal_posture` | `[0,0,0,-1.2,0,0,0.3]×2` | 자세 과제의 목표(rad). 팔꿈치 −1.2 = 굽힘 | 현재값은 그립만 쥐어도 팔꿈치가 굽는 원인(§2-3) | **`[0,0,0,-0.3,0,0,0]×2` 추천** (조작자의 내린 팔) | ✘ |
| `orientation_filter_alpha` | 0.3 | 회전 입력 1차 slerp 저역통과 (1 = 없음) | 반응 빨라짐·떨림 전달 | 매끄럽지만 늦음 | ✘ |
| `filter_min_cutoff` / `filter_beta` | 2.0 / 1.0 (v0.7.49 시연 최종; 5/10 은 지연 20 ms 지만 조작자 손의 2~10 Hz 가 드라이브 3~4 Hz 링잉을 키움) | 위치 입력 One-Euro 필터 — 지연 ≈ 60~80 ms 와 떨림의 균형 | 반응 ↑ (3.0 / 2.0) | 떨림 ↓ (1.5 / 0.5) | ✘ |
| `max_joint_velocity` | 1.5 (launch `v_max` 가 덮어씀) | IK 관절 속도 상한 [rad/s] | 빠른 동작 추종, 경로 자연스러움 | 안전·느림, 뒤처진 뒤 최단경로 추종 | ✘ (launch `v_max:=`) |
| `max_step` | 0.03 | 한 주기(10 ms) 손 목표 이동 상한 [m] (튀는 패킷 방어) | | 빠른 손동작이 잘림 | ✘ |
| `joint_limits_yaml` | 팔꿈치 [−1.8, 0] | IK 전용 관절 한계 (URDF 보다 좁게) | | 팔꿈치가 더 못 굽음(−1.5 등) — 과굽힘의 하드 리밋으로도 쓸 수 있음 | ✘ |
| `limit_margin` | 0.05 | 한계 안쪽 여유 [rad] | | | ✘ |
| `collision_d_min` | 0.03 | 자기충돌 최소 거리 [m] | 몸에 덜 붙음 | | ✘ |
| `resync_threshold` / `resync_ticks` | 0.3 / 40 | 명령-측정 차가 0.3 rad 를 40 틱(0.4 s) 넘으면 IK 를 측정값으로 재동기화 | | | ✘ |

`✘` 는 yaml 수정 후 launch 재실행(재빌드 불필요). 확실치 않으면 컨테이너 워크스페이스 루트에서 `colcon build --symlink-install --packages-select quest_arm_ik` 한 번.

---

## 6. 코드 수정이 필요한 경우 (config 로 안 잡힐 때만)

| 상황 | 추가할 것 | 크기 |
|---|---|---|
| ~~손목 추종을 조작자 의도가 있을 때만~~ → **v0.7.35 에 구현됨**: `orientation_clutch: trigger` (§2-2) | — | 완료 |
| 손목 회전을 **일부만**(예 50 %) 따라가고 싶다 — 회전 **각도 자체**를 줄이고 싶을 때 | `mapper.py` 에 `orientation_scale`(0~1): `R_target = slerp(hand0.R, R_full, s)` | 작음 (파라미터 1개 + 3줄) |
| 손목 **일부 축만** 추종(예 손바닥 뒤집기 roll 만 따라가고 pitch/yaw 는 고정) | `ik_core.py` `FrameTask` 의 `orientation_cost` 를 스칼라 대신 3-벡터로 받게 (pink 는 축별 cost 지원) + config 에 `orientation_cost: [0.0, 0.1, 0.0]` 형태 | 작음 |
| 팔꿈치 굽힘을 조작자 **팔꿈치 각도**로 직접 맞추고 싶다 | Quest 는 손 자세만 준다. v0.7.38 의 팔꿈치 힌트가 컨트롤러 회전으로 전완 방향을 추정해 근사(§8 표 4) | 완료 |

필요하면 바로 패치로 만들어 드립니다(둘 다 1시간 이내, 기존 config 와 호환).

---

## 7. 시도 기록 (채워 넣기)

| 날짜 | operator_arm / scale | orientation_cost | 손목 posture_weights | filter_alpha | v_max | 팔꿈치 | 손목 | 비고 |
|---|---|---|---|---|---|---|---|---|
| 2026-10-23 | (기본 0.75) | 0.15 | 0.5 | 0.3 | 0.3 | 종종 과굽힘, 그립만 쥐어도 조금 굽음 | 안 움직였는데 꺾임 | 방향은 맞음 (오른팔). nominal 팔꿈치 −1.2 |
| | | 0.05 | 1.5 | 0.15 | 0.6 | | | 1차 추천 |
| | | 0.0 | 1.5 | 0.15 | 0.6 | | | 2차 (위치만) |
| | | | | | | | | |

---

## 8. 부드럽게·사람과 맞게 — 지연 줄이기 (2026-10-25)

G1 영상과의 차이는 IK 가 아니라 **지연과 응답**에서 온다: 드라이브 위치 루프(`acc_set_pp` 2 rad/s², `loc_kp` ~12 → ≈0.12 s) + One-Euro 입력 필터(`min_cutoff 1.5`, `beta 0.5` 는 속도가 m/s 라 거의 안 먹어 항상 ≈0.1 s) + `v_max 0.5` 포화. 순서:

| # | 할 것 | 어디 | 기대 |
|---|---|---|---|
| 0 | `ros2 run quest_arm_ik diag_latency` 로 **기준값**(필터·드라이브 지연, 떨림, 포화율) 기록 | Thor | — |
| 1 | 드라이브 튜닝: `hr_controller_tuning_peraxis.patch` + 미니PC config `position_tuning: {acc_set_pp_rad_s2: 8.0}` → 축별 `loc_kp` ×1.5 → ×2 (축 10 은 축 3 값) | 미니PC | 드라이브 지연 0.12 → 0.05 s |
| 2 | 입력 필터: `filter_min_cutoff: 4.0`, `filter_beta: 10.0`, `task_gain: 0.8` (떨리면 3.0 / 5.0) — **실기체 결론(2026-10-26, 런북 7-0)**: 빠른 필터는 조작자 손의 2~10 Hz 를 통과시켜 드라이브 3~4 Hz 링잉을 키우므로 시연은 2.0 / 1.0, task_gain 0.5 | yaml | 필터 지연 80 → 30 ms (가짜 체인 실측); 시연값은 ≈ 60~80 ms |
| 3 | `v_max` 0.5 → 1.0 → 1.5 (1 번 뒤에; 포화율·HOLD 보며) | launch | 빠른 동작이 잘리지 않음 |
| 4 | 팔꿈치 힌트 — **v0.7.38 구현**, SH config `elbow_hint_cost: 0.5` (끄려면 0). 그립 순간 로봇 전완 방향을 컨트롤러 회전 변화만큼 돌린 방향으로 팔꿈치 위치 과제 | yaml | 팔꿈치 들기 동작에서 전완 방향 오차 최대 45° → 21° (모델), 손 오차 +4 mm |
| 5 | 다시 `diag_latency` 로 전후 비교 | Thor | 목표: 필터 ≤ 50, 드라이브 ≤ 60 ms, 떨림 σ ≤ 2 mm, 포화 ≤ 5 % |

2 는 1 과 독립이라 먼저 해도 되고, 3 은 반드시 1 뒤에(드라이브가 못 따라오면 재동기화·HOLD 만 늘어난다).

### 참고 (관련 런북 항목)
- RUNBOOK 6-6 "실기체 — Quest": 그립은 팔을 내린 자세에서, 작은 동작 → 들어 올리기 → 양손, `v_max` 0.3 → 0.5 → 0.8.
- RUNBOOK 4단계 표: 손 떨림 → `filter_min_cutoff` 1.5→1.0, `orientation_filter_alpha` 0.3→0.2; 손목이 억지로 비틀림 → `orientation_cost` ↓.
- RUNBOOK 8 "막혔을 때": 팔을 높이 들수록 사람과 다르게 움직임 → 스케일(`operator_arm:=`), 그립은 팔을 내린 자세에서.
- 드라이버 쪽 추종 지연(≈0.12 s, `loc_kp` ~12·`acc_set_pp` 2.0) 은 별건: `hr_controller_tuning_peraxis.patch` + 미니PC config `position_tuning` (RUNBOOK 6-1e).
