#!/usr/bin/env bash
# Thor 호스트(컨테이너 밖)에서 실행 — 시연용 tmux 세션 'demo' 를 만들고 붙는다. 맥북에서 모니터 없이 ssh 로 운영할 때 쓴다.
#
#   bash ~/thor_demo_tmux.sh         # 처음: 세션 생성(창 6개, 각 창 = Isaac ROS 컨테이너 셸 + env.sh) 후 붙기
#   bash ~/thor_demo_tmux.sh         # 다시: 이미 있으면 그냥 붙는다 (= tmux attach -t demo). ssh 가 끊겨도 세션·프로세스는 살아 있다
#   맥북에서 한 줄로:  ssh -t thor 'bash ~/thor_demo_tmux.sh'
#
# 창(window):  1 cloudxr  2 relay  3 enable  4 teleop  5 diag  6 show(동작 쇼 — 텔레옵 launch 를 내린 뒤)
# 각 창에는 명령을 '입력만' 해 둔다 — 확인하고 Enter. 실행 순서: 1(CloudXR, Quest Connect) → 2(relay) → 3(enable) → 4(launch) → 5(diag);  동작 쇼: 4 를 Ctrl-C 로 내리고 6(show) → 3(enable)
# 창 이동: Ctrl-b 숫자 / Ctrl-b n·p,  떼기: Ctrl-b d,  스크롤: 마우스 휠(~/.tmux.conf 의 mouse on) 또는 Ctrl-b [ (q 로 나옴)
# 어떤 창의 명령만 멈추려면 그 창에서 Ctrl-C. 세션 전체 종료: tmux kill-session -t demo
set -e
S=demo
ENV='source /workspaces/isaac_ros-dev/src/quest_arm_ik/scripts/env.sh'
V_MAX="${V_MAX:-0.4}"                       # 현장 확정 2026-10-28 (config upper_body_streamer.v_max 와 같게)
OPERATOR_ARM="${OPERATOR_ARM:-0.56}"        # 조작자 어깨→손바닥 [m]

if ! command -v tmux >/dev/null; then echo "tmux 가 없다: sudo apt install tmux"; exit 1; fi
if tmux has-session -t "$S" 2>/dev/null; then exec tmux attach -t "$S"; fi

# 공통 설정(없을 때만): 마우스 스크롤·창 클릭, 긴 스크롤백, 창 번호 1부터
[ -f ~/.tmux.conf ] || cat > ~/.tmux.conf <<'EOF'
set -g mouse on
set -g history-limit 50000
set -g base-index 1
setw -g pane-base-index 1
set -g status-left-length 30
set -g status-right '#H  %H:%M'
EOF

tmux new-session -d -s "$S" -n cloudxr
tmux new-window -t "$S" -n relay
tmux new-window -t "$S" -n enable
tmux new-window -t "$S" -n teleop
tmux new-window -t "$S" -n diag
tmux new-window -t "$S" -n show

# 모든 창: 컨테이너 셸 진입 후 env.sh (컨테이너가 아직 없으면 첫 창에서 만들어지는 동안 다른 창은 기다린다)
for w in cloudxr relay enable teleop diag show; do
  tmux send-keys -t "$S:$w" 'isaac-ros activate' C-m
  sleep 2
done
sleep 4
for w in cloudxr relay enable teleop diag show; do
  tmux send-keys -t "$S:$w" "$ENV" C-m
done
sleep 1

# 명령은 입력만 (Enter 는 사람이)
tmux send-keys -t "$S:cloudxr" 'ros2 launch isaac_ros_teleop isaac_ros_teleop.launch.py cloudxr_accept_eula:=true'
tmux send-keys -t "$S:relay"   'ros2 run quest_arm_ik hr_udp_relay'
tmux send-keys -t "$S:enable"  'ros2 service call /upper_body/enable_motors ros2_interfaces/srv/EnableMotors "{enable_all: true}"'
tmux send-keys -t "$S:teleop"  "ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=xr_teleop hr:=true sim:=false viewer:=false relay:=false v_max:=$V_MAX operator_arm:=$OPERATOR_ARM grasp:=false"
tmux send-keys -t "$S:diag"    'ros2 run quest_arm_ik diag_teleop'
# 동작 쇼 (런북 7-2): 텔레옵 launch(창 4)를 내린 뒤 Enter. sim:=false relay:=false 필수 — 릴레이는 창 2 가 띄우고, sim 이 켜지면 HR 에뮬레이션이 실기체와 섞인다(2026-10-28 사고)
tmux send-keys -t "$S:show"    'ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=show hr:=true sim:=false viewer:=false relay:=false grasp:=false show_speed:=0.5'
tmux select-window -t "$S:cloudxr"
exec tmux attach -t "$S"
