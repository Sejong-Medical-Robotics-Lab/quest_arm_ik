#!/usr/bin/env bash
# 미니PC(samhyun)에서 실행 — hr_controller 용 tmux 세션 'hr' (창 1 ctrl: 제어기, 창 2 log: tail -F ~/hr.log).
#
#   bash ~/minipc_hr_tmux.sh     # 처음: 세션 생성 후 붙기 / 다시: 그냥 붙기.  맥북에서:  ssh -t minipc 'bash ~/minipc_hr_tmux.sh'
#   창 1 에 제어기 명령이 입력만 되어 있다 — 모터 전원을 켠 뒤 Enter. (전원 전에 띄우면 boot scan 0/15 → Ctrl-C 후 다시 Enter)
#   ssh 가 끊겨도 제어기는 tmux 안에서 계속 돈다. 제어기만 멈추기: 창 1 에서 Ctrl-C.  세션 종료: tmux kill-session -t hr
# tmux 가 없으면(Ubuntu Server 에는 보통 있음: which tmux):  nohup ./run_hr_controller.sh > ~/hr.log 2>&1 &   로 띄우고  tail -F ~/hr.log
set -e
S=hr
if ! command -v tmux >/dev/null; then echo "tmux 없음 — nohup ./run_hr_controller.sh > ~/hr.log 2>&1 &  /  tail -F ~/hr.log 로 대신"; exit 1; fi
if tmux has-session -t "$S" 2>/dev/null; then exec tmux attach -t "$S"; fi
[ -f ~/.tmux.conf ] || printf 'set -g mouse on\nset -g history-limit 50000\nset -g base-index 1\n' > ~/.tmux.conf
tmux new-session -d -s "$S" -n ctrl -c "$HOME/hr_controller"
tmux new-window -t "$S" -n log -c "$HOME"
tmux send-keys -t "$S:ctrl" './run_hr_controller.sh > ~/hr.log 2>&1'
tmux send-keys -t "$S:log"  'tail -F ~/hr.log' C-m
tmux select-window -t "$S:ctrl"
exec tmux attach -t "$S"
