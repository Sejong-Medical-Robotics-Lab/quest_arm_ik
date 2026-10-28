#!/usr/bin/env bash
# 미니PC 에서 verify 가 도는 동안 브리지가 '커널 어디에서' 막히는지 본다 (2026-10-21: 직결 케이블에서도 publish/sendto 가 0.3~2.5 s 막힘).
#   사용: ./hr_net_probe.sh [초, 기본 25]      (환경: IF=eno1 PEER=192.168.10.1)
#   출력: 브리지 메인 스레드의 wchan(커널에서 잠든 함수)과 ARP 상태가 바뀔 때마다 한 줄, 끝에 UDP/링크 카운터 전후 비교.
#     wchan 이 sock_wait_for_wmem / sock_alloc_send_pskb 로 오래 머물면 = 소켓 송신 버퍼가 꽉 차 sendto 블로킹 (패킷이 NIC 를 못 떠남)
#     wchan 이 futex_wait* 로 오래 머물면            = 사용자 공간 락(Fast DDS 뮤텍스) 대기
#     neigh 가 STALE/DELAY/PROBE/INCOMPLETE/FAILED 로 돌면 = ARP 가 제때 안 풀림 (패킷이 ARP 큐에 묶였다가 한꺼번에 나감 = '옛 값 재생')
#     Udp SndbufErrors 나 tx_errors/tx_dropped 가 늘면      = 송신 경로 오류
set -u
PID=$(pgrep -f "hr_bridge_timed.py|ros2_state_bridge.py" | head -n 1)
if [ -z "${PID}" ]; then echo "브리지 프로세스가 없음 (ros2_state_bridge.py / hr_bridge_timed.py)"; exit 1; fi
IF=${IF:-eno1}; PEER=${PEER:-192.168.10.1}; DUR=${1:-25}
echo "pid=$PID if=$IF peer=$PEER dur=${DUR}s"
ethtool "$IF" 2>/dev/null | grep -E "Speed|Duplex|Auto-negotiation"
ethtool --show-eee "$IF" 2>/dev/null | grep -iE "EEE status|Tx LPI"
cat /proc/pressure/cpu 2>/dev/null | sed 's/^/cpu pressure: /'
udp_hdr=$(grep -E "^Udp:" /proc/net/snmp | head -n 1); udp0=$(grep -E "^Udp:" /proc/net/snmp | tail -n 1)
S=/sys/class/net/$IF/statistics
tx_err0=$(cat "$S/tx_errors"); tx_drop0=$(cat "$S/tx_dropped"); rx_err0=$(cat "$S/rx_errors"); rx_drop0=$(cat "$S/rx_dropped")
end=$((SECONDS + DUR)); last=""; t_last=$(date +%s.%N)
while [ "$SECONDS" -lt "$end" ]; do
  main=$(cat "/proc/$PID/wchan" 2>/dev/null || echo "?")
  others=$(for t in /proc/$PID/task/*; do [ "$(basename "$t")" = "$PID" ] && continue; cat "$t/wchan" 2>/dev/null; done | sort | uniq -c | awk '{printf "%s x%d ", $2, $1}')
  n=$(ip -4 neigh show "$PEER" 2>/dev/null | awk '{print $NF}'); [ -z "$n" ] && n="(none)"
  line="neigh=$n main=$main | $others"
  if [ "$line" != "$last" ]; then
    now=$(date +%s.%N); dt=$(awk -v a="$now" -v b="$t_last" 'BEGIN{printf "%.2f", a-b}'); printf "%s (+%ss) %s\n" "$(date +%T)" "$dt" "$line"; last="$line"; t_last=$now
  fi
  sleep 0.1
done
echo "---- 카운터 (전 → 후)"
echo "$udp_hdr"; echo "before $udp0"; echo "after  $(grep -E "^Udp:" /proc/net/snmp | tail -n 1)"
echo "$IF tx_errors $tx_err0 -> $(cat "$S/tx_errors"), tx_dropped $tx_drop0 -> $(cat "$S/tx_dropped"), rx_errors $rx_err0 -> $(cat "$S/rx_errors"), rx_dropped $rx_drop0 -> $(cat "$S/rx_dropped")"
dmesg 2>/dev/null | grep -iE "\b$IF\b|NETDEV WATCHDOG|link is (up|down)" | tail -n 5
