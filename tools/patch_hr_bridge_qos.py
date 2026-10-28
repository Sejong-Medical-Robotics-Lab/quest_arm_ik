#!/usr/bin/env python3
"""삼현 ros2_state_bridge.py 에 QoS 수정 ①②를 적용한다 (2026-10-21 원인 확정: RELIABLE 상태 발행의 publish() 가 ACK 대기로 ~2 s 막혀 브리지 전체가 얼음).

  ① 상태 토픽(system_state / motor_states / axis_map) 발행 QoS: RELIABLE depth 10 → BEST_EFFORT depth 5 (sensor-data)
  ② axis_map(정적, 약 4 KB JSON)은 10 Hz 로 매번 보내지 않고 '내용이 바뀌었을 때 또는 1 s 마다' 만 발행

사용 (미니PC):  python3 patch_hr_bridge_qos.py ~/hr_controller/tools/ros2_state_bridge.py
  - 원본은 같은 자리에 .orig 로 백업한다 (이미 있으면 덮어쓰지 않음). 되돌리기: cp ros2_state_bridge.py.orig ros2_state_bridge.py
  - 찾는 문구가 없으면(배포본이 다르면) 아무것도 바꾸지 않고 어느 항목이 안 맞는지 알려 준다. 두 번 실행해도 안전(이미 적용됐으면 건너뜀).
"""
import pathlib
import shutil
import sys

P1_OLD = "        state_qos = qos_profile(ReliabilityPolicy.RELIABLE, 10)\n"
P1_NEW = ("        # (2026-10-21) 상태 토픽은 BEST_EFFORT: RELIABLE 이면 구독자 ACK 를 기다리는 publish() 가 패킷 유실 시 ~2 s 막혀 브리지 전체가 선다.\n"
          "        #  BEST_EFFORT 발행자는 RELIABLE 구독자와 짝이 안 맞으므로 `ros2 topic echo` 는 --qos-reliability best_effort 로.\n"
          "        state_qos = qos_profile(ReliabilityPolicy.BEST_EFFORT, 5)\n")

P2_OLD = ("        axis_map_msg = String()\n"
          "        axis_map_msg.data = json.dumps(axis_map, ensure_ascii=True)\n"
          "        self.axis_map_pub.publish(axis_map_msg)\n")
P2_NEW = ("        # (2026-10-21) axis_map 은 정적이고 크다(약 4 KB → IP 조각 3개): 내용이 바뀌었거나 1 s 가 지났을 때만 발행\n"
          "        axis_map_json = json.dumps(axis_map, ensure_ascii=True)\n"
          "        now_mono = time.monotonic()\n"
          "        if (axis_map_json != getattr(self, \"_last_axis_map_json\", None)\n"
          "                or now_mono - getattr(self, \"_last_axis_map_pub\", 0.0) >= 1.0):\n"
          "            axis_map_msg = String()\n"
          "            axis_map_msg.data = axis_map_json\n"
          "            self.axis_map_pub.publish(axis_map_msg)\n"
          "            self._last_axis_map_json = axis_map_json\n"
          "            self._last_axis_map_pub = now_mono\n")


def main():
    if len(sys.argv) != 2:
        print(__doc__); return 2
    path = pathlib.Path(sys.argv[1]).expanduser()
    src = path.read_text(encoding="utf-8")
    plan = []
    for name, old, new in (("① state_qos BEST_EFFORT", P1_OLD, P1_NEW), ("② axis_map 변경 시/1 Hz 발행", P2_OLD, P2_NEW)):
        if new in src:
            print(f"{name}: 이미 적용됨")
        elif src.count(old) == 1:
            plan.append((name, old, new))
        else:
            print(f"{name}: 찾는 문구가 {src.count(old)}회 — 배포본이 다릅니다. 이 항목은 손으로 고쳐야 합니다 (아래 '찾을 문구' 참고):\n{old}")
    if not plan:
        print("바꿀 것 없음"); return 0
    if "import time" not in src:
        print("경고: 'import time' 이 없음 — ② 가 time.monotonic 을 쓰므로 파일 머리에 import time 을 추가해야 함")
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup); print(f"백업: {backup}")
    for name, old, new in plan:
        src = src.replace(old, new); print(f"{name}: 적용")
    path.write_text(src, encoding="utf-8")
    print(f"저장: {path}\n브리지를 재시작하세요 (tty2 Ctrl-C 후 run_ros2_state_bridge.sh). Thor 에서 `ros2 topic info -v /upper_body/axis_map` 의 Publisher Reliability 가 BEST_EFFORT 면 적용된 것.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
