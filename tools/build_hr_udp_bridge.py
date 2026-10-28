#!/usr/bin/env python3
"""quest_arm_ik/hr_udp_proto.py + hr_udp_bridge_main.py → tools/hr_udp_bridge.py (미니PC 용 단독 스크립트, 표준 라이브러리만).
   python3 tools/build_hr_udp_bridge.py          # 생성
   python3 tools/build_hr_udp_bridge.py --check  # 최신인지 확인 (테스트에서 사용)"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
PROTO = ROOT / "quest_arm_ik" / "hr_udp_proto.py"
MAIN = ROOT / "quest_arm_ik" / "hr_udp_bridge_main.py"
OUT = ROOT / "tools" / "hr_udp_bridge.py"


def build() -> str:
    proto = PROTO.read_text(encoding="utf-8")
    main = MAIN.read_text(encoding="utf-8")
    # main 의 패키지 import 블록(try: from .hr_udp_proto … except ImportError: pass)을 제거 — proto 가 같은 파일에 들어간다
    start = main.index("try:  # 패키지 안에서 실행될 때")
    end = main.index("    pass\n", start) + len("    pass\n")
    main_body = main[:start] + "# (proto 는 이 파일 위쪽에 포함됨)\n" + main[end:]
    main_body = main_body.replace("from __future__ import annotations\n", "", 1)
    header = ('#!/usr/bin/env python3\n'
              '# 자동 생성 파일 — 고치려면 quest_arm_ik/hr_udp_proto.py, hr_udp_bridge_main.py 를 고치고 tools/build_hr_udp_bridge.py 를 실행.\n'
              '# 미니PC 에서: python3 hr_udp_bridge.py   (ROS 불필요, Python 3.8+)\n')
    return header + proto + "\n\n# ======================= bridge =======================\n" + main_body


def main():
    text = build()
    if "--check" in sys.argv:
        cur = OUT.read_text(encoding="utf-8") if OUT.exists() else ""
        if cur != text:
            print("tools/hr_udp_bridge.py 가 오래됨 — python3 tools/build_hr_udp_bridge.py 실행"); return 1
        print("hr_udp_bridge.py 최신"); return 0
    OUT.write_text(text, encoding="utf-8")
    OUT.chmod(0o755)
    print(f"wrote {OUT} ({len(text.splitlines())} lines)"); return 0


if __name__ == "__main__":
    sys.exit(main())
