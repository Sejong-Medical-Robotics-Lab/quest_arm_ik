#!/usr/bin/env python3
"""미니PC config/hr_controller.yaml 의 축 항목에 position_tuning 키(loc_kp, acc_set_pp_rad_s2)를 단계별로 넣고 빼는 도구 (v2.2 패치용).
주석·순서를 건드리지 않도록 줄 단위로 편집하고, 원본은 hr_controller.yaml.bak 으로 남긴다.

    python3 ~/tune_axes.py --stage acc            # 1단계: 가속만 (어깨 pitch/roll 1.7~2.0 → 4.0, 나머지 5.0 → 8.0) — 2026-10-26 verify 통과 (14축 지연 36~60 ms)
    python3 ~/tune_axes.py --stage acc --set 3=5.0 --set 5=4.0   # 1단계 + 왼팔 yaw 두 축(read-back 0.57)만 오른팔 수준(4.5/2.6)으로 — 유지 잔차 10~17 mrad 가 더딘 것만 고침
    python3 ~/tune_axes.py --stage acc2           # 1단계 + 어깨 가속 6.0 (v_max 1.0 텔레옵에서 어깨 재동기화가 날 때; verify 1번축 측정속도 1.38 = 가속 구간 따라잡기)
    python3 ~/tune_axes.py --stage kp1            # (선택) 가속 + loc_kp 1차 (현재값 ×1.5) — 실측상 지연은 loc_kp 와 무관(0.57 과 17.6 이 같은 50 ms), 중력 처짐만 ∝ 1/kp
    python3 ~/tune_axes.py --stage kp2            # (선택) 가속(어깨 6.0) + loc_kp 2차 (바닥 6~10, 어깨 22~24) — 떨림 위험 대비 이득 작음
    python3 ~/tune_axes.py --stage off            # 튜닝 키 제거 (드라이브 값 유지로 복귀)
    python3 ~/tune_axes.py --stage kp1 --set 12=5.0 --set 13=9.0   # 특정 축만 덮어쓰기 (loc_kp)
    python3 ~/tune_axes.py --stage acc --acc 1=2.0 --acc 8=2.0     # 특정 축만 가속 덮어쓰기 (1·8번 어깨 pitch 가 4.0 을 거부할 때 상한 찾기)
    python3 ~/tune_axes.py --stage acc --dry-run  # 바뀔 내용만 출력

실측 기준 (2026-10-26 enable read-back):
  loc_kp     1:17.6 2:14.6 3:0.57 4:8.8 5:0.57 6:5.1 7:2.9 | 8:14.6 9:11.7 10:4.5 11:6.8 12:2.6 13:6.8 14:4.9
  acc_set_pp 1:1.7 2:2.0 3~7:5.0 | 8:1.7 9:2.0 10~14:5.0
축 순서: 1 L어깨pitch 2 L어깨roll 3 L어깨yaw 4 L팔꿈치 5 L손목yaw 6 L손목roll 7 L손목pitch / 8~14 오른팔 같은 순서. 15 머리는 건드리지 않음.
"""
import argparse
import os
import re
import shutil
import sys

ACC = {
    "acc": {1: 4.0, 2: 4.0, 8: 4.0, 9: 4.0, **{a: 8.0 for a in (3, 4, 5, 6, 7, 10, 11, 12, 13, 14)}},
    "acc2": {1: 6.0, 2: 6.0, 8: 6.0, 9: 6.0, **{a: 8.0 for a in (3, 4, 5, 6, 7, 10, 11, 12, 13, 14)}},
    "kp1": {1: 4.0, 2: 4.0, 8: 4.0, 9: 4.0, **{a: 8.0 for a in (3, 4, 5, 6, 7, 10, 11, 12, 13, 14)}},
    "kp2": {1: 6.0, 2: 6.0, 8: 6.0, 9: 6.0, **{a: 10.0 for a in (3, 4, 5, 6, 7, 10, 11, 12, 13, 14)}},
}
KP = {
    "acc": {},
    "acc2": {},
    "kp1": {1: 20.0, 2: 18.0, 3: 5.0, 4: 12.0, 5: 4.0, 6: 7.0, 7: 5.0,
            8: 18.0, 9: 16.0, 10: 7.0, 11: 10.0, 12: 4.0, 13: 9.0, 14: 7.0},
    "kp2": {1: 24.0, 2: 22.0, 3: 8.0, 4: 15.0, 5: 6.0, 6: 10.0, 7: 8.0,
            8: 24.0, 9: 22.0, 10: 10.0, 11: 15.0, 12: 6.0, 13: 12.0, 14: 10.0},
}
KEYS = ("loc_kp", "acc_set_pp_rad_s2", "limit_spd_rad_s", "vel_max_pp_rad_s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.expanduser("~/hr_controller/config/hr_controller.yaml"))
    ap.add_argument("--stage", required=True, choices=["acc", "acc2", "kp1", "kp2", "off"])
    ap.add_argument("--set", action="append", default=[], help="축=loc_kp 로 개별 덮어쓰기 (예 --set 12=5.0)")
    ap.add_argument("--acc", action="append", default=[], help="축=acc_set_pp 로 개별 덮어쓰기 (예 --acc 1=2.0 --acc 8=2.0)")
    ap.add_argument("--dry-run", action="store_true")
    ns = ap.parse_args()

    kp = dict(KP.get(ns.stage, {}))
    for item in ns.set:
        a, v = item.split("=")
        kp[int(a)] = float(v)
    acc = dict(ACC.get(ns.stage, {}))
    for item in ns.acc:
        a, v = item.split("=")
        acc[int(a)] = float(v)

    lines = open(ns.config, encoding="utf-8").read().splitlines()
    out, axis, in_axes, changes = [], None, False, []
    for ln in lines:
        if re.match(r"^axes:\s*$", ln):
            in_axes = True
        elif re.match(r"^[A-Za-z_]+:", ln):
            in_axes = False
        m = re.match(r"^\s*-\s*axis_id:\s*(\d+)", ln)
        if in_axes and m:
            axis = int(m.group(1))
        # 기존 튜닝 키는 모두 제거 (새 값으로 다시 넣음)
        if in_axes and axis is not None and re.match(r"^\s+(" + "|".join(KEYS) + r"):", ln):
            continue
        out.append(ln)
        # 각 축 블록의 마지막 고정 키(position_limit_max_deg 또는 position_limit_max_rad) 뒤에 삽입
        if in_axes and axis is not None and re.match(r"^\s+position_limit_max_(deg|rad):", ln):
            indent = re.match(r"^(\s+)", ln).group(1)
            if axis in kp:
                out.append(f"{indent}loc_kp: {kp[axis]}")
                changes.append(f"axis {axis}: loc_kp {kp[axis]}")
            if axis in acc:
                out.append(f"{indent}acc_set_pp_rad_s2: {acc[axis]}")
                changes.append(f"axis {axis}: acc_set_pp {acc[axis]}")
    # 전역 position_tuning 블록은 쓰지 않는다 (축별 값이 우선이라 혼동 방지) — 있으면 그대로 둠
    text = "\n".join(out) + "\n"
    print(f"[{ns.stage}] {len(changes)}개 변경:")
    for c in changes:
        print("  " + c)
    if ns.dry_run:
        print("(dry-run: 파일은 그대로)")
        return 0
    bak = ns.config + ".bak"
    if not os.path.exists(bak):
        shutil.copy2(ns.config, bak)
        print(f"원본 백업: {bak}")
    open(ns.config, "w", encoding="utf-8").write(text)
    print(f"저장: {ns.config}  → 제어기 재시작(tmux 창 1 Ctrl-C 후 Enter) → enable → grep -h \"OD 0x701E\\|OD 0x7025\" ~/hr.log | sort -u 로 read-back 확인")
    return 0


if __name__ == "__main__":
    sys.exit(main())
