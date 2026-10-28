#!/usr/bin/env python3
"""동작 쇼 안무를 ROS 없이 MuJoCo 로 렌더해 GIF(미리보기) 와 키프레임 PNG 를 만든다 — 모니터 없는 Thor 에서도(EGL 오프스크린) 됨.

    MUJOCO_GL=egl SH_REPO=$SH $QPY tools/render_show.py out.gif [keyframes.png] [--acts heart,wave] [--speed 0.7] [--fps 8]
    # 맥북으로: scp thor:~/workspaces/isaac_ros-dev/src/quest_arm_ik/out.gif ~/Downloads/

같이 찍는 검사: 관절 속도 최대, 관절 한계 여유, 50 ms 간격 MuJoCo 접촉(자기충돌) 횟수 — 접촉이 0 이 아니면 안무를 고칠 것.
"""
import argparse
import os
import sys

import mujoco
import numpy as np
import yaml
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from quest_arm_ik.choreography import build_show, check_trajectory  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gif")
    ap.add_argument("keyframes", nargs="?", default="")
    ap.add_argument("--acts", default="heart,wave,run,kiss")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--v-max", type=float, default=1.0)
    ap.add_argument("--fps", type=int, default=8)
    ap.add_argument("--sh-repo", default=os.environ.get("SH_REPO", ""))
    ns = ap.parse_args()
    if not ns.sh_repo:
        print("SH_REPO (SH-Humanoid 저장소 경로) 필요"); return 1
    repo = os.path.join(ns.sh_repo, "urdf_moveit")
    xacro = os.path.join(repo, "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(repo, "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))
    joints = cfg["dual_arm_ik"]["ros__parameters"]["controlled_joints"]
    shapes = yaml.safe_load(cfg["mujoco_sim"]["ros__parameters"]["collision_shapes_yaml"])
    sim = MujocoArmSim(xacro, joints, keyframe="", pd_kp=300.0, pd_kd=10.0, package_dirs=[repo], collision_shapes=shapes, srdf_path=srdf, xacro_args="")
    m, d = sim.model, sim.data

    show = build_show([a.strip() for a in ns.acts.split(",") if a.strip()], speed=ns.speed, v_max=ns.v_max)
    Q, vmax, lim = check_trajectory(show, v_max=ns.v_max)
    n_contact = 0
    for k in range(0, len(Q), 5):
        for j, v in zip(joints, Q[k]):
            d.qpos[sim.qadr[j]] = v
        mujoco.mj_forward(m, d)
        n_contact += int(d.ncon > 0)
    print(f"쇼 {show.duration:.1f} s: 관절 속도 최대 {vmax:.2f} rad/s, 한계 여유 {lim:.2f} rad, 접촉 있는 자세 {n_contact}개 (0 이어야 함)")
    for t0, t1, name, _ in show.segments:
        print(f"  {t0:5.1f}~{t1:5.1f} s  {name}")

    opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[0] = 1; opt.geomgroup[1] = 1; opt.geomgroup[2] = 1
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    W, H = 360, 420
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    m.vis.headlight.ambient[:] = 0.45; m.vis.headlight.diffuse[:] = 0.9; m.vis.headlight.specular[:] = 0.3
    r = mujoco.Renderer(m, H, W)

    def shot(az, el, text=""):
        cam.lookat[:] = [0.0, 0.0, 0.9]; cam.distance, cam.azimuth, cam.elevation = 2.2, az, el
        r.update_scene(d, cam, opt); img = Image.fromarray(r.render())
        if text:
            ImageDraw.Draw(img).text((8, 8), text, fill=(255, 255, 0))
        return img

    frames = []
    for k in range(0, len(Q), max(1, int(round(100 / ns.fps)))):
        for j, v in zip(joints, Q[k]):
            d.qpos[sim.qadr[j]] = v
        mujoco.mj_forward(m, d)
        row = Image.new("RGB", (2 * W, H))
        row.paste(shot(270, -8, f"t={k * 0.01:5.1f}s  {show.name_at(k * 0.01)}"), (0, 0))
        row.paste(shot(315, -15), (W, 0))
        frames.append(row)
    frames[0].save(ns.gif, save_all=True, append_images=frames[1:], duration=int(1000 / ns.fps), loop=0)
    print(f"GIF {ns.gif} ({len(frames)} 프레임, {ns.fps} fps; 앞 = 정면, 뒤 = 앞·왼쪽 45°)")
    if ns.keyframes:
        keys = []
        for t0, t1, name, _ in show.segments:
            if not name.startswith("→"):
                keys.append(((t0 + t1) / 2, name))
        strip = Image.new("RGB", (len(keys) * W, H))
        for i, (t, name) in enumerate(keys):
            k = min(len(Q) - 1, int(t / 0.01))
            for j, v in zip(joints, Q[k]):
                d.qpos[sim.qadr[j]] = v
            mujoco.mj_forward(m, d)
            strip.paste(shot(300, -12, f"{name} (t={t:.0f}s)"), (i * W, 0))
        strip.save(ns.keyframes); print(f"키프레임 {ns.keyframes}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
