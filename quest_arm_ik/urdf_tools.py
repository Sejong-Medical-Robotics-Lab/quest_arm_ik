"""URDF 전처리 유틸 (ROS 비의존): xacro 전개, package:// 해석, 링크 충돌형상을 기본도형으로 교체.

Pinocchio(IK)와 MuJoCo(시뮬)가 같은 전처리 결과를 쓰므로 두 쪽의 기하가 항상 일치한다.
"""
from __future__ import annotations

import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence

_TMP_ROOT = os.path.join(tempfile.gettempdir(), "quest_arm_ik")


def _tmp_path(name: str) -> str:
    os.makedirs(_TMP_ROOT, exist_ok=True)
    return os.path.join(_TMP_ROOT, name)


def find_package_dir(pkg: str, package_dirs: Sequence[str]) -> Optional[str]:
    """package_dirs 의 디렉터리 자체(이름이 pkg), 바로 아래의 pkg 폴더, 또는 ament 인덱스에서 찾는다."""
    for r in [os.path.abspath(d) for d in package_dirs if d]:
        if os.path.basename(r) == pkg and os.path.isdir(r):
            return r
        cand = os.path.join(r, pkg)
        if os.path.isdir(cand):
            return cand
        # 한 단계 아래까지 (ros_ws/src/<repo>/<pkg>)
        for sub in sorted(os.listdir(r)) if os.path.isdir(r) else []:
            cand = os.path.join(r, sub, pkg)
            if os.path.isdir(cand):
                return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001
        return None


def resolve_package_uris(text: str, package_dirs: Sequence[str], base_dir: str = "") -> str:
    """package://PKG/rest → 절대경로. 못 찾으면 FileNotFoundError."""
    if "package://" not in text:
        return text
    dirs = list(package_dirs) + ([base_dir] if base_dir else [])

    def repl(m):
        pkg, rest = m.group(1), m.group(2)
        d = find_package_dir(pkg, dirs)
        if d is None:
            raise FileNotFoundError(f"package://{pkg} 를 package_dirs {dirs} 에서 찾지 못함")
        return os.path.join(d, rest)

    return re.sub(r"package://([^/\"']+)/([^\"']+)", repl, text)


def parse_xacro_args(spec) -> Dict[str, str]:
    """"hand:=box dex3_fingers:=revolute" / "hand=box,dex3_curl=0.2" / dict → {이름: 값}."""
    if not spec:
        return {}
    if isinstance(spec, dict):
        return {str(k): str(v) for k, v in spec.items()}
    out: Dict[str, str] = {}
    for tok in re.split(r"[,\s]+", str(spec).strip()):
        if not tok:
            continue
        if ":=" in tok:
            k, v = tok.split(":=", 1)
        elif "=" in tok:
            k, v = tok.split("=", 1)
        else:
            raise ValueError(f"xacro_args 항목 '{tok}' 는 name:=value 꼴이어야 함")
        out[k.strip()] = v.strip()
    return out


def xacro_to_urdf_text(xacro_path: str, package_dirs: Sequence[str] = (), mappings: Optional[Dict[str, str]] = None) -> str:
    """xacro 파일을 전개해 URDF 문자열로. $(find pkg) 는 (include 된 파일 안의 것까지) package_dirs/ament 로 해석 (ROS 없이도 동작).
    mappings: xacro 인자 (예 {"hand": "box"}) — 명령행 xacro 의 name:=value 와 같다."""
    import xacro  # pip install xacro
    import xacro.substitution_args as sa
    dirs = list(package_dirs) + [os.path.dirname(os.path.abspath(xacro_path))]
    orig_find = sa._eval_find

    def _find(pkg):
        d = find_package_dir(pkg, dirs)
        if d is not None:
            return d
        try:
            return orig_find(pkg)
        except Exception as e:  # noqa: BLE001
            raise FileNotFoundError(f"$(find {pkg}) 해석 실패 — package_dirs 지정 필요 ({e})")

    sa._eval_find = _find
    try:
        return xacro.process_file(xacro_path, mappings=dict(mappings or {})).toprettyxml(indent="  ")
    finally:
        sa._eval_find = orig_find


def _origin(xyz: Sequence[float], rpy: Sequence[float]) -> ET.Element:
    el = ET.Element("origin")
    el.set("xyz", " ".join(f"{float(v):.6g}" for v in xyz))
    el.set("rpy", " ".join(f"{float(v):.6g}" for v in rpy))
    return el


def _geometry(shape: Dict) -> ET.Element:
    geo = ET.Element("geometry")
    t = str(shape.get("type", "box")).lower()
    size = [float(v) for v in shape.get("size", [])]
    if t == "box":
        if len(size) != 3:
            raise ValueError("box size 는 [x y z] 전체 길이")
        ET.SubElement(geo, "box", size=" ".join(f"{v:.6g}" for v in size))
    elif t == "cylinder":
        if len(size) != 2:
            raise ValueError("cylinder size 는 [radius length] (z축 방향)")
        ET.SubElement(geo, "cylinder", radius=f"{size[0]:.6g}", length=f"{size[1]:.6g}")
    elif t == "sphere":
        if len(size) != 1:
            raise ValueError("sphere size 는 [radius]")
        ET.SubElement(geo, "sphere", radius=f"{size[0]:.6g}")
    else:
        raise ValueError(f"지원하지 않는 shape type: {t} (box | cylinder | sphere)")
    return geo


def apply_collision_shapes(text: str, shapes: Sequence[Dict], replace_visual: bool = True) -> str:
    """shapes: [{link, type, size, xyz, rpy}, ...]. 같은 link 의 항목들을 모아 그 링크의 <collision>(과 <visual>)을
    기본도형으로 교체한다. 거대한 STL(스탠드·몸통)을 기둥/상자 몇 개로 바꿔 IK 자기충돌과 MuJoCo 양쪽에서 쓰기 위함."""
    if not shapes:
        return text
    root = ET.fromstring(text)
    by_link: Dict[str, List[Dict]] = {}
    for s in shapes:
        by_link.setdefault(str(s["link"]), []).append(s)
    links = {l.get("name"): l for l in root.findall("link")}
    for name, items in by_link.items():
        if name not in links:
            raise ValueError(f"collision_shapes: URDF 에 링크 '{name}' 없음")
        link = links[name]
        for tag in ("collision",) + (("visual",) if replace_visual else ()):
            for el in list(link.findall(tag)):
                link.remove(el)
        for s in items:
            xyz, rpy = s.get("xyz", [0, 0, 0]), s.get("rpy", [0, 0, 0])
            for tag in ("collision",) + (("visual",) if replace_visual else ()):
                el = ET.SubElement(link, tag)
                el.append(_origin(xyz, rpy))
                el.append(_geometry(s))
    return ET.tostring(root, encoding="unicode")


def prepare_urdf(model_path: str, package_dirs: Sequence[str] = (), collision_shapes: Sequence[Dict] = (),
                 resolve_packages: bool = True, tag: str = "", xacro_args=None) -> str:
    """.urdf/.xacro 를 읽어 전처리한 URDF 파일 경로를 돌려준다 (변경이 없으면 원본 경로 그대로).
    1) xacro 전개 (xacro_args: "hand:=box …" 또는 dict), 2) collision_shapes 적용, 3) package:// → 절대경로."""
    ext = os.path.splitext(model_path)[1].lower()
    if ext == ".xacro":
        text = xacro_to_urdf_text(model_path, package_dirs, parse_xacro_args(xacro_args))
        changed = True
    else:
        text = open(model_path, encoding="utf-8").read()
        changed = False
    if collision_shapes:
        text = apply_collision_shapes(text, collision_shapes)
        changed = True
    if resolve_packages and "package://" in text:
        text = resolve_package_uris(text, package_dirs, os.path.dirname(os.path.abspath(model_path)))
        changed = True
    if not changed:
        return model_path
    base = os.path.splitext(os.path.basename(model_path))[0]
    out = _tmp_path(f"{base}{('.' + tag) if tag else ''}.prepared.urdf")
    open(out, "w", encoding="utf-8").write(text)
    return out


def hand_variant(names: Sequence[str]) -> str:
    """링크/바디 이름으로 손 종류를 알려준다: 'dex3 (손가락 N관절)' | 'box' | 'none'. 시작 로그에 찍어 어떤 모델이 실제로 로드됐는지 바로 보이게."""
    names = list(names)
    if any("_hand_dex3_frame" in n or "_hand_palm_link" in n for n in names):
        n_f = sum(1 for n in names if "_hand_" in n and n.endswith("_joint"))
        return f"dex3 (손가락 {n_f}관절)" if n_f else "dex3 (손가락 고정)"
    if any(n.endswith("_hand_link") for n in names):
        return "box"
    return "none"


def rpy_to_quat_wxyz(rpy: Sequence[float]) -> List[float]:
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
