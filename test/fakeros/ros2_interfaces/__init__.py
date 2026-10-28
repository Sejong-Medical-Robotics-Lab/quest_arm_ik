"""SH-Humanoid integration/ros2_interfaces 의 .msg/.srv 를 읽어 파이썬 클래스로 (고정 배열은 0 으로 채운 list)."""
import os, re
IFACE = os.environ.get("ROS2_INTERFACES_DIR") or os.path.join(os.environ.get("SH_REPO", ""), "integration", "ros2_interfaces")
if not os.path.isdir(os.path.join(IFACE, "msg")):
    raise ImportError(f"ros2_interfaces .msg 를 찾을 수 없음: {IFACE} — SH_REPO=/abs/SH-Humanoid 를 지정")
_DEFAULT = {"bool": False, "uint8": 0, "int8": 0, "uint16": 0, "int16": 0, "uint32": 0, "int32": 0, "uint64": 0, "int64": 0,
            "float32": 0.0, "float64": 0.0, "string": ""}


def _fields(text):
    consts, fields = {}, []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        if "=" in line and not line.startswith(("std_msgs", "builtin")):
            typ, rest = line.split(None, 1); name, val = [x.strip() for x in rest.split("=", 1)]
            consts[name] = float(val) if typ.startswith("float") else int(val)
            continue
        typ, name = line.split()
        m = re.match(r"([\w/]+)(\[(\d*)\])?$", typ)
        base, arr, n = m.group(1), m.group(2), m.group(3)
        fields.append((name, base, arr is not None, int(n) if n else None))
    return consts, fields


def _make(clsname, consts, fields):
    def __init__(self):
        for name, base, is_arr, n in fields:
            if base == "std_msgs/Header":
                from std_msgs.msg import Header; v = Header()
            elif is_arr:
                v = [_DEFAULT.get(base, 0)] * n if n else []
            else:
                v = _DEFAULT.get(base, 0)
            setattr(self, name, v)
    ns = {"__init__": __init__, "__slots__": [f[0] for f in fields]}
    ns.update(consts)
    return type(clsname, (), ns)


def load_msgs():
    out = {}
    for f in sorted(os.listdir(os.path.join(IFACE, "msg"))):
        if f.endswith(".msg"):
            c, fl = _fields(open(os.path.join(IFACE, "msg", f)).read()); out[f[:-4]] = _make(f[:-4], c, fl)
    return out


def load_srvs():
    out = {}
    for f in sorted(os.listdir(os.path.join(IFACE, "srv"))):
        if f.endswith(".srv"):
            req_t, res_t = open(os.path.join(IFACE, "srv", f)).read().split("---")
            name = f[:-4]
            Req = _make(name + "_Request", *_fields(req_t)); Res = _make(name + "_Response", *_fields(res_t))
            out[name] = type(name, (), {"Request": Req, "Response": Res})
    return out
