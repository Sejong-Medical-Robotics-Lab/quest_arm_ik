"""손 쥐기(grasp) 매핑 — ROS 비의존.

컨트롤러 트리거 값 s∈[0,1] → 손가락 관절 목표  q_j = open_j + s·(closed_j − open_j).
Unitree G1 Dex3-1 (3지, 손당 7관절) 의 열림/쥠 각은 Menagerie g1_with_hands.xml 의 관절 범위 안에서 잡았다:
  왼손: 검지·중지 굽힘은 음(−), 엄지 굽힘은 양(+) / 오른손은 부호 반대.  thumb_0(엄지 회전) 은 0 으로 둔다 — 벌리면(±0.5) 쥘 때 엄지가 검지·중지와 겹친다 (g1_with_hands 충돌 모델로 확인).
삼현 2세대 + Dex3 (dex3_hand.xacro, dex3_fingers:=revolute) 와 G1 g1_with_hands.xml 이 같은 관절 이름을 쓴다.
"""
from __future__ import annotations

from typing import Dict, Mapping, Optional, Sequence

SIDES = ("left", "right")

# {side: {joint: [open, closed]}} — open 은 완전히 편 0 이 아니라 살짝 오므린 자연스러운 자세(엄지가 옆으로 뻗지 않음)
DEX3_GRASP: Dict[str, Dict[str, Sequence[float]]] = {
    "left": {
        "left_hand_thumb_0_joint": [0.0, 0.0],
        "left_hand_thumb_1_joint": [0.6, 1.0],
        "left_hand_thumb_2_joint": [0.4, 1.2],
        "left_hand_index_0_joint": [-0.2, -1.3],
        "left_hand_index_1_joint": [-0.3, -1.5],
        "left_hand_middle_0_joint": [-0.2, -1.3],
        "left_hand_middle_1_joint": [-0.3, -1.5],
    },
    "right": {
        "right_hand_thumb_0_joint": [0.0, 0.0],
        "right_hand_thumb_1_joint": [-0.6, -1.0],
        "right_hand_thumb_2_joint": [-0.4, -1.2],
        "right_hand_index_0_joint": [0.2, 1.3],
        "right_hand_index_1_joint": [0.3, 1.5],
        "right_hand_middle_0_joint": [0.2, 1.3],
        "right_hand_middle_1_joint": [0.3, 1.5],
    },
}


def clip01(s: float) -> float:
    return 0.0 if s < 0.0 else 1.0 if s > 1.0 else float(s)


def grasp_targets(spec: Mapping[str, Mapping[str, Sequence[float]]], grasp: Mapping[str, float]) -> Dict[str, float]:
    """spec={side: {joint: [open, closed]}}, grasp={side: s} → {joint: angle}. spec 에 없는 side 는 무시."""
    out: Dict[str, float] = {}
    for side, joints in spec.items():
        s = clip01(float(grasp.get(side, 0.0)))
        for j, (q_open, q_closed) in joints.items():
            out[j] = float(q_open) + s * (float(q_closed) - float(q_open))
    return out


class GraspMapper:
    """트리거 값을 1차 저역통과(시정수 tau)로 부드럽게 하고 관절 목표로 바꾼다."""

    def __init__(self, spec: Optional[Mapping[str, Mapping[str, Sequence[float]]]] = None, tau: float = 0.1):
        self.spec = dict(spec if spec is not None else DEX3_GRASP)
        self.tau = max(float(tau), 0.0)
        self.target: Dict[str, float] = {s: 0.0 for s in self.spec}
        self.value: Dict[str, float] = {s: 0.0 for s in self.spec}
        self.received: Dict[str, bool] = {s: False for s in self.spec}

    def set(self, side: str, s: float) -> None:
        if side in self.target:
            self.target[side] = clip01(s)
            self.received[side] = True

    def update(self, dt: float) -> Dict[str, float]:
        a = 1.0 if self.tau <= 0.0 else min(1.0, dt / self.tau)
        for side in self.value:
            self.value[side] += a * (self.target[side] - self.value[side])
        return grasp_targets(self.spec, self.value)

    @property
    def joint_names(self):
        return [j for joints in self.spec.values() for j in joints]
