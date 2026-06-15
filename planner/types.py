"""轨迹规划器的核心数据结构。

本模块定义了规划器使用的全部数据类型：

    - ``ATOMIC_ACTIONS`` — 5 个原子动作名称
    - ``DeltaAction`` — 连续四参数增量 (dx, dy, dz, dphi)
    - ``TrajectoryCandidate`` — 一条候选轨迹，包含离散动作及对应的连续增量
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


#: 五个原子动作的名称。
#:
#: - ``"forward"`` — 前进 ``horizontal_step`` 米
#: - ``"left"`` — 左偏航 ``yaw_step_deg`` 度
#: - ``"right"`` — 右偏航 ``yaw_step_deg`` 度
#: - ``"up"`` — 上升 ``vertical_step`` 米
#: - ``"down"`` — 下降 ``vertical_step`` 米
ATOMIC_ACTIONS: tuple[str, ...] = ("forward", "left", "right", "up", "down")


@dataclass(frozen=True)
class DeltaAction:
    """由离散原子动作转换得到的连续四参数增量。

    单位:
        - ``dx``, ``dy``, ``dz`` — 米
        - ``dphi`` — 度（正数 = 左偏航 / 逆时针）

    示例::

        >>> action = DeltaAction(dx=0.1, dphi=15.0)
        >>> action.as_list()
        [0.1, 0.0, 0.0, 15.0]
    """

    #: 体轴坐标系下的前进位移（米）。
    dx: float = 0.0

    #: 体轴坐标系下的侧向位移（米）。
    #: 在 5 个原子动作中恒为 0，因为没有纯左右平移动作。
    dy: float = 0.0

    #: 垂直位移（米）。
    dz: float = 0.0

    #: 偏航角变化（度）；正数 = 左转。
    dphi: float = 0.0

    def as_list(self) -> list[float]:
        """返回 ``[dx, dy, dz, dphi]`` 列表形式的四参数。"""
        return [self.dx, self.dy, self.dz, self.dphi]


@dataclass(frozen=True)
class TrajectoryCandidate:
    """一条候选轨迹，包含三个并列输出。

    三个核心字段长度保持一致：

        - ``actions`` —  (A) 离散动作名称序列
        - ``clean_deltas`` —  (B) 无噪声的 :class:`DeltaAction` 序列
        - ``noisy_deltas`` —  (C) 添加高斯噪声后的 :class:`DeltaAction` 序列

    示例::

        >>> c = TrajectoryCandidate(
        ...     name="example",
        ...     actions=("forward", "left"),
        ...     clean_deltas=(DeltaAction(dx=0.1), DeltaAction(dphi=15.0)),
        ...     noisy_deltas=(DeltaAction(dx=0.103), DeltaAction(dphi=14.8)),
        ... )
        >>> c.to_dict()
        {'name': 'example', ...}
    """

    #: 候选轨迹名称，例如 ``"random_trajectory_1"``。
    name: str

    #: (A) 离散动作名称序列。每个元素来自 :data:`ATOMIC_ACTIONS`。
    actions: tuple[str, ...]

    #: (B) 无噪声的连续增量序列，与动作一一对应。
    clean_deltas: tuple[DeltaAction, ...] = field(default_factory=tuple)

    #: (C) 添加噪声后的连续增量序列，与动作一一对应。
    noisy_deltas: tuple[DeltaAction, ...] = field(default_factory=tuple)

    #: 任意元数据（例如 ``{"source": "pure_random"}``）。
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_list(self) -> list[str]:
        """将动作名称序列转为普通 Python 列表。"""
        return list(self.actions)

    def to_dict(self) -> dict[str, Any]:
        """将候选轨迹序列化为 JSON 兼容的字典。

        返回的字典包含键 ``name``、``actions``、``clean_deltas``、
        ``noisy_deltas`` 和 ``metadata``。增量数据以 ``[dx, dy, dz, dphi]``
        列表形式存储。
        """
        return {
            "name": self.name,
            "actions": list(self.actions),
            "clean_deltas": [d.as_list() for d in self.clean_deltas],
            "noisy_deltas": [d.as_list() for d in self.noisy_deltas],
            "metadata": dict(self.metadata),
        }

    def __len__(self) -> int:
        """返回轨迹的步数。"""
        return len(self.actions)
