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
    """由离散原子动作积分得到的累计连续增量。

    表示从起始状态到执行完一系列动作之后的总位移和总偏航。
    坐标采用**世界坐标系**（即相对于起点的绝对位移）。

    单位:
        - ``dx``, ``dy``, ``dz`` — 米
        - ``dphi`` — 度（正数 = 左偏航 / 逆时针）

    示例::

        >>> action = DeltaAction(dx=0.26, dy=0.15, dphi=30.0)
        >>> action.as_list()
        [0.26, 0.15, 0.0, 30.0]
    """

    #: 世界坐标系下的总 X 方向位移（米）。
    dx: float = 0.0

    #: 世界坐标系下的总 Y 方向位移（米）。
    dy: float = 0.0

    #: 总垂直位移（米）。
    dz: float = 0.0

    #: 总偏航变化（度）；正数 = 左转。
    dphi: float = 0.0

    def as_list(self) -> list[float]:
        """返回 ``[dx, dy, dz, dphi]`` 列表形式的四参数。"""
        return [self.dx, self.dy, self.dz, self.dphi]


@dataclass(frozen=True)
class TrajectoryCandidate:
    """一条候选轨迹，包含离散动作序列及其积分得到的累计连续增量。

    三个核心字段：

        - ``actions`` —  (A) 离散动作名称序列
        - ``clean_delta`` —  (B) 无噪声的累计增量（单个 :class:`DeltaAction`）
        - ``noisy_delta`` —  (C) 添加高斯噪声后的累计增量（单个 :class:`DeltaAction`）

    clean_delta 和 noisy_delta 均为单值（不是序列），表示执行完所有动作后
    相对于起点的总位移和总偏航。

    示例::

        >>> c = TrajectoryCandidate(
        ...     name="example",
        ...     actions=("forward", "forward"),
        ...     clean_delta=DeltaAction(dx=0.2),
        ...     noisy_delta=DeltaAction(dx=0.203),
        ... )
        >>> c.to_dict()
        {'name': 'example', ...}
    """

    #: 候选轨迹名称，例如 ``"random_trajectory_1"``。
    name: str

    #: (A) 离散动作名称序列。每个元素来自 :data:`ATOMIC_ACTIONS`。
    actions: tuple[str, ...]

    #: (B) 无噪声的累计增量（单值），从起点到终点的总位移和总偏航。
    clean_delta: DeltaAction | None = None

    #: (C) 添加噪声后的累计增量（单值）。
    noisy_delta: DeltaAction | None = None

    #: 任意元数据（例如 ``{"source": "pure_random"}``）。
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_list(self) -> list[str]:
        """将动作名称序列转为普通 Python 列表。"""
        return list(self.actions)

    def to_dict(self) -> dict[str, Any]:
        """将候选轨迹序列化为 JSON 兼容的字典。

        返回的字典包含键 ``name``、``actions``、``clean_delta``、
        ``noisy_delta`` 和 ``metadata``。
        """
        result: dict[str, Any] = {
            "name": self.name,
            "actions": list(self.actions),
        }
        if self.clean_delta is not None:
            result["clean_delta"] = self.clean_delta.as_list()
        if self.noisy_delta is not None:
            result["noisy_delta"] = self.noisy_delta.as_list()
        result["metadata"] = dict(self.metadata)
        return result

    def __len__(self) -> int:
        """返回轨迹的步数。"""
        return len(self.actions)
