"""离散动作轨迹规划器（含增量转换与噪声）。

提供核心的 :class:`TrajectoryPlanner` 类。给定一条*专家轨迹*（动作名称字符串
序列），生成 K 条候选轨迹。生成策略混合两种方式：

    1. **纯随机** — 每步均匀采样 5 个原子动作之一；长度在
       ``[expert_len/2, expert_len*2]`` 范围内随机。
    2. **专家变异** — 以专家轨迹为模板，随机调整长度（插入/删除）并
       以一定概率替换每个动作。

每条候选包含三个并列输出：

    - ``actions`` — 离散动作名称序列（字符串）
    - ``clean_deltas`` — 通过查询预定义映射表得到的无噪声 :class:`DeltaAction` 值
    - ``noisy_deltas`` — 在 clean_deltas 的四个分量上分别添加可配置的独立
      高斯噪声后的结果
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from .types import ATOMIC_ACTIONS, DeltaAction, TrajectoryCandidate


# ---------------------------------------------------------------------------
#  内部辅助函数
# ---------------------------------------------------------------------------


def _action_to_delta_map(
    horizontal_step: float,
    vertical_step: float,
    yaw_step_deg: float,
) -> dict[str, DeltaAction]:
    """构建原子动作 → :class:`DeltaAction` 的映射表。

    Args:
        horizontal_step: forward 动作的前进步长（米）。
        vertical_step: up/down 动作的垂直步长（米）。
        yaw_step_deg: left/right 动作的偏航步长（度）。

    Returns:
        将 5 个原子动作名称映射到对应 :class:`DeltaAction` 的字典。
    """
    return {
        "forward": DeltaAction(horizontal_step, 0.0, 0.0, 0.0),
        "left": DeltaAction(0.0, 0.0, 0.0, yaw_step_deg),
        "right": DeltaAction(0.0, 0.0, 0.0, -yaw_step_deg),
        "up": DeltaAction(0.0, 0.0, vertical_step, 0.0),
        "down": DeltaAction(0.0, 0.0, -vertical_step, 0.0),
    }


# ---------------------------------------------------------------------------
#  配置类
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlannerConfig:
    """:class:`TrajectoryPlanner` 的配置参数。

    参数分为三组：

    **候选生成** 控制产生多少条候选以及变异强度::

        candidate_count = 5
        action_replacement_rate = 0.2
        seed = None
        max_length = 200

    **步长参数** 定义了每个原子动作的物理含义::

        horizontal_step = 0.1    # 每次 ``forward`` 前进的米数
        vertical_step   = 0.1    # 每次 ``up`` / ``down`` 升降的米数
        yaw_step_deg    = 15.0   # 每次 ``left`` / ``right`` 旋转的角度

    **噪声参数** 控制加到四个增量分量上的独立高斯噪声。默认全为 0（无噪声）::

        noise_sigma_dx   = 0.0   # 米
        noise_sigma_dy   = 0.0   # 米
        noise_sigma_dz   = 0.0   # 米
        noise_sigma_dphi = 0.0   # 度
    """

    # -- 候选生成 ------------------------------------------------------------

    #: 生成的候选轨迹数量。
    candidate_count: int = 5

    #: 候选轨迹中从专家路径派生（变异）的比例。
    #: ``0.0`` = 全部纯随机；``1.0`` = 全部专家变异；``0.5`` = 各一半。
    expert_derived_ratio: float = 0.5

    #: 专家变异时每步的替换概率。``0.0`` = 不替换；``1.0`` = 全部替换。
    action_replacement_rate: float = 0.2

    #: 随机种子，用于结果复现。``None`` = 每次不同。
    seed: int | None = None

    #: 候选轨迹的最大步数上限。
    max_length: int = 200

    # -- 步长参数 ------------------------------------------------------------

    #: 一次 ``forward`` 动作前进的距离（米）。
    horizontal_step: float = 0.1

    #: 一次 ``up`` 或 ``down`` 动作升降的距离（米）。
    vertical_step: float = 0.1

    #: 一次 ``left`` 或 ``right`` 动作偏航的角度（度）。
    yaw_step_deg: float = 15.0

    # -- 噪声参数 ------------------------------------------------------------

    #: dx 分量的高斯噪声标准差（米）。
    noise_sigma_dx: float = 0.0

    #: dy 分量的高斯噪声标准差（米）。
    noise_sigma_dy: float = 0.0

    #: dz 分量的高斯噪声标准差（米）。
    noise_sigma_dz: float = 0.0

    #: dphi 分量的高斯噪声标准差（度）。
    noise_sigma_dphi: float = 0.0


# ---------------------------------------------------------------------------
#  主规划器类
# ---------------------------------------------------------------------------


@dataclass
class TrajectoryPlanner:
    """根据专家动作序列生成候选轨迹。

    用法::

        >>> planner = TrajectoryPlanner(PlannerConfig(candidate_count=4))
        >>> candidates = planner.generate(("left", "forward", "forward"))
        >>> len(candidates)
        4
        >>> candidates[0].actions
        ('right', 'up', 'forward', ...)
    """

    #: 规划器配置（构造后不可变）。
    config: PlannerConfig = field(default_factory=PlannerConfig)

    # -- 内部属性（在 __post_init__ 中初始化）--------------------------------
    _rng: np.random.Generator = field(init=False, repr=False)
    _action_map: dict[str, DeltaAction] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """验证配置参数，初始化随机数生成器和动作映射表。"""
        if self.config.candidate_count < 1:
            raise ValueError("candidate_count 必须 >= 1")
        if not (0.0 <= self.config.action_replacement_rate <= 1.0):
            raise ValueError("action_replacement_rate 必须在 [0, 1] 范围内")
        if not (0.0 <= self.config.expert_derived_ratio <= 1.0):
            raise ValueError("expert_derived_ratio 必须在 [0, 1] 范围内")
        self._rng = np.random.default_rng(self.config.seed)
        self._action_map = _action_to_delta_map(
            horizontal_step=self.config.horizontal_step,
            vertical_step=self.config.vertical_step,
            yaw_step_deg=self.config.yaw_step_deg,
        )

    # ==============================
    #  公开 API
    # ==============================

    def generate(
        self,
        expert_actions: Sequence[str],
    ) -> list[TrajectoryCandidate]:
        """根据专家动作序列生成候选轨迹列表。

        返回的列表包含 ``config.candidate_count`` 条候选。
        纯随机与专家变异两种策略的比例由 ``config.expert_derived_ratio`` 控制。
        每条候选都携带三个并列输出（``actions``、``clean_deltas``、``noisy_deltas``）。

        Args:
            expert_actions:
                专家（真实）轨迹，由动作名称字符串组成的序列。
                每个元素必须是 :data:`~planner.types.ATOMIC_ACTIONS` 之一。
                例如：``("forward", "left", "forward", "up")``。

        Returns:
            包含每条候选的 :class:`~planner.types.TrajectoryCandidate` 列表。

        Raises:
            ValueError: 如果 ``expert_actions`` 为空或包含非法动作名称。
        """
        if len(expert_actions) < 1:
            raise ValueError("expert_actions 不能为空")
        self._validate_actions(expert_actions)

        expert_len = len(expert_actions)
        candidates: list[TrajectoryCandidate] = []

        mutation_count = int(round(self.config.candidate_count * self.config.expert_derived_ratio))
        pure_random_count = self.config.candidate_count - mutation_count

        # 策略 1：纯随机
        for i in range(pure_random_count):
            length = self._random_length(expert_len)
            actions = self._random_sequence(length)
            candidates.append(
                self._build_candidate(
                    name=f"random_trajectory_{i + 1}",
                    actions=actions,
                    source="pure_random",
                )
            )

        # 策略 2：专家变异
        for i in range(mutation_count):
            length = self._random_length(expert_len)
            actions = self._mutate_from_expert(expert_actions, length)
            candidates.append(
                self._build_candidate(
                    name=f"mutated_trajectory_{i + 1}",
                    actions=actions,
                    source="expert_mutation",
                    extra_meta={"expert_length": expert_len},
                )
            )

        return candidates

    # ==============================
    #  候选构建（动作 → 增量 → 加噪声）
    # ==============================

    def _build_candidate(
        self,
        name: str,
        actions: tuple[str, ...],
        source: str,
        extra_meta: dict | None = None,
    ) -> TrajectoryCandidate:
        """从动作名称序列构建完整的 :class:`TrajectoryCandidate`。

        该方法依次执行动作→增量映射和根据配置添加高斯噪声。

        Args:
            name: 候选名称/标识。
            actions: 动作名称序列。
            source: ``metadata["source"]`` 的值。
            extra_meta: 额外的元数据键值对，会合并到 metadata 中。

        Returns:
            已填充 ``clean_deltas`` 和 ``noisy_deltas`` 字段的
            :class:`TrajectoryCandidate`。
        """
        clean_deltas = tuple(self._action_map[a] for a in actions)
        noisy_deltas = tuple(self._add_noise(d) for d in clean_deltas)

        meta: dict = {"source": source}
        if extra_meta:
            meta.update(extra_meta)

        return TrajectoryCandidate(
            name=name,
            actions=actions,
            clean_deltas=clean_deltas,
            noisy_deltas=noisy_deltas,
            metadata=meta,
        )

    def _add_noise(self, delta: DeltaAction) -> DeltaAction:
        """对 :class:`DeltaAction` 的四个分量分别添加独立高斯噪声。

        仅在对应的 ``noise_sigma_*`` 配置参数非零时才会添加噪声。

        Args:
            delta: 待加噪声的干净 :class:`DeltaAction`。

        Returns:
            添加噪声后的新 :class:`DeltaAction`（若所有 sigma 为 0 则返回原值）。
        """
        cfg = self.config
        if all(
            v == 0.0
            for v in (
                cfg.noise_sigma_dx,
                cfg.noise_sigma_dy,
                cfg.noise_sigma_dz,
                cfg.noise_sigma_dphi,
            )
        ):
            return delta

        noise = self._rng.normal(
            loc=[0.0, 0.0, 0.0, 0.0],
            scale=[
                cfg.noise_sigma_dx,
                cfg.noise_sigma_dy,
                cfg.noise_sigma_dz,
                cfg.noise_sigma_dphi,
            ],
        )
        return DeltaAction(
            dx=delta.dx + float(noise[0]),
            dy=delta.dy + float(noise[1]),
            dz=delta.dz + float(noise[2]),
            dphi=delta.dphi + float(noise[3]),
        )

    # ==============================
    #  内部辅助方法
    # ==============================

    def _random_length(self, expert_len: int) -> int:
        """在 ``[expert_len/2, expert_len*2]`` 范围内随机采样一个长度。

        Args:
            expert_len: 原始专家轨迹的长度。

        Returns:
            被限制在 ``[1, config.max_length]`` 范围内的整数长度。
        """
        low = max(1, int(expert_len / 2))
        high = min(self.config.max_length, expert_len * 2)
        if high < low:
            high = low
        return int(self._rng.integers(low, high + 1))

    def _random_sequence(self, length: int) -> tuple[str, ...]:
        """生成长度为 *length* 的均匀随机动作序列。

        Args:
            length: 所需的步数。

        Returns:
            动作名称字符串的元组。
        """
        indices = self._rng.integers(0, len(ATOMIC_ACTIONS), size=length)
        return tuple(ATOMIC_ACTIONS[i] for i in indices)

    def _mutate_from_expert(
        self,
        expert_actions: Sequence[str],
        target_length: int,
    ) -> tuple[str, ...]:
        """以专家轨迹为模板生成变异序列。

        先通过随机删除和插入将专家序列调整到 *target_length*，
        然后以 ``config.action_replacement_rate`` 的概率替换每个元素。

        Args:
            expert_actions: 原始专家轨迹。
            target_length: 返回序列的目标长度。

        Returns:
            长度为 *target_length* 的变异动作名称元组。
        """
        actions = list(expert_actions)

        # 若过长则随机删除
        while len(actions) > target_length:
            del_idx = int(self._rng.integers(0, len(actions)))
            del actions[del_idx]

        # 若过短则随机插入
        while len(actions) < target_length:
            ins_idx = int(self._rng.integers(0, len(actions) + 1))
            new_action = ATOMIC_ACTIONS[int(self._rng.integers(0, len(ATOMIC_ACTIONS)))]
            actions.insert(ins_idx, new_action)

        # 按概率替换
        for i in range(len(actions)):
            if self._rng.random() < self.config.action_replacement_rate:
                actions[i] = ATOMIC_ACTIONS[
                    int(self._rng.integers(0, len(ATOMIC_ACTIONS)))
                ]

        return tuple(actions)

    def _validate_actions(self, actions: Sequence[str]) -> None:
        """检查序列中每个动作是否为已知的原子动作。

        Args:
            actions: 待验证的动作序列。

        Raises:
            ValueError: 如果遇到未知的动作名称。
        """
        valid_set = set(ATOMIC_ACTIONS)
        for action in actions:
            if action not in valid_set:
                raise ValueError(f"未知动作: {action!r}，合法动作: {ATOMIC_ACTIONS}")
