"""离散动作轨迹规划器。

基于 5 个原子动作（forward / left / right / up / down）生成候选轨迹。

典型用法::

    from planner import PlannerConfig, TrajectoryPlanner

    planner = TrajectoryPlanner(PlannerConfig(candidate_count=6))
    candidates = planner.generate(("left", "forward", "forward"))
"""

from .trajectory_planner import PlannerConfig, TrajectoryPlanner
from .types import ATOMIC_ACTIONS, DeltaAction, TrajectoryCandidate

__all__ = [
    "ATOMIC_ACTIONS",
    "DeltaAction",
    "PlannerConfig",
    "TrajectoryCandidate",
    "TrajectoryPlanner",
]
