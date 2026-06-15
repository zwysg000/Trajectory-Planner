"""离散动作轨迹规划器的测试与使用示例。

运行方式::

    cd trajectory_planner_v4
    python test_planer.py

所有以 ``test_`` 开头的函数均为独立的单元测试。
:func:`main` 函数会运行全部测试并演示一个完整的工作流程。
"""

from __future__ import annotations

import math
import json

from planner import (
    ATOMIC_ACTIONS,
    DeltaAction,
    PlannerConfig,
    TrajectoryCandidate,
    TrajectoryPlanner,
)


# ============================================================================
#  基础规划功能测试
# ============================================================================


def test_atomic_actions() -> None:
    """验证 5 个原子动作已正确定义。"""
    assert len(ATOMIC_ACTIONS) == 5
    assert "forward" in ATOMIC_ACTIONS
    assert "left" in ATOMIC_ACTIONS
    assert "right" in ATOMIC_ACTIONS
    assert "up" in ATOMIC_ACTIONS
    assert "down" in ATOMIC_ACTIONS
    print("✅  ATOMIC_ACTIONS 定义正确")


def test_generate_with_deterministic_seed() -> None:
    """相同种子的两个规划器应生成完全相同的候选。"""
    expert = ("forward", "forward", "left", "forward", "forward")

    p1 = TrajectoryPlanner(PlannerConfig(seed=42, candidate_count=4))
    p2 = TrajectoryPlanner(PlannerConfig(seed=42, candidate_count=4))

    for c1, c2 in zip(p1.generate(expert), p2.generate(expert)):
        assert c1.actions == c2.actions

    print("✅  固定 seed 结果可复现")


def test_generate_output_format() -> None:
    """每条候选包含三个输出，其中 clean/noisy_delta 为单值。"""
    expert = ("forward", "forward", "left", "forward")

    planner = TrajectoryPlanner(PlannerConfig(candidate_count=5, seed=0))
    candidates = planner.generate(expert)

    assert len(candidates) == 5

    for c in candidates:
        assert isinstance(c, TrajectoryCandidate)
        assert isinstance(c.actions, tuple)
        assert len(c.actions) >= 1

        # clean_delta 和 noisy_delta 应为单个 DeltaAction（不是元组）
        assert isinstance(c.clean_delta, DeltaAction), (
            f"预期单值 DeltaAction, 实际 {type(c.clean_delta)}"
        )
        assert isinstance(c.noisy_delta, DeltaAction)

        for a in c.actions:
            assert a in ATOMIC_ACTIONS, f"非法动作: {a}"
        assert "source" in c.metadata

    print("✅  输出格式正确（actions 为序列，clean/noisy_delta 为单值）")


def test_candidate_length_range() -> None:
    """候选轨迹长度应在 [expert_len/2, expert_len*2] 范围内。"""
    expert = ("forward",) * 10
    expert_len = len(expert)
    low = max(1, expert_len // 2)
    high = expert_len * 2

    planner = TrajectoryPlanner(PlannerConfig(candidate_count=20, seed=7))
    candidates = planner.generate(expert)

    for c in candidates:
        assert low <= len(c.actions) <= high

    print(f"✅  候选长度在 [{low}, {high}] 范围内")


def test_mixed_strategies() -> None:
    """两种策略（纯随机和专家变异）都应出现在结果中。"""
    expert = ("forward",) * 8
    planner = TrajectoryPlanner(PlannerConfig(candidate_count=6, seed=5))
    candidates = planner.generate(expert)

    sources = [c.metadata["source"] for c in candidates]
    assert "pure_random" in sources
    assert "expert_mutation" in sources
    print(f"✅  混合策略验证通过: {sources}")


def test_expert_derived_ratio_all_random() -> None:
    """expert_derived_ratio=0 应全部为纯随机候选。"""
    planner = TrajectoryPlanner(PlannerConfig(
        expert_derived_ratio=0.0, candidate_count=4, seed=0,
    ))
    candidates = planner.generate(("forward", "forward", "forward"))
    sources = [c.metadata["source"] for c in candidates]
    assert all(s == "pure_random" for s in sources)
    print("✅  expert_derived_ratio=0，全部为纯随机")


def test_expert_derived_ratio_all_mutated() -> None:
    """expert_derived_ratio=1 应全部为专家变异候选。"""
    planner = TrajectoryPlanner(PlannerConfig(
        expert_derived_ratio=1.0, candidate_count=4, seed=0,
    ))
    candidates = planner.generate(("forward", "forward", "forward"))
    sources = [c.metadata["source"] for c in candidates]
    assert all(s == "expert_mutation" for s in sources)
    print("✅  expert_derived_ratio=1，全部为专家变异")


# ============================================================================
#  累计增量积分测试
# ============================================================================


def test_integrate_empty() -> None:
    """空序列的累计增量应全为 0。"""
    from planner.trajectory_planner import _integrate_actions
    result = _integrate_actions((), 0.1, 0.1, 15.0)
    assert result == DeltaAction(0, 0, 0, 0)
    print("✅  空序列积分正确")


def test_integrate_forward_only() -> None:
    """连续 forward 应沿正 X 方向累积。"""
    from planner.trajectory_planner import _integrate_actions
    result = _integrate_actions(("forward",) * 3, 0.1, 0.1, 15.0)
    assert abs(result.dx - 0.3) < 1e-10
    assert abs(result.dy) < 1e-10
    assert result.dz == 0.0
    assert result.dphi == 0.0
    print("✅  纯 forward 积分正确")


def test_integrate_left_then_forward() -> None:
    """左转后再前进，位移应有 X 和 Y 分量。"""
    from planner.trajectory_planner import _integrate_actions
    # left(15°) → forward(0.1m) → forward(0.1m)
    result = _integrate_actions(("left", "forward", "forward"), 0.1, 0.1, 15.0)
    expected_x = 0.2 * math.cos(math.radians(15))
    expected_y = 0.2 * math.sin(math.radians(15))
    assert abs(result.dx - expected_x) < 1e-10, f"dx: {result.dx} != {expected_x}"
    assert abs(result.dy - expected_y) < 1e-10
    assert result.dphi == 15.0
    print(f"✅  left+forward×2 积分正确: dx={result.dx:.4f}, dy={result.dy:.4f}, dphi={result.dphi}°")


def test_integrate_left_left_forward() -> None:
    """先左转 30° 再前进 0.5m。"""
    from planner.trajectory_planner import _integrate_actions
    result = _integrate_actions(("left", "left", "forward", "forward", "forward",
                                  "forward", "forward"), 0.1, 0.1, 15.0)
    expected_x = 0.5 * math.cos(math.radians(30))
    expected_y = 0.5 * math.sin(math.radians(30))
    assert abs(result.dx - expected_x) < 1e-10
    assert abs(result.dy - expected_y) < 1e-10
    assert result.dphi == 30.0
    print(f"✅  左转30°前进0.5m: dx={result.dx:.4f}, dy={result.dy:.4f} (预期 {expected_x:.4f}, {expected_y:.4f})")


def test_integrate_up_down() -> None:
    """up/down 只影响 dz。"""
    from planner.trajectory_planner import _integrate_actions
    result = _integrate_actions(("up", "up", "down"), 0.1, 0.2, 15.0)
    assert result.dz == 0.2  # 0.2 + 0.2 - 0.2 = 0.2
    assert result.dx == 0.0
    assert result.dy == 0.0
    assert result.dphi == 0.0
    print("✅  up/down 积分正确")


# ============================================================================
#  候选构建测试
# ============================================================================


def test_build_candidate_produces_single_delta() -> None:
    """_build_candidate 应生成单值 clean_delta / noisy_delta。"""
    planner = TrajectoryPlanner(PlannerConfig(horizontal_step=0.1))
    c = planner._build_candidate("t", ("forward",), "t")
    assert isinstance(c.clean_delta, DeltaAction)
    assert isinstance(c.noisy_delta, DeltaAction)
    print("✅  clean_delta 和 noisy_delta 均为单值")


def test_build_candidate_integration() -> None:
    """验证构建候选时积分结果正确。"""
    planner = TrajectoryPlanner(PlannerConfig(
        horizontal_step=0.5, vertical_step=0.2, yaw_step_deg=30.0,
    ))
    c = planner._build_candidate(
        "t",
        ("forward", "left", "forward", "up", "right", "down"),
        "t",
    )
    # forward(0.5, 0), left(30°), forward(沿30°方向0.5m), up(z+0.2),
    # right(30°→0°), down(z-0.2)
    # 总位移: dx=0.5+0.5*cos30°, dy=0+0.5*sin30°, dz=0.2-0.2=0, dphi=30-30=0
    expected_x = 0.5 + 0.5 * math.cos(math.radians(30))
    expected_y = 0.5 * math.sin(math.radians(30))
    assert abs(c.clean_delta.dx - expected_x) < 1e-10
    assert abs(c.clean_delta.dy - expected_y) < 1e-10
    assert c.clean_delta.dz == 0.0
    assert c.clean_delta.dphi == 0.0
    print(f"✅  复杂序列积分正确: dx={c.clean_delta.dx:.4f}")


# ============================================================================
#  噪声测试
# ============================================================================


def test_noise_zero_by_default() -> None:
    """所有噪声 sigma 为 0 时，noisy 应与 clean 完全一致。"""
    planner = TrajectoryPlanner(PlannerConfig(seed=0))
    c = planner._build_candidate("t", ("forward", "left", "forward"), "t")
    assert c.clean_delta == c.noisy_delta
    print("✅  默认噪声为 0，noisy == clean")


def test_noise_non_zero() -> None:
    """Sigma 不为 0 时，noisy 应与 clean 有差异。"""
    planner = TrajectoryPlanner(PlannerConfig(
        seed=0,
        noise_sigma_dx=0.01,
        noise_sigma_dy=0.005,
        noise_sigma_dz=0.01,
        noise_sigma_dphi=0.5,
    ))
    c = planner._build_candidate("t", ("forward",) * 10, "t")
    # 与 per-step 不同：噪声是一次性加在总增量上，
    # 不一定是噪声均值接近 0 了（因为只有 1 个样本），
    # 只需要确认有差异即可
    assert c.clean_delta != c.noisy_delta
    print("✅  噪声非零，noisy != clean")


def test_noise_deterministic() -> None:
    """相同种子和配置应产生相同的噪声值。"""
    cfg = PlannerConfig(
        seed=42,
        noise_sigma_dx=0.01, noise_sigma_dy=0.01,
        noise_sigma_dz=0.01, noise_sigma_dphi=0.5,
    )
    actions = ("forward", "forward")
    c1 = TrajectoryPlanner(cfg)._build_candidate("t", actions, "t")
    c2 = TrajectoryPlanner(cfg)._build_candidate("t", actions, "t")

    assert c1.clean_delta == c2.clean_delta
    assert c1.noisy_delta == c2.noisy_delta

    print("✅  噪声结果可复现")


def test_to_dict_includes_delta() -> None:
    """:meth:`TrajectoryCandidate.to_dict` 应包含 clean_delta 和 noisy_delta。"""
    planner = TrajectoryPlanner(PlannerConfig(seed=0))
    c = planner._build_candidate("test", ("forward", "left"), "test")
    d = c.to_dict()

    assert "clean_delta" in d
    assert "noisy_delta" in d
    assert "actions" in d
    # 确认是单值列表（4 个 float），不是列表的列表
    assert len(d["clean_delta"]) == 4
    assert len(d["noisy_delta"]) == 4

    print("✅  to_dict() 包含 clean_delta 和 noisy_delta（均为 4 元素列表）")


# ============================================================================
#  推荐配置与演示
# ============================================================================


def build_planner() -> TrajectoryPlanner:
    """返回一个使用推荐默认配置的 :class:`TrajectoryPlanner`。

    返回的实例使用:
        - 6 条随机/变异候选
        - 20% 替换率
        - 10 cm 水平和垂直步长，15° 偏航步长
        - 所有四个增量分量上带有小幅高斯噪声

    Returns:
        一个配置好的 :class:`TrajectoryPlanner`。
    """
    return TrajectoryPlanner(PlannerConfig(
        candidate_count=6,
        expert_derived_ratio=0.5,
        action_replacement_rate=0.2,
        seed=None,
        max_length=200,
        horizontal_step=0.1,
        vertical_step=0.1,
        yaw_step_deg=15.0,
        noise_sigma_dx=0.005,
        noise_sigma_dy=0.002,
        noise_sigma_dz=0.003,
        noise_sigma_dphi=0.5,
    ))


def main() -> None:
    """运行所有测试并演示完整工作流程。"""
    # ---- 单元测试 ----
    test_atomic_actions()
    test_generate_with_deterministic_seed()
    test_generate_output_format()
    test_candidate_length_range()
    test_mixed_strategies()
    test_expert_derived_ratio_all_random()
    test_expert_derived_ratio_all_mutated()
    test_integrate_empty()
    test_integrate_forward_only()
    test_integrate_left_then_forward()
    test_integrate_left_left_forward()
    test_integrate_up_down()
    test_build_candidate_produces_single_delta()
    test_build_candidate_integration()
    test_noise_zero_by_default()
    test_noise_non_zero()
    test_noise_deterministic()
    test_to_dict_includes_delta()

    # ---- 演示 ----
    print("\n" + "=" * 60)
    print("使用示例: 从专家轨迹生成候选（积分 + 噪声）")
    print("=" * 60)

    expert = (
        "left", "left",
        "forward", "forward", "forward", "forward", "forward",
        "right", "up",
        "forward", "forward",
    )

    print(f"\n专家轨迹 ({len(expert)} 步): {list(expert)}")

    planner = build_planner()
    candidates = planner.generate(expert)

    print(f"\n生成了 {len(candidates)} 条候选:\n")
    for c in candidates:
        preview = " → ".join(c.actions[:6])
        if len(c.actions) > 6:
            preview += f" ... (+{len(c.actions) - 6} 步)"
        cd = c.clean_delta
        nd = c.noisy_delta
        print(f"  [{c.name:22s}] {len(c.actions):3d} 步 | {preview}")
        print(f"            clean: [dx={cd.dx:+7.4f}, dy={cd.dy:+7.4f}, dz={cd.dz:+7.4f}, dphi={cd.dphi:+7.2f}]")
        print(f"            noisy: [dx={nd.dx:+7.4f}, dy={nd.dy:+7.4f}, dz={nd.dz:+7.4f}, dphi={nd.dphi:+7.2f}]")

    # to_dict 输出
    print("\nto_dict() 输出（第一条候选）:")
    print(json.dumps(candidates[0].to_dict(), indent=2, ensure_ascii=False))

    print("\n✅  所有测试通过!")


if __name__ == "__main__":
    main()
