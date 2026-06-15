"""离散动作轨迹规划器的测试与使用示例。

运行方式::

    cd trajectory_planner_v4
    python test_planer.py

所有以 ``test_`` 开头的函数均为独立的单元测试。
:func:`main` 函数会运行全部测试并演示一个完整的工作流程。
"""

from __future__ import annotations

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
    """每条候选包含三个并列输出且长度一致。"""
    expert = ("forward", "forward", "left", "forward")

    planner = TrajectoryPlanner(PlannerConfig(candidate_count=5, seed=0))
    candidates = planner.generate(expert)

    assert len(candidates) == 5

    for c in candidates:
        assert isinstance(c, TrajectoryCandidate)
        assert isinstance(c.actions, tuple)
        assert isinstance(c.clean_deltas, tuple)
        assert isinstance(c.noisy_deltas, tuple)
        assert len(c.actions) == len(c.clean_deltas) == len(c.noisy_deltas) >= 1

        for a in c.actions:
            assert a in ATOMIC_ACTIONS, f"非法动作: {a}"
        for d in c.clean_deltas:
            assert isinstance(d, DeltaAction)
        for d in c.noisy_deltas:
            assert isinstance(d, DeltaAction)

        assert "source" in c.metadata

    print("✅  输出格式正确（3 个字段等长）")


def test_candidate_length_range() -> None:
    """候选轨迹长度应在 ``[expert_len/2, expert_len*2]`` 范围内。"""
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
    planner = TrajectoryPlanner(PlannerConfig(expert_derived_ratio=0.0, candidate_count=4, seed=0))
    candidates = planner.generate(("forward", "forward", "forward"))
    sources = [c.metadata["source"] for c in candidates]
    assert all(s == "pure_random" for s in sources)
    print(f"✅  expert_derived_ratio=0，全部为纯随机")


def test_expert_derived_ratio_all_mutated() -> None:
    """expert_derived_ratio=1 应全部为专家变异候选。"""
    planner = TrajectoryPlanner(PlannerConfig(expert_derived_ratio=1.0, candidate_count=4, seed=0))
    candidates = planner.generate(("forward", "forward", "forward"))
    sources = [c.metadata["source"] for c in candidates]
    assert all(s == "expert_mutation" for s in sources)
    print(f"✅  expert_derived_ratio=1，全部为专家变异")


# ============================================================================
#  增量转换测试
# ============================================================================


def test_delta_conversion_forward() -> None:
    """ "forward" 应映射为 ``(dx=步长, dy=0, dz=0, dphi=0)``。"""
    planner = TrajectoryPlanner(PlannerConfig(horizontal_step=0.1))
    c = planner._build_candidate("t", ("forward",), "t")
    assert c.clean_deltas[0].dx == 0.1
    assert c.clean_deltas[0].dy == 0.0
    assert c.clean_deltas[0].dz == 0.0
    assert c.clean_deltas[0].dphi == 0.0
    print("✅  forward → [0.1, 0, 0, 0]")


def test_delta_conversion_left() -> None:
    """ "left" 应映射为 ``(dx=0, dy=0, dz=0, dphi=+偏航步长)``。"""
    planner = TrajectoryPlanner(PlannerConfig(yaw_step_deg=15.0))
    c = planner._build_candidate("t", ("left",), "t")
    assert c.clean_deltas[0].dx == 0.0
    assert c.clean_deltas[0].dy == 0.0
    assert c.clean_deltas[0].dz == 0.0
    assert c.clean_deltas[0].dphi == 15.0
    print("✅  left → [0, 0, 0, +15]")


def test_delta_conversion_right() -> None:
    """ "right" 应映射为 ``(dx=0, dy=0, dz=0, dphi=-偏航步长)``。"""
    planner = TrajectoryPlanner(PlannerConfig(yaw_step_deg=15.0))
    c = planner._build_candidate("t", ("right",), "t")
    assert c.clean_deltas[0].dphi == -15.0
    print("✅  right → [0, 0, 0, -15]")


def test_delta_conversion_up_down() -> None:
    """ "up" / "down" 应分别映射为 ``dz=+步长`` / ``dz=-步长``。"""
    planner = TrajectoryPlanner(PlannerConfig(vertical_step=0.1))
    c = planner._build_candidate("t", ("up", "down"), "t")
    assert c.clean_deltas[0].dz == 0.1
    assert c.clean_deltas[1].dz == -0.1
    print("✅  up → [0, 0, +0.1, 0]  |  down → [0, 0, -0.1, 0]")


def test_delta_conversion_mixed_sequence() -> None:
    """混合专家序列的转换结果应全部正确。"""
    planner = TrajectoryPlanner(
        PlannerConfig(
            horizontal_step=0.5,
            vertical_step=0.2,
            yaw_step_deg=30.0,
        )
    )
    actions = ("forward", "left", "forward", "up", "right", "down")
    c = planner._build_candidate("test", actions, "test")

    expected = [
        DeltaAction(dx=0.5, dy=0.0, dz=0.0, dphi=0.0),
        DeltaAction(dx=0.0, dy=0.0, dz=0.0, dphi=30.0),
        DeltaAction(dx=0.5, dy=0.0, dz=0.0, dphi=0.0),
        DeltaAction(dx=0.0, dy=0.0, dz=0.2, dphi=0.0),
        DeltaAction(dx=0.0, dy=0.0, dz=0.0, dphi=-30.0),
        DeltaAction(dx=0.0, dy=0.0, dz=-0.2, dphi=0.0),
    ]
    for got, exp in zip(c.clean_deltas, expected):
        assert got == exp, f"预期 {exp}, 实际 {got}"

    print("✅  混合序列转换全部正确")


# ============================================================================
#  噪声测试
# ============================================================================


def test_noise_zero_by_default() -> None:
    """所有噪声 sigma 为 0 时，noisy 应与 clean 完全一致。"""
    planner = TrajectoryPlanner(PlannerConfig(seed=0))
    actions = ("forward", "left", "forward")
    c = planner._build_candidate("t", actions, "t")
    for clean, noisy in zip(c.clean_deltas, c.noisy_deltas):
        assert clean == noisy
    print("✅  默认噪声为 0，noisy == clean")


def test_noise_non_zero() -> None:
    """Sigma 不为 0 时，noisy 应与 clean 有差异。"""
    planner = TrajectoryPlanner(
        PlannerConfig(
            seed=0,
            noise_sigma_dx=0.01,
            noise_sigma_dy=0.005,
            noise_sigma_dz=0.01,
            noise_sigma_dphi=0.5,
        )
    )
    actions = ("forward",) * 50
    c = planner._build_candidate("t", actions, "t")

    all_same = all(
        c.clean_deltas[i] == c.noisy_deltas[i] for i in range(len(c.actions))
    )
    assert not all_same, "噪声 > 0 时应有差异"

    dx_noises = [n.dx - c.clean_deltas[i].dx for i, n in enumerate(c.noisy_deltas)]
    mean_noise = sum(dx_noises) / len(dx_noises)
    assert abs(mean_noise) < 0.01, f"dx 噪声均值应接近 0, 实际 {mean_noise:.4f}"

    print(f"✅  噪声非零，均值接近 0（{mean_noise:.4f}）")


def test_noise_deterministic() -> None:
    """相同种子和配置应产生相同的噪声值。"""
    cfg = PlannerConfig(
        seed=42,
        noise_sigma_dx=0.01,
        noise_sigma_dy=0.01,
        noise_sigma_dz=0.01,
        noise_sigma_dphi=0.5,
    )
    actions = ("forward", "forward")
    c1 = TrajectoryPlanner(cfg)._build_candidate("t", actions, "t")
    c2 = TrajectoryPlanner(cfg)._build_candidate("t", actions, "t")

    for d1, d2 in zip(c1.noisy_deltas, c2.noisy_deltas):
        assert d1 == d2

    print("✅  噪声结果可复现")


def test_to_dict_includes_deltas() -> None:
    """:meth:`TrajectoryCandidate.to_dict` 应包含 clean_deltas 和 noisy_deltas。"""
    planner = TrajectoryPlanner(PlannerConfig(seed=0))
    actions = ("forward", "left")
    c = planner._build_candidate("test", actions, "test")
    d = c.to_dict()

    assert "clean_deltas" in d
    assert "noisy_deltas" in d
    assert "actions" in d
    assert len(d["clean_deltas"]) == len(c.actions)
    assert len(d["noisy_deltas"]) == len(c.actions)

    print("✅  to_dict() 包含 clean_deltas 和 noisy_deltas")


# ============================================================================
#  推荐配置与演示
# ============================================================================


def build_planner() -> TrajectoryPlanner:
    """返回一个使用推荐默认配置的 :class:`TrajectoryPlanner`。

    返回的实例使用:
        - 6 条随机/变异候选
        - 20% 变异率
        - 10 cm 水平和垂直步长，15° 偏航步长
        - 所有四个增量分量上带有小幅高斯噪声

    Returns:
        一个配置好的 :class:`TrajectoryPlanner`。
    """
    return TrajectoryPlanner(
        PlannerConfig(
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
        )
    )


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
    test_delta_conversion_forward()
    test_delta_conversion_left()
    test_delta_conversion_right()
    test_delta_conversion_up_down()
    test_delta_conversion_mixed_sequence()
    test_noise_zero_by_default()
    test_noise_non_zero()
    test_noise_deterministic()
    test_to_dict_includes_deltas()

    # ---- 演示 ----
    print("\n" + "=" * 60)
    print("使用示例: 从专家轨迹生成候选（含 delta + 噪声）")
    print("=" * 60)

    expert = (
        "left",
        "left",
        "forward",
        "forward",
        "forward",
        "forward",
        "forward",
        "right",
        "up",
        "forward",
        "forward",
    )

    print(f"\n专家轨迹 ({len(expert)} 步): {list(expert)}")

    planner = build_planner()
    candidates = planner.generate(expert)

    print(f"\n生成了 {len(candidates)} 条候选:\n")
    for c in candidates:
        preview = " → ".join(c.actions[:6])
        if len(c.actions) > 6:
            preview += f" ... (+{len(c.actions) - 6} 步)"
        print(f"  [{c.name:22s}] {len(c.actions):3d} 步 | {preview}")

    # 展开第一条候选的前几步
    c = candidates[0]
    print("\n第一条候选展开（前 5 步）:")
    hdr = f"  {'步':>3s}  {'动作':10s}  {'clean [dx, dy, dz, dphi]':42s}  {'noisy [dx, dy, dz, dphi]':42s}"
    print(hdr)
    print("  " + "─" * len(hdr))
    for i in range(min(5, len(c.actions))):
        cd = c.clean_deltas[i]
        nd = c.noisy_deltas[i]
        print(
            f"  {i + 1:3d}  {c.actions[i]:10s}  "
            f"[{cd.dx:+7.4f}, {cd.dy:+7.4f}, {cd.dz:+7.4f}, {cd.dphi:+7.2f}]  "
            f"[{nd.dx:+7.4f}, {nd.dy:+7.4f}, {nd.dz:+7.4f}, {nd.dphi:+7.2f}]"
        )
    if len(c.actions) > 5:
        print(f"  ... (共 {len(c.actions)} 步)")

    print("\nto_dict() 输出:")
    print(json.dumps(candidates[0].to_dict(), indent=2, ensure_ascii=False))

    print("\n✅  所有测试通过!")


if __name__ == "__main__":
    main()
