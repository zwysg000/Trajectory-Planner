# Trajectory Planner — 启发式候选轨迹规划器

ANWM论文第 8.1 节启发式规划器的实现。

给定一条专家轨迹（最优路径），派生多条候选轨迹，用于训练世界模型从多个候选路径中选择最优路径。

---

## 轨迹构成

轨迹由 5 种原子动作拼接而成，每个原子动作有一个名称和对应的连续增量值：

| 动作 |   名称    | 连续增量 (dx, dy, dz, dphi) |
| :--: | :-------: | :-------------------------: |
| 前进 | `forward` |      `(0.1m, 0, 0, 0)`      |
| 左转 |  `left`   |      `(0, 0, 0, +15°)`      |
| 右转 |  `right`  |      `(0, 0, 0, -15°)`      |
| 上升 |   `up`    |      `(0, 0, 0.1m, 0)`      |
| 下降 |  `down`   |     `(0, 0, -0.1m, 0)`      |

> 步长均为默认值，可在配置中修改。

例如，轨迹 `("left", "left", "forward", "forward", "forward")` 表示：先左转 30°，再向前飞 30cm。

---

## 规划器功能

**输入** 一条专家轨迹（最优路径的动作序列）。

**输出** K 条候选轨迹，每条候选同时包含：

|      输出      |           类型            | 含义                                       |
| :------------: | :-----------------------: | :----------------------------------------- |
|   `actions`    |     `tuple[str, ...]`     | 离散动作名称序列                           |
| `clean_deltas` | `tuple[DeltaAction, ...]` | 无噪声的连续增量 `[dx, dy, dz, dphi]`      |
| `noisy_deltas` | `tuple[DeltaAction, ...]` | 在 clean_deltas 上叠加高斯噪声后的连续增量 |

**clean_deltas 与 noisy_deltas 的关系：**

世界模型训练需要从"感知输入"中推理路径。用 clean 值训练过于理想化，因此通过叠加高斯噪声模拟真实世界中传感器的不确定性，使模型学会在噪声中做决策。

如果不需要噪声，将 `noise_sigma_*` 全部置为 0 即可。

---

## 候选生成策略

每条候选的长度在 `[专家轨迹长度/2, 专家轨迹长度×2]` 范围内随机。两种策略的比例由 `expert_derived_ratio` 控制：

**策略一：纯随机。** 长度随机，每步从 5 个动作中均匀采样。

**策略二：专家变异。** 以专家轨迹为模板，调整到随机长度后再按 `action_replacement_rate` 的概率替换每个动作。变异率控制候选与专家的偏离程度：0 为完全保留，1 为完全随机。

---

## 配置参数

| 参数                      | 默认值 | 说明                                               |
| ------------------------- | :----: | -------------------------------------------------- |
| `candidate_count`         |  `5`   | 生成的候选轨迹数量                                 |
| `action_replacement_rate` | `0.2`  | 专家变异时每个动作被替换的概率。0=不替换，1=全替换 |
| `seed`                    | `None` | 随机种子。设为整数可复现结果                       |
| `max_length`              | `200`  | 候选轨迹步数上限                                   |
| `horizontal_step`         | `0.1`  | forward 的前进距离（米）                           |
| `vertical_step`           | `0.1`  | up/down 的升降距离（米）                           |
| `yaw_step_deg`            | `15.0` | left/right 的偏航角度（度）                        |
| `noise_sigma_dx`          | `0.0`  | dx 噪声标准差（米）                                |
| `noise_sigma_dy`          | `0.0`  | dy 噪声标准差（米）                                |
| `noise_sigma_dz`          | `0.0`  | dz 噪声标准差（米）                                |
| `noise_sigma_dphi`        | `0.0`  | dphi 噪声标准差（度）                              |

---

## 用法

```python
from planner import PlannerConfig, TrajectoryPlanner

# 专家轨迹（最优路径）
expert = ("left", "left", "forward", "forward", "forward",
          "forward", "forward", "right", "up")

# 规划器配置
planner = TrajectoryPlanner(PlannerConfig(
    candidate_count=6,
    expert_derived_ratio=0.5,
    action_replacement_rate=0.2,
    horizontal_step=0.1,
    vertical_step=0.1,
    yaw_step_deg=15.0,
    noise_sigma_dx=0.005,
    noise_sigma_dy=0.002,
    noise_sigma_dz=0.003,
    noise_sigma_dphi=0.5,
    seed=42,
))

# 生成候选
candidates = planner.generate(expert)

# 每条候选均包含三个输出
for c in candidates:
    print(c.name, len(c.actions), "步")
    print("  actions:", list(c.actions))
    print("  clean_deltas:", [d.as_list() for d in c.clean_deltas])
    print("  noisy_deltas:", [d.as_list() for d in c.noisy_deltas])
```

---

## 项目文件

```
trajectory_planner_v4/
├── README.md
├── API.md
├── requirements.txt
├── test_planer.py
└── planner/
    ├── __init__.py
    ├── types.py
    └── trajectory_planner.py
```
