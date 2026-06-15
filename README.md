# Trajectory Planner — 启发式候选轨迹规划器

[ANWM](https://arxiv.org/abs/2512.21887)论文第 8.1 节启发式规划器的实现。

给定一条专家轨迹（最优路径），派生多条候选轨迹，用于训练世界模型从多个候选路径中选择最优路径。

## 轨迹构成

轨迹由 5 种原子动作拼接而成：

| 动作 |   名称    | 效果                |
| :--: | :-------: | :------------------ |
| 前进 | `forward` | 沿当前朝向前进 0.1m |
| 左转 |  `left`   | 原地左转 15°        |
| 右转 |  `right`  | 原地右转 15°        |
| 上升 |   `up`    | 上升 0.1m           |
| 下降 |  `down`   | 下降 0.1m           |

> 步长均为默认值，可在配置中修改。该设计遵循原论文 `Table 4`。

例如，轨迹 `("left", "left", "forward", "forward", "forward")` 表示：先左转 30°，再向前飞 30cm（沿 30° 方向）。

## 规划器功能

**输入** 一条专家轨迹（最优路径的动作序列）。

**输出** K 条候选轨迹，每条候选包含：

|     输出      |       类型        | 含义                                                     |
| :-----------: | :---------------: | :------------------------------------------------------- |
|   `actions`   | `tuple[str, ...]` | 离散动作名称序列                                         |
| `clean_delta` |   `DeltaAction`   | **单值**，执行完所有动作后相对于起点的累计总位移和总偏航 |
| `noisy_delta` |   `DeltaAction`   | 在 clean_delta 上叠加高斯噪声后的结果                    |

`clean_delta` 和 `noisy_delta` 均为**单值**（不是每步一个），表示从起点到终点的积分结果。

累计位移按**世界坐标系**计算——先左转再前进会产生 X 和 Y 两个方向的分量。

`[dx, dy, dz, dphi]` 的格式与论文 `3.3 第二部分` 给出的 ANWM 输入格式对齐。

**clean_delta 与 noisy_delta 的关系：**

世界模型训练需要从"感知输入"中推理路径。用 clean 值训练过于理想化，因此在累计最终结果上叠加高斯噪声模拟真实世界中传感器的不确定性，使模型学会在噪声中做决策。

如果不需要噪声，将 `noise_sigma_*` 全部置为 0 即可。

## 候选生成策略

每条候选的长度在 `[专家轨迹长度/2, 专家轨迹长度×2]` 范围内随机。两种策略的比例由 `expert_derived_ratio` 控制：

**策略一：纯随机。** 长度随机，每步从 5 个动作中均匀采样。

**策略二：专家变异。** 以专家轨迹为模板，调整到随机长度后再按 `action_replacement_rate` 的概率替换每个动作。替换率控制候选与专家的偏离程度：0 为完全保留，1 为完全随机。

## 配置参数

| 参数                      | 默认值 | 说明                                                                     |
| ------------------------- | :----: | ------------------------------------------------------------------------ |
| `candidate_count`         |  `5`   | 生成的候选轨迹数量                                                       |
| `expert_derived_ratio`    | `0.5`  | 候选轨迹中从专家路径派生的比例。0=全部纯随机，0.5=各一半，1=全部专家变异 |
| `action_replacement_rate` | `0.2`  | 专家变异时每个动作被替换的概率。0=不替换，1=全替换                       |
| `seed`                    | `None` | 随机种子。设为整数可复现结果                                             |
| `max_length`              | `200`  | 候选轨迹步数上限                                                         |
| `horizontal_step`         | `0.1`  | forward 的前进距离（米）                                                 |
| `vertical_step`           | `0.1`  | up/down 的升降距离（米）                                                 |
| `yaw_step_deg`            | `15.0` | left/right 的偏航角度（度）                                              |
| `noise_sigma_dx`          | `0.0`  | dx 噪声标准差（米）                                                      |
| `noise_sigma_dy`          | `0.0`  | dy 噪声标准差（米）                                                      |
| `noise_sigma_dz`          | `0.0`  | dz 噪声标准差（米）                                                      |
| `noise_sigma_dphi`        | `0.0`  | dphi 噪声标准差（度）                                                    |

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

# 每条候选输出三个内容
for c in candidates:
    print(c.name, len(c.actions), "步")
    print("  actions:", list(c.actions))
    print("  clean_delta:", c.clean_delta.as_list())
    print("  noisy_delta:", c.noisy_delta.as_list())

# 输出示例 (seed=42):
#
# random_trajectory_1 5 步
#   actions: ['up', 'up', 'right', 'right', 'down']
#   clean_delta: [0.0, 0.0, 0.1, -30.0]
#   noisy_delta: [0.005, -0.004, 0.096, -29.94]
# random_trajectory_2 14 步
#   actions: ['up', 'right', 'forward', 'down', 'right', 'right', ...]
#   clean_delta: [0.183, -0.076, 0.0, -60.0]
#   noisy_delta: [0.179, -0.075, -0.003, -59.56]
# mutated_trajectory_1 12 步
#   actions: ['left', 'up', 'left', 'down', 'up', 'forward', ...]
#   clean_delta: [0.244, 0.171, 0.1, 30.0]
#   noisy_delta: [0.245, 0.171, 0.101, 30.44]
# mutated_trajectory_2 11 步
#   actions: ['up', 'left', 'forward', 'forward', 'forward', ...]
#   clean_delta: [0.483, 0.078, 0.3, -15.0]
#   noisy_delta: [0.484, 0.079, 0.302, -14.6]
```
