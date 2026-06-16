# Trajectory Planner — 启发式候选轨迹规划器

[ANWM](https://arxiv.org/abs/2512.21887)论文第 8.1 节启发式规划器的实现。

**具体功能**：假定我们已经有了一个最准确的专家轨迹，这个规划器能从专家轨迹派生多个候选轨迹，我们希望后续的打分方案能从这些候选轨迹中，选择出最接近专家轨迹的那一条。

这是一个**最基础版本的规划器**。由于它依赖于专家轨迹，所以并不是真实可用的规划器，其实现意义在于:

- 确定规划器的输出应该是什么样子，为后续更高级、可用的规划器提供一个基础。
- 为后续的打分器实现方案提供一个可用于测评的工具。

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

**输出** K 条候选轨迹，每条候选输出**三样东西**：

|     输出      | 含义                                                                                   |
| :-----------: | :------------------------------------------------------------------------------------- |
|   `actions`   | 一串动作名，例如 `["left", "forward", "up"]`                                           |
| `clean_delta` | 执行完所有动作后，从起点到终点的**总位移和总偏航**，以 `[dx, dy, dz, dphi]` 四个数表示 |
| `noisy_delta` | 在 clean_delta 上加了一点随机噪声的结果，模拟真实传感器误差                            |

`[dx, dy, dz, dphi]` 的参数设计与论文 `3.3 第二部分` 给出的 ANWM 输入格式对齐。

- 默认无人机的初始状态是 **(0, 0, 0)** 位置，朝向 **(0°)**
- 每执行一步 `forward`，就沿着当前朝向走 0.1 米
- 每执行一步 `left`，朝向向左转 15°
- 每执行一步 `right`，朝向向右转 15°
- 每执行一步 `up`，高度上升 0.1 米
- 每执行一步 `down`，高度下降 0.1 米

全部执行完后，`[dx, dy, dz, dphi]` 就是无人机相对于起点的位置和朝向。

例如轨迹 `["left", "left", "forward", "forward", "forward"]`：

```
起点位置 (0, 0, 0)，朝向 0°
  left  → 朝向 15°
  left  → 朝向 30°
  forward → 沿 30° 方向走 0.1m → 位置 (0.087, 0.050, 0)
  forward → 沿 30° 方向再走 0.1m → 位置 (0.173, 0.100, 0)
  forward → 沿 30° 方向再走 0.1m → 位置 (0.260, 0.150, 0)
最终: 位置 (0.26, 0.15, 0)，朝向 30°
```

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
#   actions: ['up', 'right', 'forward', 'down', 'right', 'right', 'left', ...]
#   clean_delta: [0.183, -0.076, 0.0, -60.0]
#   noisy_delta: [0.179, -0.075, -0.003, -59.56]
# random_trajectory_3 10 步
#   actions: ['left', 'up', 'forward', 'up', 'up', 'left', 'forward', ...]
#   clean_delta: [0.183, 0.076, 0.1, 15.0]
#   noisy_delta: [0.181, 0.075, 0.102, 15.18]
# mutated_trajectory_1 12 步
#   actions: ['left', 'up', 'left', 'down', 'up', 'forward', 'forward', ...]
#   clean_delta: [0.244, 0.171, 0.1, 30.0]
#   noisy_delta: [0.245, 0.171, 0.101, 30.44]
# mutated_trajectory_2 11 步
#   actions: ['up', 'left', 'forward', 'forward', 'forward', 'forward', ...]
#   clean_delta: [0.483, 0.078, 0.3, -15.0]
#   noisy_delta: [0.484, 0.079, 0.302, -14.6]
# mutated_trajectory_3 14 步
#   actions: ['left', 'down', 'forward', 'right', 'right', 'forward', ...]
#   clean_delta: [0.68, -0.052, 0.0, 15.0]
#   noisy_delta: [0.68, -0.051, 0.001, 15.33]

```

## 我们与 ANWM 在流程上的区别

ANWM 选择的是 Long-horizon visual generation 的方式，类似"视频生成"：规划器一次性给出长序列，世界模型自回归生成一长串视频帧，最后拿最后一帧跟目标图片比相似度。

由于我们的系统涉及到许多长尾场景，通常会对视野造成极大的干扰，Long-horizon 较为困难，因此初步定下的运作方式应该是**走一步看一步**，每步只考虑几个候选动作，执行完再观测、再决策。 详细参考飞书上的流程图。

|       维度       |                             ANWM                             |                                我们的方案                                 |
| :--------------: | :----------------------------------------------------------: | :-----------------------------------------------------------------------: |
|     **输入**     |                Instruction + **目标地点图片**                |                       Instruction + 当前无人机画面                        |
| **目标地点图片** |                             需要                             |                                 **没有**                                  |
|  **规划器输出**  |              一次性给能直达终点的**长动作序列**              |                     每次只输出**短的下一步动作序列**                      |
|  **自回归循环**  | 世界模型拿着长序列自回归生成完整视频帧，直到到达目的地那一帧 | 规划器每次输出短序列 → 世界模型预测 → 规划器打分 → 执行 → 拍下一帧 → 循环 |
|   **打分依据**   |              最后生成的帧与目标图片的**相似度**              |           世界模型预测的未来帧和规划器预期的匹配度（具体待定）            |
|  **规划器角色**  |                      一次性生成完整路径                      |                     每步生成短动作候选 + 二次决策打分                     |

我们方案的关键点：

1. **的规划器只输出很短的短动作序列**（告诉无人机下一步做什么），而不是一次性规划完整路径
2. **世界模型对这些短序列进行预测**（生成未来帧），而不是直接生成完整视频
3. **规划器拿到预测帧后做二次决策**：结合原始序列和世界模型返回的帧进行详细思考，给出最终打分和选择
4. **选出的动作在仿真器中真实执行**，然后拍下一帧更新状态，循环直到到达目的地

## 关于打分

目前怎么打分**还有待商榷**，但基本思路是：

- 世界模型拿着短序列生成未来帧
- 未来帧输入打分器
- 打分器根据它给出的序列和视觉模型返回的帧进行详细思考，给出最优轨迹
- 后续可能会将规划器、世界模型、打分器整体串联，统一优化
