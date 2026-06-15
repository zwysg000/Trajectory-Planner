# API 文档

## `planner.ATOMIC_ACTIONS`

```python
ATOMIC_ACTIONS: tuple[str, ...] = ("forward", "left", "right", "up", "down")
```

5 个合法原子动作的元组。

---

## `class DeltaAction`

由离散原子动作积分得到的**累计**连续增量（单值，不是序列）。

表示从起始状态执行完一系列动作之后的总位移和总偏航。

```python
@dataclass(frozen=True)
class DeltaAction:
    dx: float     # 米 — 总 X 方向位移
    dy: float     # 米 — 总 Y 方向位移
    dz: float     # 米 — 总垂直位移
    dphi: float   # 度 — 总偏航变化（正数 = 左转）
```

**方法：**

| 方法        | 返回          | 说明                           |
| ----------- | ------------- | ------------------------------ |
| `as_list()` | `list[float]` | 转为 `[dx, dy, dz, dphi]` 列表 |

---

## `class PlannerConfig`

| 参数                      | 类型          | 默认值 | 说明                                    |
| ------------------------- | ------------- | :----: | --------------------------------------- |
| `candidate_count`         | `int`         |  `5`   | 生成的候选轨迹数量                      |
| `expert_derived_ratio`    | `float`       | `0.5`  | 候选轨迹中从专家路径派生的比例 [0, 1]   |
| `action_replacement_rate` | `float`       | `0.2`  | 专家变异时每步的替换概率 [0, 1]         |
| `seed`                    | `int \| None` | `None` | 随机种子。固定值可复现；`None` 每次不同 |
| `max_length`              | `int`         | `200`  | 候选轨迹步数上限                        |
| `horizontal_step`         | `float`       | `0.1`  | forward 步长（米）                      |
| `vertical_step`           | `float`       | `0.1`  | up/down 步长（米）                      |
| `yaw_step_deg`            | `float`       | `15.0` | left/right 偏航步长（度）               |
| `noise_sigma_dx`          | `float`       | `0.0`  | dx 噪声标准差（米）                     |
| `noise_sigma_dy`          | `float`       | `0.0`  | dy 噪声标准差（米）                     |
| `noise_sigma_dz`          | `float`       | `0.0`  | dz 噪声标准差（米）                     |
| `noise_sigma_dphi`        | `float`       | `0.0`  | dphi 噪声标准差（度）                   |

---

## `class TrajectoryPlanner`

```python
@dataclass
class TrajectoryPlanner:
    config: PlannerConfig
```

### `generate(expert_actions) -> list[TrajectoryCandidate]`

| 参数             | 类型            | 说明                                          |
| ---------------- | --------------- | --------------------------------------------- |
| `expert_actions` | `Sequence[str]` | 专家轨迹，如 `("forward", "left", "forward")` |

**返回：** `candidate_count` 条 `TrajectoryCandidate`。
纯随机和专家变异两种策略的比例由 `expert_derived_ratio` 控制。

**异常：** `ValueError` — `expert_actions` 为空或包含非法动作名。

---

## `class TrajectoryCandidate`

一条候选轨迹，包含三个输出。注意 `clean_delta` 和 `noisy_delta` 是**单值**（不是序列）。

```python
@dataclass(frozen=True)
class TrajectoryCandidate:
    name: str                          # 名称
    actions: tuple[str, ...]           # (A) 离散动作序列
    clean_delta: DeltaAction | None    # (B) 无噪声累计增量（单值）
    noisy_delta: DeltaAction | None    # (C) 加噪声累计增量（单值）
    metadata: dict[str, Any]           # 元数据
```

**方法：**

| 方法        | 返回        | 说明                                                           |
| ----------- | ----------- | -------------------------------------------------------------- |
| `as_list()` | `list[str]` | 动作序列转为列表                                               |
| `to_dict()` | `dict`      | 转为字典，含 `clean_delta` 和 `noisy_delta`（均为 4 元素列表） |
| `__len__()` | `int`       | 轨迹步数                                                       |

---

## 积分规则

|   动作    |                                 对累计状态的影响                                 |
| :-------: | :------------------------------------------------------------------------------: |
| `forward` | 沿当前偏航方向前进 `horizontal_step`: `x += step*cos(yaw)`, `y += step*sin(yaw)` |
|  `left`   |                             偏航增加 `yaw_step_deg`                              |
|  `right`  |                             偏航减少 `yaw_step_deg`                              |
|   `up`    |                               `z += vertical_step`                               |
|  `down`   |                               `z -= vertical_step`                               |

## 噪声模型

在累计结果 `[dx, dy, dz, dphi]` 的四个分量上分别添加独立高斯噪声：

```
noisy_dx = clean_dx + N(0, noise_sigma_dx)
noisy_dy = clean_dy + N(0, noise_sigma_dy)
noisy_dz = clean_dz + N(0, noise_sigma_dz)
noisy_dphi = clean_dphi + N(0, noise_sigma_dphi)
```

所有 sigma 默认 0（无噪声），仅在显式设置时生效。
