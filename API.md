# API 文档

## `planner.ATOMIC_ACTIONS`

```python
ATOMIC_ACTIONS: tuple[str, ...] = ("forward", "left", "right", "up", "down")
```

5 个合法原子动作的元组。

## `class DeltaAction`

一个连续 delta 动作，由离散原子动作转换而来。

```python
@dataclass(frozen=True)
class DeltaAction:
    dx: float     # 米
    dy: float     # 米（恒为 0，无左右平移）
    dz: float     # 米
    dphi: float   # 度
```

**方法：**

| 方法        | 返回          | 说明                           |
| ----------- | ------------- | ------------------------------ |
| `as_list()` | `list[float]` | 转为 `[dx, dy, dz, dphi]` 列表 |

## `class PlannerConfig`

| 参数                      | 类型          | 默认值 | 说明                                    |
| ------------------------- | ------------- | ------ | --------------------------------------- |
| `candidate_count`         | `int`         | `5`    | 生成的候选轨迹数量                      |
| `expert_derived_ratio`    | `float`       | `0.5`  | 候选轨迹中从专家路径派生的比例 [0, 1]   |
| `action_replacement_rate` | `float`       | `0.2`  | 专家变异策略的替换概率 [0, 1]           |
| `seed`                    | `int \| None` | `None` | 随机种子。固定值可复现；`None` 每次不同 |
| `max_length`              | `int`         | `200`  | 候选最大步数上限                        |
| `horizontal_step`         | `float`       | `0.1`  | forward 步长（米）                      |
| `vertical_step`           | `float`       | `0.1`  | up/down 步长（米）                      |
| `yaw_step_deg`            | `float`       | `15.0` | left/right 偏航步长（度）               |
| `noise_sigma_dx`          | `float`       | `0.0`  | dx 噪声标准差（米）                     |
| `noise_sigma_dy`          | `float`       | `0.0`  | dy 噪声标准差（米）                     |
| `noise_sigma_dz`          | `float`       | `0.0`  | dz 噪声标准差（米）                     |
| `noise_sigma_dphi`        | `float`       | `0.0`  | dphi 噪声标准差（度）                   |

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

## `class TrajectoryCandidate`

一条候选轨迹，包含三个输出。

```python
@dataclass(frozen=True)
class TrajectoryCandidate:
    name: str                          # 名称
    actions: tuple[str, ...]           # (A) 离散动作序列
    clean_deltas: tuple[DeltaAction, ...]   # (B) 无噪声 delta 值
    noisy_deltas: tuple[DeltaAction, ...]   # (C) 加噪声 delta 值
    metadata: dict[str, Any]           # 元数据
```

**方法：**

| 方法        | 返回        | 说明                                            |
| ----------- | ----------- | ----------------------------------------------- |
| `as_list()` | `list[str]` | 动作序列转为列表                                |
| `to_dict()` | `dict`      | 转为字典（含 `clean_deltas` 和 `noisy_deltas`） |
| `__len__()` | `int`       | 轨迹步数                                        |

## 离散动作 → DeltaAction 映射

|   动作    |       dx (m)       | dy (m) |      dz (m)      |    dphi (°)     |
| :-------: | :----------------: | :----: | :--------------: | :-------------: |
| `forward` | +`horizontal_step` |   0    |        0         |        0        |
|  `left`   |         0          |   0    |        0         | +`yaw_step_deg` |
|  `right`  |         0          |   0    |        0         | -`yaw_step_deg` |
|   `up`    |         0          |   0    | +`vertical_step` |        0        |
|  `down`   |         0          |   0    | -`vertical_step` |        0        |

## 噪声模型

对每个 DeltaAction 的 4 个分量分别添加独立高斯噪声：

```
noisy_dx = clean_dx + N(0, noise_sigma_dx)
noisy_dy = clean_dy + N(0, noise_sigma_dy)
noisy_dz = clean_dz + N(0, noise_sigma_dz)
noisy_dphi = clean_dphi + N(0, noise_sigma_dphi)
```

所有 sigma 默认 0（无噪声），仅在显式设置时生效。
