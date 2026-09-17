---
class: fact
generated: false
---
# alignment 组件设计专题（2026-08-23）

> 对齐状态机的方法论总结：状态机/API/判稳设计/别家对比/挂起项。
> 契约与参考代码见 `FOC_CORE_PSEUDOCODE.md` §6（本文件不重复代码，只讲设计方法与取舍）。
> 决策索引：D3（非阻塞状态机）、D16（纯算法化）、D17（速度判据判稳）。

## 1. 组件定位

对齐 = 开机时把转子拉到已知电角度位置并记零位（`zero_offset_elec`），随后闭环以此为基准。

- **为什么需要**：增量式/绝对值编码器给的是机械角，不知道电角度零点；磁场定向需要 `θ_elec = θ_mech·pp·dir − zero_offset`
- **为什么是状态机**：legacy 是阻塞流程（1000 步 × 2ms 斜坡 + 判稳循环），D3 非阻塞化后主循环 1kHz 每 tick 推进一步，无 `delay`
- **为什么纯算法**（D16）：tick 输出 `TickResult{state, u, v, w}` 数据，不发波——hal 仅被 foc_core 依赖（单一 IO 出口）

## 2. 状态机

```
start() → IDLE → RAMP ──满时──→ SETTLE ──连续判稳通过──→ LOCKED
                  ↑                        │超时
                  │                        ↓
                abort() ←──────────────── FAULT（原因码 fault() 可查）
```

| 状态 | 动作 | 转移 |
|---|---|---|
| IDLE | 无（等 start() 点火） | start() → RAMP |
| RAMP | 电压斜坡 0→align_voltage @ 固定电角度 1.5π（svpwm::write 计算 uvw） | t ≥ ramp_time → SETTLE |
| SETTLE | 判稳（速度判据，见 §4） | 连续通过 → LOCKED；超时 → FAULT |
| LOCKED | 冻结零位，输出 0（撤电压由 foc_core 统一出口） | — |
| FAULT | 原因码可查，输出 0，断电由调用方决定 | abort() → IDLE |

结构约定（Q3）：tick = 纯路由表；每个状态逻辑拆 `do_xxx`（转移集中在 do_xxx 内，参数按需传）；无动作态直接 break 不设空函数。

## 3. API 一览

| 方法 | 角色 |
|---|---|
| `Aligner(cfg, pp, dir)` | 构造（pp/dir 为电机参数不走 Config，D14 宿主平铺） |
| `start()` / `abort()` | 点火 / 急停复位 |
| `tick(raw, dt)` → `TickResult` | 每 tick 推进，返回本 tick 应发的波 |
| `is_locked()` | **控制流信号**：foc_core 模式分叉点（对齐期/闭环期）——信息隐藏：不暴露 State 枚举 |
| `zero_offset_elec()` | **产物出口**：LOCKED 后 foc_core 取零位（一次性） |
| `fault()` | **诊断出口**：FAULT 原因码 |

类型组织：`State`/`Fault`/`TickResult` 同族放 namespace 级（类内只留方法 + private 状态）。

## 4. 判稳设计（D17：速度判据）

**为什么需要判稳**：判稳是 FAULT 态的传感器——没有判稳，"对齐失败"无法检测，容错设计是空壳（SimpleFOC 无 fault 才敢不判稳）。

**三参数三角色**：

| 参数 | 角色 | 管什么 |
|---|---|---|
| `settle_max_speed`（2.0 rad/s） | 判稳标准 | 准确性：零位误差上界 = max_speed·dt |
| `settle_samples`（2） | 确认次数 | 抗噪性：连续 N 拍才可信（去抖） |
| `settle_timeout`（0.5 s） | 等待上限 | 容错：迟迟不稳 → FAULT |

判据：`|Δ|/dt < settle_max_speed`（实现 `|Δ| < max_speed·dt`）——速度是间隔无关的物理量，改控制频率不用重调阈值。

**方案演进**（为什么最终选速度判据）：
- legacy：`|Δ| < 0.1 rad @ 50ms 间隔`（= 2 rad/s）——非阻塞化后间隔变 1ms、阈值未缩放 → **语义漂移**：0.1 rad/ms = 100 rad/s 才算动，宽松 50 倍
- A 保持现状（0.1 rad @ 1ms）：静止场景够用，匀速旋转边界误判
- B 阈值缩放（0.002 rad）：与 legacy 等效但抗噪差（编码器噪声可能超）
- C **速度判据（采纳）**：`|Δ|/dt < 2.0`，语义正、与间隔解耦
- D 电流判据（MCSDK 式）：v1 无电流采样（GetCurrentFn=nullptr），v2 升级路径

## 5. 别家对比（判稳/失败检测）

| 家 | 对齐后怎么知道"稳了" | 失败检测 | 零位方式 |
|---|---|---|---|
| SimpleFOC | 不判稳（信任斜坡停稳），对齐角 -90°（=1.5π，与我们一致） | 无 | 相对零位（读一次角度） |
| QDrive | 固定延时 + SAMPLE_COUNT 次采样平均 | 极对间一致性校验（偏差>20% → EncoderError） | **绝对零位**（平均 pp 个极对，抵消极对制造误差） |
| MCSDK | 电流域 Validation Tick（连续 N 个速度环 Id 在带宽内） | Id 带宽 | 状态机内 |
| legacy | 位移差分（50ms 间隔） | retry≤3（耗尽仍继续 = 假成功） | 相对零位（settled·pp·dir） |
| **本库** | **速度判据 + 连续计数**（D17） | 超时/不稳 → FAULT | 相对零位（对齐位置为基准） |

**要点**：
- 主流（SimpleFOC/QDrive）不判稳，用"采样平均"吸收残余运动误差；代价是没有失败检测
- MCSDK 是唯一认真判稳的（电流域），v1 不可行
- 绝对零位（QDrive）需要转 pp 圈 + 极对循环，云台每次开机对齐的场景相对零位就够

## 6. 参数表（Config）

| 字段 | 默认 | 物理含义 |
|---|---|---|
| `align_voltage` | 自由轴 3.0 / 受限轴 1.0 V | 对齐满电压（legacy 经验值） |
| `voltage_supply` | 母线电压 V | RAMP 发波需 center（补 cpp 时发现的缺口） |
| `align_ramp_time` | 2.0 s | 电压斜坡时长（legacy 2s / 1000 步） |
| `settle_max_speed` | 2.0 rad/s | 判稳速度上限（等效 legacy 50ms/0.1rad） |
| `settle_samples` | 2 | 连续判稳通过次数 |
| `settle_timeout` | 0.5 s | 判稳等待上限（覆盖 legacy retry≤3） |

## 7. 挂起/待拍板项

1. **RAMP 发波是否乘 direction**（P2 同源）：legacy 未乘。分析：发波坐标系与闭环坐标系可以不同——零位换算（raw·pp·dir）会把物理位置兜底折算进闭环域，发波不乘也自洽；乘则电机在"自己的坐标系"转 1.5π。两种都行，关键是零位必须乘。待拍板。
2. **UNSTABLE 判据**（Δ > 0.8·2π 判发散）：与 angle_tracker 的回绕判据同源（对齐期角度跳变 = 编码器异常/机械冲击），实现时补全。
3. **RAMP_TIMEOUT / NO_SENSOR 分支**：RAMP 超时（对齐前检查）与传感器缺失（get_angle_cb 为空）的 FAULT 路径，实现时补全。
4. **对齐角 1.5π 的笔误教训**：legacy 是 `1.5·PI`（270°）；曾误写 `1.5·k2PI`（=3π≡180°，差 90° 电角度）——修复后与 SimpleFOC 对齐角（-90°）一致。
