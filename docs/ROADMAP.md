# foc 库升级路线（v3+ 候选清单）

日期：2026-08-26（当前节点视角：v1 双模式 + v2 CURRENT 已完成，5 测试全绿）
定位：以"当前完成态"为基准，记录**已知但未做**的升级项——动机、来源、落地条件。拍板后移入 FOC_CORE_PSEUDOCODE.md §12。

## 当前节点快照（2026-08-26）

- **组件**：algo（lpf/ramp/pid/smooth_planner/deadzone）/ transforms / svpwm（D13 零序注入）/ angle_tracking / alignment / **current_loop** / FOC 双入口编排
- **架构**：`pos_tick`（1kHz：对齐/同步/级联 → 命令缓冲）+ `modulation_tick`（PWM 频：统一发波；CURRENT 内嵌电流闭环）；OPEN_LOOP 独立发波（方案 a）
- **测试**：5 target 全绿（svpwm / tracker / alignment / foc / current_loop，ASan+UBSan）
- **锚点**：D8 六项全覆盖；CURRENT 组件级收敛验证（RL 模型）
- **已知局限**：modulation_tick 用 position 缓冲电角度（滞后 ≤1ms，不外推）；VOLTAGE 调制逐相 clamp（非矢量缩放）；模式构造期固定；无传感器降级路径

## A. 控制增强（来源：四家调研，详见 CURRENT_LOOP.md §6）

| # | 升级项 | 来源 | 动机 | 落地条件 |
|---|---|---|---|---|
| A1 | **相位外推**：Park 用 `phase + ω·Δt` 补偿采样-控制延迟 | odrive | modulation_tick 读 1ms 前的电角度，电流环相位滞后 | 真实时序数据（timestamp）；G431 采样延迟实测 |
| A2 | **BEMF 估计 + d/q 交叉耦合补偿**：`-i·ω·Lq` / `+i·ω·Ld` | simplefoc | 高速时反电动势/耦合项显著，电流环带宽受限 | 电机参数（Lq/Ld/Ke 或 KV） |
| A3 | **电流模式降级阶梯**：voltage → estimated → dc_current → foc_current | simplefoc | 传感器缺失/失效时保持可控 | 失败语义设计（GetCurrentFn 返回值约定） |
| A4 | **校准体系**：电流零偏 / 相电阻 / 对齐电压自适应 | qdrive | 真实硬件零偏漂移、相电阻误差影响电流环精度 | 真实硬件（G431 + 电流采样） |
| A5 | **饱和矢量缩放 + 锁积分衰减**：mod 整体缩放保方向，饱和时 `integral ×= 0.99` | odrive/qdrive | 逐相 clamp 会畸变电压矢量方向；积分饱和复位慢 | 当前 svpwm 饱和路径改造（D13 之后） |

## B. 架构级

| # | 升级项 | 动机 | 落地条件 |
|---|---|---|---|
| B1 | **OPEN_LOOP 并入统一发波出口**（方案 b） | tick_velocity 自带 set_pwm 是唯一"双出口"残留（方案 a 是有意保留） | 拍板 b；tick_velocity 只算不发 + 电压缓冲 |
| B2 | **运行时模式切换（bumpless）**：VOLTAGE↔CURRENT 热切换 | 云台运行中切模式需求；legacy 是运行时字段 | 积分状态迁移设计（切换瞬间 PID/缓冲一致性） |
| B3 | **modulation_tick 频率参数化**：与 PWM 解耦 | 当前假定调用方决定频率；高频化需要显式配置 | Config 加频率字段或调用约定文档 |
| B4 | **中断原子性**（G431 落地） | pos/modulation 跨中断共享缓冲（float 非原子） | 中断优先级设计；临界区或单写单读模式 |
| B5 | **角度外推**（与 A1 同源） | current 用 position 缓冲角度滞后 | 同 A1 |

## C. 固件/平台

| # | 升级项 | 动机 | 落地条件 |
|---|---|---|---|
| C1 | **example_foc 完善**：PC 电机模型闭环仿真（wheel 模型思路） | 演示 + 无硬件验证 | 电机模型（反电动势/齿槽/负载） |
| C2 | **G431 移植**：hal 实现（TIM/ADC/DMA/编码器）、中断接线 | 目标平台 | 固件骨架阶段（cyclotron 主项目） |
| C3 | **真实云台集成**：与 legacy 行为对齐验证 | 长期目标 | 硬件 + 标定 |

## D. 测试/工程

| # | 升级项 | 动机 | 落地条件 |
|---|---|---|---|
| D1 | **CURRENT 集成级联回归**（锚点 6 扩展） | 当前集成级只有冒烟（有界+激励） | 收敛断言需跨层观察（iq_ref 私有）——可加测试钩子 |
| D2 | **仿真模型升级**：完整电机模型（反电动势/齿槽/负载/摩擦力） | 组件测试精度上限 | 模型复杂度与测试稳定性权衡 |

## 优先级建议

1. **v3 第一批（性能）**：A1（相位外推）→ A5（矢量缩放饱和）→ A2（BEMF/耦合补偿）
2. **v3 第二批（架构）**：B2（运行时切换）→ B1（出口统一）→ B4（中断原子性，随 G431 一起）
3. **平台线**：C2（G431 移植）优先于 C1/C3——固件骨架阶段启动
4. **测试**：D1 随 B2 一起做（切换测试天然需要集成级断言）
