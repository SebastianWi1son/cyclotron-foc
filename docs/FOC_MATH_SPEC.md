# FOC 数学算法规格（MATH SPEC）— 内部算法精确规格

> ⚠️ **已废弃（2026-08-23）：内容已并入 `FOC_CORE_PSEUDOCODE.md`（每组件含数学规格 + hpp + cpp 完整实现），本文档仅作历史存档，不再维护。**
> 阅读入口：foc/docs/FOC_CORE_PSEUDOCODE.md

---

> 日期：2026-08-23 ｜ 版本：v1（存档）
> 定位：FOC_CORE_PSEUDOCODE.md（结构骨架）的**数学细化**——每个组件内部算法的精确公式、离散化、常量、边界处理
> 权威来源：legacy `lib/foc/foc.c` + `foc_transform.h` + `lib/algorithm/dsp`（数学约定照搬）；lunokhod `control/wheel/`（PID/LPF/Ramp/SmoothPlanner **复用**，行为以其源码为准）
> 单位：rad / rad/s / A / V（D7）；浮点全程 float；零依赖（仅 `<cmath>`）

---

## 0. 全局常量与约定（所有组件共用）

```cpp
constexpr float TWO_PI = 6.28318530718f;
constexpr float PI     = 3.14159265359f;
constexpr float SQRT3  = 1.73205080757f;   // legacy 原文常量
constexpr float JUMP_THRESHOLD = 0.8f * TWO_PI;   // = 5.026548f，多圈跳变判据（legacy 沿用）
```

| 约定 | 规则 | 出处 |
|---|---|---|
| 角度 wrap | `x = fmodf(x, TWO_PI); if (x < 0) x += TWO_PI;` → [0, 2π) | legacy foc.c |
| dt 守卫（tick 入口） | `dt <= 0` → 直接 return，不动作 | legacy foc.c |
| dt 守卫（PID 内部） | `dt <= 0 || dt > 0.5` → 强制 `dt = 0.001f` | wheel pid.cpp |
| 0=disabled | deadzone / vel_lpf_tf / align 参数 / PID max_rate_out、thresh_i_sep 等 = 0 关闭对应功能（traj_vmax 语义见 §10 P1） | wheel 经验 |
| clamp | 对称限幅 `clamp(x, ±limit)` | 全库 |
| fmodf 负值 | C 的 fmodf 结果带符号，必须补 `+TWO_PI` 归一 | legacy foc.c |

---

## 1. transforms — 坐标变换（零状态纯函数，`foc::transforms`）

> **D12 类型化数据流**：所有变换输入/输出用几何域小结构体（值类型，寄存器返回 + 内联，零开销）；
> **命名中性（D12）**：类型/函数/变量名不绑电流电压语义——变换是通用线性变换，物理量语义由**链路封装**承担（电流链 `update_currents` / 电压链 `svpwm::write`），变量直接用几何域名（ab/dq/abc）

```cpp
struct AlphaBeta  { float a, b; };      // 两相静止系（等幅值约定）
struct Dq         { float d, q; };      // 两相旋转系
struct ThreePhase { float a, b, c; };   // 三相

inline AlphaBeta  clarke(float ia, float ib);          // 电流链（v2 采样用）；输入标量（ic 隐式）
inline Dq         park(AlphaBeta ab, float angle_elec);         // 电流链
inline AlphaBeta  inv_park(Dq dq, float angle_elec);            // 电压链（发波）
inline ThreePhase inv_clarke(AlphaBeta ab);                     // 电压链（发波）
```

- 类型安全收益：`Dq` 误传入需要 `AlphaBeta` 的参数 → 编译期拒绝；`angle_elec` 保持裸 float（标量域）
- 类型别名复用（SimpleFOC 式）：电流/电压共用同一类型，不做物理量特化（SimpleFOC 的 ABVoltage_s 已废弃，证据）

### 1.1 Clarke（三相静止 → 两相静止，等幅值）

```
AlphaBeta clarke(float ia, float ib):
    return { ia, (ia + 2·ib) / SQRT3 }        // 除法用 * (1/SQRT3) 等价；ic 不参与（Ia+Ib+Ic=0 隐式）
```

- 等幅值恒等式（ic = -ia-ib 时）：**|ab|² = (2/3)·|abc|²**（锚点 1 用）
  > 勘误（2026-08-23 验证时发现）：等幅值约定 ≠ 模长相等——等幅值 = αβ 矢量模长等于相正弦峰值（平衡三相下 |ab| = I_peak，而 |abc|² = (3/2)·I²）；模长相等是**功率不变**约定（|ab|² = |abc|²，此时 αβ 幅值为相峰值的 √(3/2) 倍）。legacy 用等幅值，锚点钉死恒等式。

### 1.2 Park（两相静止 → 两相旋转）

```
Dq park(AlphaBeta ab, float angle_elec):
    cos_a = cosf(angle_elec);  sin_a = sinf(angle_elec)
    return { ab.a·cos_a + ab.b·sin_a,        // d
             -ab.a·sin_a + ab.b·cos_a }      // q
```

> 注意 legacy 符号约定：**d 在前、q 在后**，且 `q = -α·sinθ + β·cosθ`（与某些教材相反，以 legacy 为准，锚点钉死）。

### 1.3 InvPark（两相旋转 → 两相静止）

```
AlphaBeta inv_park(Dq dq, float angle_elec):
    cos_a = cosf(angle_elec);  sin_a = sinf(angle_elec)
    return { -dq.q·sin_a + dq.d·cos_a,        // α
              dq.q·cos_a + dq.d·sin_a }       // β
```

- 实现注：cosf/sinf 各只调一次，复用给 d/q（或 α/β）
- 数值例：θ=π/2, ab={1,0} → dq={0,-1}（可手算验证符号）

### 1.4 inv_clarke（两相静止 → 三相交流分量，等幅值逆变换，D11）

```
ThreePhase inv_clarke(AlphaBeta ab):
    return { ab.a,
             (SQRT3·ab.b - ab.a) / 2.0f,
             (-ab.a - SQRT3·ab.b) / 2.0f }
```

- **严格互逆验证**：把 Clarke 的 iβ=(ia+2·ib)/√3 代回 ib 公式，得 ib ✓（等幅值约定下成对）
- 与 Clarke 成对完备：往返 `clarke∘inv_clarke = I`（锚点 1 新增断言）
- 历史：legacy 把此变换内嵌在 SVPWM 发波公式中（无独立函数）；**2026-08-23 显式化（D11）**——理由：① 死区补偿（v2）按相电流极性修三相电压，操作在 abc 域；② 过调制（v2）的零序/三次谐波注入也在 abc 域；③ 变换模块成对完备、可独立锚点测试。插入点 = inv_clarke 输出之后、中心偏置之前（见 §2 modulate）
- 数值例：θ 无关；uα=0, uβ=1 → ua=0, ub=√3/2≈0.866, uc=-√3/2≈-0.866（三相交流分量和为 0）

### 1.5 完整实现（transforms.hpp，已验证：-Wall -Wextra -Werror + 锚点数值例全过）

```cpp
#pragma once

#include <cmath>

namespace foc::transforms {

// 几何域小结构体（值类型，寄存器返回 + 内联，零开销）
// 命名中性（D12）：不绑电流/电压语义——变换是通用线性变换，
// 物理量语义由链路封装承担（update_currents / svpwm::write）
struct AlphaBeta  { float a, b; };      // 两相静止系（等幅值约定）
struct Dq         { float d, q; };      // 两相旋转系
struct ThreePhase { float a, b, c; };   // 三相

inline constexpr float SQRT3 = 1.73205080757f;   // legacy 原文常量

// Clarke：三相 → αβ（电流链，v2 采样用）
// 等幅值约定（legacy 原文）：ab = { ia, (ia + 2·ib)/√3 }；ic 由 Ia+Ib+Ic=0 隐式
inline AlphaBeta clarke(float ia, float ib) {
    return { ia, (ia + 2.0f * ib) / SQRT3 };
}

// Park：αβ → dq（电流链；legacy 符号约定：d 在前）
// d = α·cosθ + β·sinθ,  q = -α·sinθ + β·cosθ
inline Dq park(AlphaBeta ab, float angle_elec) {
    float cos_a = std::cos(angle_elec);
    float sin_a = std::sin(angle_elec);
    return {  ab.a * cos_a + ab.b * sin_a,
             -ab.a * sin_a + ab.b * cos_a };
}

// InvPark：dq → αβ（电压链，发波）
// α = -q·sinθ + d·cosθ,  β = q·cosθ + d·sinθ
inline AlphaBeta inv_park(Dq dq, float angle_elec) {
    float cos_a = std::cos(angle_elec);
    float sin_a = std::sin(angle_elec);
    return { -dq.q * sin_a + dq.d * cos_a,
              dq.q * cos_a + dq.d * sin_a };
}

// inv_clarke：αβ → 三相交流分量（电压链，发波；D11 从 SVPWM 内嵌显式化）
// 等幅值约定，与 clarke 严格互逆（clarke∘inv_clarke = I）；输出不含中心偏置（交流分量和为 0）
inline ThreePhase inv_clarke(AlphaBeta ab) {
    return { ab.a,
             (SQRT3 * ab.b - ab.a) * 0.5f,
             (-ab.a - SQRT3 * ab.b) * 0.5f };
}

}  // namespace foc::transforms
```

---

## 2. svpwm — 调制器（中心对齐，仅线性区，D9）

> D11 后职责收窄：**svpwm 只做调制（限幅 + 中心偏置），不再内嵌逆 Clarke**；逆变换归还 transforms::inv_clarke（§1.4）
> D12：**配置聚合**（wheel Config 经验）——`Config` 每实例构造一次，tick 只传数据；参数不再混配置与数据

```cpp
struct Config {
    float voltage_limit;    // V，输出限幅（0 = 不输出，安全默认）
    float voltage_supply;   // V，母线电压（决定中心偏置）
};

// 便捷入口（v1 调用方，行为与 legacy 完全一致）：
ThreePhase write(const Config& cfg, Dq dq, float angle_elec);
//    return modulate(cfg, compose(dq, angle_elec, cfg.voltage_limit))

// 组合链（显式，v2 插入点在此展开）→ abc 交流分量：
compose(Dq dq, float angle_elec, float voltage_limit):
    dq.q = clamp(dq.q, ±voltage_limit)      # 1. 先限幅（legacy 顺序，dq 域）
    dq.d = clamp(dq.d, ±voltage_limit)
    θ = wrap(angle_elec)                    # 2. 电角度归一化 [0, 2π)
    ab = inv_park(dq, θ)                    # 3. InvPark（§1.3）
    return inv_clarke(ab)                   # 4. 逆 Clarke（§1.4）

// 底层调制（唯一真正属于 svpwm 的部分）：
ThreePhase modulate(const Config& cfg, ThreePhase abc_ac);   // 输出绝对相电压（含中心）
    center = cfg.voltage_supply / 2.0f
    return { abc_ac.a + center, abc_ac.b + center, abc_ac.c + center }
```

**v2 插入点（D11 预留，记账）**：死区补偿/过调制插在 `compose` 与 `modulate` 之间（abc 域）：

```cpp
ThreePhase abc = compose(dq, angle_elec, cfg.voltage_limit);
abc = deadtime_comp(abc, i_abc);   // v2：按相电流极性修正三相电压
abc = zero_seq_inject(abc);        // v2：过调制/三次谐波注入
ThreePhase r = modulate(cfg, abc);
```

**不变式（锚点 2 断言）：**

| 不变式 | 公式 | 容差 |
|---|---|---|
| 中心对齐 | ua + ub + uc = 3·center | 1e-3 |
| 线性区幅值 | max(\|ua-ub\|, \|ub-uc\|, \|uc-ua\|) ≤ 2·voltage_limit（当 \|uq\|,\|ud\| ≤ limit） | 1e-3 |
| 零输出 | uq=ud=0 → ua=ub=uc=center | 1e-6 |

**推导要点**（写注释用）：inv_park 产出幅值 uq 的旋转矢量 uαβ；中心对齐 = 三相正弦电压各加 center 直流偏置，故三相之和恒为 3·center；线电压幅度 = 2·|uαβ|，线性区 |uαβ|=√(uq²+ud²) ≤ √2·limit（最坏对角），故线电压 ≤ 2·limit 需满足 √2·limit ≤ limit 不成立——**注意**：legacy 用相同 clamp 上限于 uq/ud，极端时线电压可达 2√2·limit，但该情形仅在 uq、ud 同时满幅且 θ 恰在峰值时出现，legacy 接受此行为。v1 电压模式 ud≡0，线电压 ≤ 2·limit 严格成立。锚点只断言 ud≡0 路径（VOLTAGE）。

---

## 3. 复用算法 — LPF / Ramp / SmoothPlanner / PID（lunokhod wheel，行为以其源码为权威）

> 复用形态见 §10 P4；以下公式为 wheel 源码 `control/wheel/src/*.cpp` 的精确行为，**foc 不复制、不重写**。

### 3.1 LPF（一阶低通）

```
init(Tf):   Tf_ = Tf; prev_ = 0.0f
calc(raw, dt):
    alpha = dt / (Tf_ + dt)              # 无 dt<=0 守卫？wheel 源码没有——调用方保证（foc 入口已守）
    prev_ = alpha·raw + (1 - alpha)·prev_
    return prev_
reset():    prev_ = 0.0f
```

- Tf=0 → alpha=1 → 直通（0=disabled 语义成立，无除零：dt/(0+dt)=1）

### 3.2 Ramp（斜率限制器）

```
init(max_rate):  max_rate_ = max_rate; prev_ = 0.0f
calc(cmd, dt):
    step = max_rate_·dt
    out  = cmd
    if cmd > prev_ + step: out = prev_ + step
    elif cmd < prev_ - step: out = prev_ - step
    prev_ = out                          # 关键：每帧都写回（含直通帧）
    return out
reset():    prev_ = 0.0f
```

- max_rate=0 → step=0 → out 恒被拉回 prev_（初始 0）→ **冻结输出**（wheel 语义，非直通！见 §10 P1）

### 3.3 SmoothPlanner（二阶轨迹规划：Ramp + 两级 LPF）

```
init(max_rate, Tf):  ramp_(max_rate); f1_(Tf); f2_(Tf)
calc(cmd, dt):
    ramped = ramp_.calc(cmd, dt)
    return f2_.calc(f1_.calc(ramped, dt), dt)
reset():    ramp_.reset(); f1_.reset(); f2_.reset()
```

- 与 legacy `dsp_traj`（inner_ramp + filter1/filter2）**完全同构**：max_rate→梯形限速，Tf→S 曲线圆角
- **接口缺口**：wheel 版无"注入初始状态"接口（legacy 对齐后直接改 `planner.ramp_target/filter1/filter2 = settled`）→ 见 §10 P3

### 3.4 PID（工业级：微分先行 + 梯形积分 + 积分分离 + 抗饱和 + 输出斜坡）

```
struct PIDConfig { kp_, ki_, kd_, limit_out_, limit_i_, thresh_i_sep_, max_rate_out_, d_filter_Tf_; }  // 全 0 默认

calc(cmd, measure, dt):
    # ① dt 守卫（wheel 特有，比 legacy foc 多一个上界）
    if dt <= 0 || dt > 0.5: dt = 0.001f
    # ② 误差与 P 项
    error = cmd - measure
    p_term = kp_·error
    # ③ I 项：梯形（Tustin）积分 + 抗饱和限幅 + 积分分离
    i_temp = integral_ + ki_·dt·0.5·(error + error_prev_)     # 梯形：用新旧误差均值
    i_temp = clamp(i_temp, ±limit_i_)
    if thresh_i_sep_ <= 0 || |error| <= thresh_i_sep_:        # 大误差冻结积分（不更新）
        integral_ = i_temp
    # ④ D 项：微分先行（对测量值微分，消除设定值跳变冲击）+ D 项 LPF
    d_raw = -kd_·(measure - measure_prev_) / dt
    d_term = d_filter_.calc(d_raw, dt)                        # Tf=0 → 直通
    # ⑤ 输出：合成 → 对称限幅 → 输出斜坡
    output = clamp(p_term + integral_ + d_term, ±limit_out_)
    if max_rate_out_ > 0: output = ramp_out_.calc(output, dt) # 0 = 关闭斜坡
    # ⑥ 状态更新
    error_prev_ = error;  measure_prev_ = measure
    return output

reset():  integral_=error_prev_=measure_prev_=0; d_filter_.reset(); ramp_out_.reset()
```

**与 legacy 的关系（记录，防考古困惑）：**

| 版本 | 形态 | 差异 |
|---|---|---|
| legacy 旧版（`test/bsp/.../pid.h`，8 参） | foc.c 实际按此编译 | 误差微分（有冲击）、无 D 滤波、积分分离仅"不应用"（积分仍累加） |
| legacy 新版（`lib/algorithm/pid`，9 参） | 与 wheel 同构 | 微分先行 + D 滤波；foc.c 的 8 参调用与之**参数错位**（ramp/sep_err 映射错误 + d_filter_Tf 缺失）——legacy 双版本遗留 |
| **wheel（foc 采用）** | 新版语义 C++ 化 | 以源码为准，见上 |

> 结论：foc 直接用 wheel（行为 = legacy 新版正确语义），不沿用 legacy foc.c 的 7 字段 `foc_pid_param_t` 错位调用。

---

## 4. angle_tracker — 多圈展开 + 速度估计（`foc::angle_tracker`）

```
struct Config { float vel_lpf_tf; }    // s；0 = 关闭 LPF

init(cfg):    vel_lpf_tf_ = cfg.vel_lpf_tf
reset(raw):   raw_prev_ = raw; abs_prev_ = raw; abs_ = raw; full_rotations_ = 0; vel_ = 0
             （对齐成功后调用：清零圈数 + 同步绝对角）

update(raw, dt):
    if dt <= 0: return
    d_raw = raw - raw_prev_
    if |d_raw| > JUMP_THRESHOLD:                        # 0.8·2π = 5.0265 rad
        full_rotations_ += (d_raw > 0) ? -1 : +1        # 正向跳变（2π→0）圈数 -1
    raw_prev_ = raw
    last_abs = abs_
    abs_ = full_rotations_·TWO_PI + raw                 # 连续展开角
    raw_vel = (abs_ - last_abs) / dt                    # 差分测速
    vel_ = (vel_lpf_tf_ > 0) ? lpf(vel_lpf_tf_).calc(raw_vel, dt) : raw_vel
```

- 边界：**首次 update 前必须先 reset()**（否则 raw_prev_ 未定义 → 伪跳变）；对齐完成必须 reset 同步
- 锚点 3/5 断言：
  - 回绕：raw 序列 {5.8, 0.1}（d_raw=-5.7 < -5.0265）→ full_rotations=+1；{0.1, 5.8} → -1
  - 恒速 ω：t >> Tf 后 |vel_ - ω| < 容差（LPF 收敛）；零速静止：vel_ 恒 0 无漂移

---

## 5. alignment — 非阻塞对齐状态机（`foc::alignment`）

```
struct Config {
    float align_voltage;    // V，默认 3.0（自由轴 3.0 / 受限轴 1.0 经验沿用）
    float align_ramp_time;  // s，默认 2.0（对应 legacy 1000步×2ms）
    float settle_threshold; // rad，默认 0.1（legacy 沿用）
    int   settle_samples;   // 默认 2（legacy 两次采样判稳）
    float settle_timeout;   // s，默认 0.5（覆盖 legacy retry≤3 语义）
};
enum class State : uint8_t { IDLE, RAMP, SETTLE, LOCKED, FAULT };
enum class Fault : uint8_t { NONE, NO_SENSOR, RAMP_TIMEOUT, UNSTABLE, SETTLE_TIMEOUT };

tick(raw, dt, set_pwm, ctx):          # 每 tick 精确行为
    if dt <= 0: return
    case IDLE:
        # 无动作（PWM 由调用方保证 0；对齐未开始）
    case RAMP:
        t_ += dt
        v = align_voltage_·min(t_ / align_ramp_time_, 1.0f)     # 电压斜坡（legacy i/1000 连续化）
        set_pwm(ctx, svpwm 固定电角度 1.5π=4.7124 rad, uq=v, ud=0)   # 经 svpwm::write 发波
        if t_ >= align_ramp_time_: t_ = 0; settle_count_ = 0; → SETTLE
    case SETTLE:
        t_ += dt
        if |raw - settle_prev_| < settle_threshold_: settle_count_++
        else: settle_count_ = 0                                   # 不连续则清零重来
        settle_prev_ = raw
        if settle_count_ >= settle_samples_: → LOCKED
        if t_ >= settle_timeout_: → FAULT(SETTLE_TIMEOUT)
        if |raw - settle_prev_| > JUMP_THRESHOLD: → FAULT(UNSTABLE)   # 角度发散/丢圈
    case LOCKED:
        zero_offset_elec_ = raw·pole_pairs_·direction
        set_pwm(ctx, 0, 0, 0)                                     # 断电，交闭环接管
    case FAULT:
        # 不输出；调用方读 fault() 决定断电/重试
```

**legacy 阻塞版 → 非阻塞参数映射（写注释用）：**

| legacy 阻塞行为 | 数值 | 映射 |
|---|---|---|
| 1000 步 × 2ms 电压斜坡 | v = align_voltage·(i/1000)，总 2s | align_ramp_time = 2.0，v(t) = align_voltage·(t/2.0) |
| 两次采样（间隔 50ms）判稳 | \|a1-a2\| < 0.1 rad | settle_threshold=0.1, settle_samples=2（非阻塞后间隔=dt，语义放宽为"连续 N 次"） |
| retry ≤ 3（每次 100ms 窗口） | 最多 ~0.5s 稳定期 | settle_timeout = 0.5 |
| 无上报 | — | FAULT + 原因码（v1 新能力） |

- 锚点 4：RAMP 电压随时间单调升到 align_voltage；SETTLE 两次稳定 → LOCKED；超时 → FAULT(SETTLE_TIMEOUT)；跳变 → FAULT(UNSTABLE)

---

## 6. foc_core — 级联编排（`foc::core::FocCore`，v1 两模式）

### 6.1 VOLTAGE tick（对齐完成后每 tick）

```
tick(target_angle, dt):
    if dt <= 0: return
    if !aligned:
        alignment_.tick(hw.get_angle(ctx), dt, hw.set_pwm, hw.ctx)   # 对齐期只跑状态机
        return
    raw = hw.get_angle(ctx)
    tracker_.update(raw, dt)                          # §4
    # ── 轨迹规划（traj_vmax=0 的语义见 §10 P1）──
    planned = planner_.calc(target_angle, dt)         # SmoothPlanner
    # ── 位置环：软死区 + angle PID ──
    err_ang = soft_deadzone(planned - tracker_.angle(), deadzone_)    # §7
    target_vel = angle_pid_.calc(planned, tracker_.angle(), dt)       # 注意 wheel 接口 (cmd, measure)
    # ── vel 前馈（planner 差分，legacy 照搬）──
    target_vel += (planned - planned_prev_) / dt
    planned_prev_ = planned
    target_vel = clamp(target_vel, ±traj_vmax_)
    # ── 速度环 ──
    uq = vel_pid_.calc(target_vel, tracker_.velocity(), dt)
    # ── 发波 ──
    angle_elec = tracker_.angle()·pole_pairs_·direction
    r = svpwm::write(uq, 0.0f, angle_elec - zero_offset_elec_, voltage_limit_, voltage_supply_)
    hw.set_pwm(ctx, r.ua, r.ub, r.uc)
```

> 速度前馈与 legacy 顺序一致：angle PID 输出 + planner 差分（前馈）→ clamp → 进速度环。wheel PID 接口是 `calc(cmd, measure)`，所以位置环调用传 `(planned, tracker.angle())`，速度环传 `(target_vel, tracker.velocity())`——误差符号与 legacy `pid_calculate(error, dt)` 一致。

### 6.2 OPEN_LOOP tick（开环测试）

```
tick_velocity(target_vel, limit_voltage, dt):
    if dt <= 0: return
    limit_voltage = clamp(limit_voltage, 0, voltage_limit_)          # 双重保护
    if |target_vel| < 0.05f:                                         # legacy 死区阈值（rad/s）
        hw.set_pwm(ctx, 0, 0, 0); return
    open_loop_angle_ = wrap(open_loop_angle_ + target_vel·dt)        # 虚拟角度积分 + 归一化
    angle_elec = open_loop_angle_·pole_pairs_                        # ⚠ legacy 不乘 direction！见 §10 P2
    r = svpwm::write(limit_voltage, 0.0f, angle_elec, voltage_limit_, voltage_supply_)
    hw.set_pwm(ctx, r.ua, r.ub, r.uc)
```

### 6.3 对齐完成同步（LOCKED 后由 enable/对齐流程调用）

```
# 等效 legacy foc_start_and_sync 尾部（阻塞版→非阻塞版拆分）：
tracker_.reset(settled_raw)                       # §4 reset：abs 同步、圈数清零
planned_prev_ = settled_raw                       # 前馈差分基线
planner_ 注入 settled_raw                         # ⚠ wheel SmoothPlanner 无此接口，见 §10 P3
cmd_target_out = settled_raw                      # 调用方以此设目标角（等效 legacy *cmd_target）
```

---

## 7. soft_deadzone — 抛物线软死区（本库自有）

```
float soft_deadzone(float error, float range):
    if range <= 0.0f: return error                # 0=disabled 直通（⚠ legacy 无此守卫，range=0 会除零）
    a = |error|
    if a < range: return error·(a / range)        # 抛物线衰减：误差越小增益越小（0 处增益 0）
    return error                                  # 区外原样
```

- 语义：抑制稳态超调/低频震荡（对位置误差做软衰减）；连续可导（抛物线拼接）
- 锚点 6 覆盖：级联链路中含 deadzone=0 与 >0 两路径

---

## 8. 锚点测试 ↔ 数学断言（D8 六项 ↔ 具体断言公式）

| # | 锚点 | 组件 | 断言（容差 1e-3） |
|---|---|---|---|
| 1 | 变换恒等式 | transforms | ① 等幅值恒等式：ab.a²+ab.b² = (2/3)·(ia²+ib²+ic²)（ic=-ia-ib）② 往返：inv_park(park(ab,θ),θ) = ab（±1e-4）③ 符号钉死：θ=π/2, ab={1,0} → dq={0,-1} ④ **成对往返：clarke∘inv_clarke = I（D11）** ⑤ 类型安全：Dq 不能传进 park（编译期，静态断言） |
| 2 | SVPWM 不变式 | svpwm | ua+ub+uc = 3·center；dq={0,0} → 三相全 center；\|dq.q\|≤limit 时线电压 ≤ 2·limit（ud≡0 路径） |
| 3 | 多圈回绕 | angle_tracker | {5.8,0.1}→+1 圈；{0.1,5.8}→-1 圈；小步长（0.5 rad）不误判 |
| 4 | 对齐状态机 | alignment | RAMP 时长=align_ramp_time 且电压单调升；两次稳定→LOCKED 且 zero_offset = raw·pp·dir；超时→FAULT(SETTLE_TIMEOUT) |
| 5 | 速度估计 | angle_tracker | 恒速 ω：LPF 收敛 \|vel-ω\|<ε（t>5·Tf）；零速：vel≡0；reset 后无伪速度 |
| 6 | 级联回归 | foc_core | VOLTAGE 模式：\|uq\| ≤ voltage_limit；死区/前馈路径输出有界；仿真收敛到 target |

---

## 9. 可手算数值例（敲代码时逐行对照）

| 场景 | 输入 | 期望输出 |
|---|---|---|
| Clarke | ia=1, ib=0 | iα=1, iβ=1/√3≈0.5774；iα²+iβ²=1.3333=ia²+ib²+ic²（ic=-1）✓ |
| Park | θ=π/2, iα=1, iβ=0 | id=0, iq=-1 |
| InvPark | θ=π/2, dq={0,1}（d=0,q=1） | ab={-1, 0} |
| inv_clarke | ab={0,1} | abc={0, ≈0.866, ≈-0.866}（和为 0） |
| SVPWM | dq={0,1}, θ=0, limit=1, supply=12 | ab={0,1}→abc 交流={0,0.866,-0.866}；center=6；输出={6, 6.866, 5.134}；和=18=3·center ✓ |
| LPF | Tf=0.01, dt=0.001, raw≡1 | 指数收敛 1-e^(-t/Tf)：第 1 步≈0.0909，第 10 步≈0.613，第 100 步≈0.99995 |
| PID 一步 | kp=2, ki=0, cmd=1, measure=0, dt=0.001 | error=1, p=2, 输出=2（d_filter Tf=0 直通） |

---

## 10. 决策点（复用 lunokhod 算法引出的接口缺口，需拍板）

| # | 决策点 | 背景 | 推荐 | 不拍板后果 |
|---|---|---|---|---|
| P1 | traj_vmax=0 语义 | wheel Ramp 在 max_rate=0 时**冻结输出 0**（非直通）；伪代码原写"0=不限速"与 legacy/wheel 实际行为矛盾 | foc 层判断：traj_vmax<=0 时跳过 planner（planned=target 直通），不依赖 Ramp 的 0 行为 | 语义含糊，测试锚点没法写 |
| P2 | OPEN_LOOP 是否乘 direction | legacy `angle_elec = open_loop_angle·pole_pairs` **漏乘 direction**（疑似缺陷，闭环路径乘了） | 修正：乘 direction（与闭环一致）；锚点钉 | 反向绕线电机开环方向错 |
| P3 | planner 对齐后注入初始角 | wheel SmoothPlanner 只有 reset()（全 0），无 legacy 式 set_state(settled)；对齐后首帧会从 0 渐变 → 位置阶跃 | lunokhod wheel 增加 `set_state(float)`（ramp 与两级 LPF 状态齐同步，行为等效 legacy 直接改字段） | 对齐后目标角跳变/冲击 |
| P4 | 复用形态 | lunokhod 非 git 仓库（submodule 暂不可行）；wheel 库含 foc 不需要的 wheel.cpp | CMake 直接引用 `lunokhod/control/wheel/` 的 4 个算法源（inc+src），不编译 wheel.cpp；lunokhod 转 git 后按 ALGO_LIB_STRATEGY A3 升级 submodule | 跨仓库路径耦合（可接受，同为 workspace 内） |
| P5 | namespace 包装 | wheel 的 PID/LPF/Ramp/SmoothPlanner 是**全局类**（无 namespace），与 foc:: 分层冲突 | foc 侧包 `namespace foc::dsp { }` + 引用处 `using foc::dsp::PID;`（不动 lunokhod 源码）；或等算法库定案统一命名 | 全局名污染；两库类名将来冲突 |

---

## 11. 开工顺序（与 FOC_CORE_PSEUDOCODE §11 对应，逐组件对照本规格）

1. 骨架：CMake 三 target + inc/src 布局（D10）
2. transforms + svpwm（§1/§2）+ 锚点 1/2
3. angle_tracker（§4）+ 锚点 3/5
4. alignment 状态机（§5）+ 锚点 4
5. foc_core（§6 VOLTAGE + OPEN_LOOP）+ soft_deadzone（§7）+ 锚点 6
6. example 闭环仿真（PC 验证全链路）
