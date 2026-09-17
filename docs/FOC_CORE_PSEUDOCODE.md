---
class: fact
generated: false
---
# FOC 核心库 — 开工前置检查 & 指导伪代码

> 日期：2026-08-23 ｜ 状态：**D1~D10/O1 全部已决（2026-08-23，记录表见 FOC_GUIDE.md）→ 按本文件伪代码开工**
> 依据：FOC_GUIDE.md（决策机制/组件边界）+ FOC_DESIGN.md（决策清单）+ REFERENCE_REVIEW.md / FOC_CORE_COMPARISON.md（参考库审阅）
> 数学基线：legacy `lib/foc/`（foc.c / foc.h / foc_transform.h 全文精读，约定照搬）
> 数学**权威**：本文 §3~§10（每组件含数学规格 + 实现）。原 `FOC_MATH_SPEC.md` 已于 2026-08-23 并入本文，
> 2026-08-23 退役至 `trash/`（见 `trash/README.md`）。

---

## 0. 决断清单

> ✅ **全部已决。** 结论 + 日期 + 理由的唯一来源：**[`log/DECISIONS.md`](log/DECISIONS.md)**。
> 本节不再复制结论 —— 复制就是"同一句话说两遍"，迟早打架。

编号速查：`D1`~`D12` = 开工批 · `D13`~`D17` = 实现期 · `D-A1`~`A6` = v2 电流环 · `O1` = 开放问题。

---

## 1. 文件布局（开工计划 vs 实际落地）

> **左列是 2026-08-23 开工时的计划，右列是实际落地。以右列为准**（也可直接看 `README.md`）。
> 保留左列是因为它是决策痕迹 —— 但**别拿它找文件**。

| 计划（2026-08-23，D1 推荐版） | 实际（v1+v2 完工后） |
|---|---|
| `inc/foc/` 下平铺头文件 | 同目录，但 `angle_tracker` → `angle_tracking`（命名更准确） |
| `foc_core` 文件负责编排 | 定名为 `foc`；`FocConfig` 也并入它 |
| `config` 单独成文件聚合 `FocConfig` | 未单独成文件 |
| （未列） | 新增 `algo/`：pid / lpf / ramp / smooth_planner / deadzone |
| （未列） | 新增 `current_loop`（v2 电流环） |
| `CMakeLists.txt` 三 target（库 / test / example） | ✅ 一致 |
| `src/` 声明定义分离，一组件一 `.cpp` | ✅ 一致 |
| `test/` 锚点六项 | ✅ 一致（5 个测试 target） |
| `examples/` PC 电机模型闭环 | ✅ 一致 |

依赖方向（单向，零环）：`config → hal → transforms / svpwm / angle_tracking / alignment → foc`

---

## 2. hal.hpp — 硬件抽象（D5：函数指针 + ctx）

```cpp
namespace foc::hal {

// 回调：全部带 ctx（legacy 无 ctx 的痛点修复，wheel 同构）
using GetAngleFn   = float  (*)(void* ctx);                      // 物理角 [0, 2π) rad
using GetCurrentFn = float  (*)(void* ctx, float* ia, float* ib); // 输出 ia/ib，返回 ic；v2 CURRENT 必提供（v1 忽略）
using SetPwmFn     = void   (*)(void* ctx, float ua, float ub, float uc); // 相电压 V（绝对，含中心）
using EnableFn     = void   (*)(void* ctx, bool on);             // PWM 使能

struct Hardware {
  void* ctx;                 // 驱动层实例透传（G431 = 电机对象）
  GetAngleFn   get_angle;    // 必需
  SetPwmFn     set_pwm;      // 必需
  EnableFn     enable;       // 必需
  GetCurrentFn get_current;  // 可选（v1 恒 nullptr → VOLTAGE 模式）
};

// 无 delay_ms_cb —— D3 非阻塞对齐的直接红利
}
```

---

## 3. transforms — 坐标变换（零状态纯函数；D12 类型化）

### 数学规格

- 几何域小结构体（值类型，寄存器返回 + 内联，零开销）；命名中性（D12）：不绑电流/电压语义，物理量语义由链路封装承担
- 等幅值约定（legacy 原文）：`ab = { ia, (ia + 2·ib)/√3 }`，ic 由 Ia+Ib+Ic=0 隐式
- 等幅值恒等式（ic = -ia-ib）：**|ab|² = (2/3)·|abc|²**（勘误：等幅值 ≠ 模长相等；模长相等是功率不变约定）
- Park 符号约定（legacy）：**d 在前**，`q = -α·sinθ + β·cosθ`（与某些教材相反，锚点钉死）
- 与 clarke 严格互逆：`clarke∘inv_clarke = I`；inv_clarke 输出为交流分量（不含中心偏置，和为 0）

### 完整实现（transforms.hpp — 头文件 inline，无 .cpp）

> 为什么无 cpp：零状态纯函数 + 1kHz 每 tick 调用多次，头文件 inline 消除调用开销并允许跨调用点优化；inline 定义必须在使用它的每个翻译单元可见（ODR），无法下沉到 .cpp。D10 声明/定义分离适用于有状态组件。
> 已验证：-Wall -Wextra -Werror + 锚点数值例全过（2026-08-23）

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

## 4. svpwm — 调制器（中心对齐 + 零序注入 SVPWM，D9：仅线性区；D11：不再内嵌逆 Clarke；D12：Config 聚合；D13：SPWM→SVPWM 语义修正）

### 数学规格

- 配置聚合（wheel 经验）：`Config` 每实例构造一次，tick 只传数据
- 链：clamp（dq 域，legacy 顺序：先限幅后变换）→ wrap(θ) → inv_park → inv_clarke → modulate（中心偏置 + 零序注入）
- 角度 wrap：`fmodf + 负值补 2π` → [0, 2π)（防大角度三角函数精度损失）
- 零序注入（D13）：`v0 = -(max+min)/2`（min-max 中点）加到三相交流分量上 → 调制波峰值从 A 压到 (max-min)/2 = √3·A/2，线性调制比从 1.0 提到 2/√3≈1.155（线电压上限 0.866·Vdc → 1.000·Vdc）；carrier-based SVPWM 与经典 7 段式扇区法严格等价；线电压完全不变（零序在三相等量加）
- 不变式（锚点 2）：① 线电压 ≤ 2·voltage_limit（零序不影响，VOLTAGE 路径 ud≡0）② dq={0,0} → 三相全 center ③ dq=Vdc/√3 全角度不饱和（SVPWM 线性区上边界；SPWM 无注入时此处已削波）
- v2 插入点（D11 预留）：死区补偿/过调制 = compose 与 modulate 之间（abc 域）

### 完整实现

**svpwm.hpp**（声明）：

```cpp
#pragma once

#include "foc/transforms.hpp"

namespace foc::svpwm {

struct Config {
    float voltage_limit;    // V，输出限幅（0 = 不输出，安全默认）
    float voltage_supply;   // V，母线电压（决定中心偏置）
};

// 便捷入口：dq 电压 → 三相绝对相电压（V，含中心偏置 + 零序注入）
// 链：clamp(dq) → wrap(θ) → inv_park → inv_clarke → modulate
// 输入 dq（V，dq 域）+ 电角度（rad）；输出三相绝对相电压（V，含中心偏置 + 零序注入）
transforms::ThreePhase write(const Config& cfg, transforms::Dq dq, float angle_elec);

// 底层调制：三相交流分量 + 零序注入 + 中心偏置（D13；v2 死区补偿/过调制插入点：inv_clarke 后、本函数前）
transforms::ThreePhase modulate(const Config& cfg, transforms::ThreePhase abc_ac);

}  // namespace foc::svpwm
```

**svpwm.cpp**（实现，已验证：-Wall -Wextra -Werror + 锚点数值例全过，2026-08-23）：

```cpp
#include "foc/svpwm.hpp"
#include <cmath>

namespace foc::svpwm {

namespace {   // 文件私有（内部链接）
constexpr float TWO_PI = 6.28318530718f;

// 对称限幅 [-limit, +limit]（wheel PID::constrainf 同款，避免嵌套 fmax/fmin）
float constrainf(float val, float limit) {
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

// 组合链：clamp → wrap → inv_park → inv_clarke → abc 交流分量
// v2 死区补偿/过调制插入点 = 本函数与 modulate 之间（abc 域）
transforms::ThreePhase compose(transforms::Dq dq, float angle_elec, float voltage_limit) {
    // 1. 先限幅（legacy 顺序：dq 域；limit=0 → 输出 0，安全默认）
    dq.q = constrainf(dq.q, voltage_limit);
    dq.d = constrainf(dq.d, voltage_limit);
    // 2. 电角度归一化 [0, 2π)——防大角度三角函数精度损失
    float theta = std::fmod(angle_elec, TWO_PI);
    if (theta < 0.0f) theta += TWO_PI;
    // 3. InvPark → 4. 逆 Clarke
    return transforms::inv_clarke(transforms::inv_park(dq, theta));
}
}  // namespace

transforms::ThreePhase write(const Config& cfg, transforms::Dq dq, float angle_elec) {
    return modulate(cfg, compose(dq, angle_elec, cfg.voltage_limit));
}

transforms::ThreePhase modulate(const Config& cfg, transforms::ThreePhase abc_ac) {
    float center = cfg.voltage_supply * 0.5f;
    // 零序注入（D13）：v0 = -(max+min)/2，调制波峰值压到 (max-min)/2
    float vmax = std::fmax(abc_ac.a, std::fmax(abc_ac.b, abc_ac.c));
    float vmin = std::fmin(abc_ac.a, std::fmin(abc_ac.b, abc_ac.c));
    float zero_seq = -(vmax + vmin) * 0.5f;
    return { abc_ac.a + center + zero_seq,
             abc_ac.b + center + zero_seq,
             abc_ac.c + center + zero_seq };
}

}  // namespace foc::svpwm
```

锚点：线电压 ≤ 2·voltage_limit；dq=Vdc/√3 全角度不饱和（D8-2 + D13）；inv_clarke 成对往返（D11，并入 D8-1）。

### 数值例（D13 实测，2026-08-23）

cfg{voltage_limit=1, voltage_supply=12}：

| 输入 | 输出 (ua, ub, uc) | 说明 |
|---|---|---|
| dq={0,1}, θ=0 | (6.000, 6.866, 5.134) | 巧合 v0=0（a=0, b=-c 对称），与 SPWM 相同；和=18 |
| dq={0,1}, θ=1 | (5.135, 6.865, 5.929) | 注入生效：ac 对称化 (-0.865, +0.865, -0.071)，v0=-0.0236，和=18-0.071=3·center+3·v0 |
| dq={0, Vdc/√3}, θ 全扫描 | 全 ∈ [0, 12] | 线性区上边界不饱和（SPWM 同输入已削波到 >12） |

---

## 5. angle_tracker.hpp — 多圈展开 + 速度估计（D4：核心内）

```cpp
namespace foc::angle_tracker {

// 输入：raw 物理角 [0,2π)（HAL get_angle 语义）；输出：abs 展开角 + 滤波速度
struct Config {
  float vel_lpf_tf;   // s；0 = 关闭 LPF（wheel 0=disabled 语义）
};

class Tracker {
public:
  void   init(const Config& cfg);
  void   reset(float raw_angle);        // 对齐成功后同步（含 full_rotations 清零）
  void   update(float raw_angle, float dt);  // dt 实测穿透
  float  angle() const;                 // 多圈展开角 rad（连续）
  float  velocity() const;              // rad/s（LPF 后）
  int    full_rotations() const;
};
}
```

伪代码（update）：
```
d_raw = raw - raw_prev
if |d_raw| > 0.8·2π:      // 跳变判据（legacy 沿用）
    full_rotations += (d_raw > 0) ? -1 : +1
raw_prev = raw
abs = full_rotations·2π + raw
raw_vel = (abs - abs_prev) / dt
vel = (cfg.vel_lpf_tf > 0) ? lpf(raw_vel, dt) : raw_vel   // LPF 接口见 §7
```

锚点：±2π 跳变计数正确（D8-3）；恒速收敛、零速无漂移（D8-5）。

---

## 6. alignment.hpp — 非阻塞状态机（D3）

```cpp
namespace foc::alignment {

// legacy 对齐动作的非阻塞化：电压斜坡 → 判稳 → 记零位 → 同步
// legacy 阻塞版：1000 步×2ms 斜坡 @1.5π 电角度 → 两次采样(50ms 间隔)Δ<0.1rad 判稳，retry≤3 → zero_offset_elec = settled·pp·dir
struct Config {
  float align_voltage;   // V（自由轴 3.0 / 受限轴 1.0 经验值沿用）
  float align_ramp_time; // s（默认 2.0，对应 legacy 2s）
  float settle_threshold; // rad（默认 0.1，legacy 沿用）
  int   settle_samples;  // 判稳所需连续采样数（默认 2）
  float settle_timeout;  // s（默认 0.5，覆盖 legacy retry≤3 语义）
};

enum class State : uint8_t { IDLE, RAMP, SETTLE, LOCKED, FAULT };
enum class Fault : uint8_t { NONE, NO_SENSOR, RAMP_TIMEOUT, UNSTABLE, SETTLE_TIMEOUT };

class Aligner {
public:
  void   init(const Config& cfg, int pole_pairs, int direction);
  void   start();                        // IDLE → RAMP（等效 legacy foc_start_and_sync 入口）
  void   abort();                        // 任意态 → IDLE（G431 应响应急停）
  State  tick(float raw_angle, float dt, hal::SetPwmFn set_pwm, void* ctx);
  //   IDLE: 无动作
  //   RAMP: 电角度固定 1.5π；v = align_voltage·(t/ramp_time) 每 tick 递增
  //         到 t≥ramp_time → SETTLE（t 由 dt 累积）
  //   SETTLE: 采样 raw_angle，连续 settle_samples 次 |Δ|<settle_threshold → LOCKED；
  //           超时 → FAULT(SETTLE_TIMEOUT)；角度发散(Δ>2π·0.8?) → FAULT(UNSTABLE)
  //   LOCKED: 计算 zero_offset_elec = raw·pole_pairs·direction，保持输出 0
  //   FAULT:  原因码可查，输出由调用方断电
  bool   locked() const;   // LOCKED 后供 FocCore 取 zero_offset_elec / 同步角度
  float  zero_offset_elec() const;
  Fault  fault() const;
};
}
```

锚点：RAMP→SETTLE→LOCKED 迁移 + 失败路径（不稳→超时→FAULT）（D8-4）。对齐期间每 tick 的 set_pwm 由状态机直接调（ctx 穿透），不经过 FocCore 闭环。

---


## 7. foc.hpp — 编排（v1 两模式 + v2 电流环双入口，D-A2 方案 b 已拍板）

> 本节按**文件为单位**给出完整代码（`inc/foc.hpp` + `src/foc.cpp` + `inc/current_loop.hpp`），可直接抄。
> 依赖：`inc/algo/`（lpf/ramp/smooth_planner/pid/deadzone）、`transforms/svpwm/angle_tracking/alignment/hal` 均在本仓库。
> **双入口架构（D-A2）**：`tick()` = position loop（低频 1kHz：对齐/同步/位置速度环 → 产出**命令缓冲**）；`current_tick()` = current loop（PWM 频：**统一发波出口**；CURRENT 内嵌电流闭环）。级联命令经成员跨 tick 缓冲（uq_ref_/iq_ref_/align_uvw_）。双速率是各家主流（odrive/simplefoc/qdrive），legacy 1kHz 同频为简化特例；v2 结构不欠债、频率不提前付税（分析见 docs/CURRENT_LOOP.md）。

```cpp
// ============ inc/foc.hpp — 编排层门面（foc 库唯一对外 API） ============
#pragma once

#include "algo/deadzone.hpp"
#include "algo/pid.hpp"
#include "algo/smooth_planner.hpp"
#include "alignment.hpp"
#include "angle_tracking.hpp"
#include "current_loop.hpp"
#include "hal.hpp"
#include <cstdint>

namespace foc {

// ----- ctrl mode options -----
enum class CtrlMode : uint8_t { OPEN_LOOP, VOLTAGE, CURRENT };

// ----- foc internal config -----
struct Config {
    // --- electric ---
    float voltage_supply_;   // V
    float voltage_limit_;    // V（0 = 不输出，安全默认）
    int pole_pairs_;         // 极对数
    int direction_;          // 1 / -1
    float vel_lpf_tf_;       // s；0 = 关闭 LPF（直通）
    // --- align（§6 平铺） ---
    float align_voltage_; float align_ramp_time_; float settle_max_speed_;
    int settle_samples_; float settle_timeout_;
    // --- trajectory ---
    float traj_vmax_;        // rad/s；0 = 冻结输出（wheel Ramp 语义）
    float traj_tf_;          // s
    // --- pid（纯配置；实例由 FOC 构造时建） ---
    algo::PIDConfig angle_pid_;
    algo::PIDConfig vel_pid_;
    algo::PIDConfig iq_pid_;   // v2 CURRENT：q 轴电流环
    algo::PIDConfig id_pid_;   // v2 CURRENT：d 轴电流环
    float iq_limit_;           // A；v2（D-A3 simplefoc 经验：电流目标限幅；0 = 不限制）
    algo::Deadzone deadzone_;  // {range, soft}；range=0 → 直通（值语义对象进 Config）
    CtrlMode ctrl_mode_;       // v1: OPEN_LOOP / VOLTAGE；v2: CURRENT
};

// 级联（legacy 已验证，照搬）：target → traj → angle PID → vel 前馈+环 → uq/iq_ref → [v2 电流环] → svpwm
class FOC {
public:
    FOC(hal::Hardware hw, const Config& cfg);            // 构造统一；hw 在前（用户拍板）
    void enable(bool on);                                // hw.enable wrapper + 状态复位
    void align_and_sync(float* cmd_target_out);          // 非阻塞启动（等效 legacy foc_start_and_sync）
    // --- position loop（低频 1kHz） ---
    void tick(float angle_cmd, float dt);                // 对齐/同步/位置速度环 → 命令缓冲
    // --- current loop（PWM 频，D-A2 双入口） ---
    void current_tick(float dt);                         // 统一发波出口；CURRENT 内嵌电流闭环
    // --- open loop（v1） ---
    void tick_velocity(float vel_cmd, float limit_voltage, float dt);
    // --- getter ---
    bool is_aligned() const;
    float angle() const;
    float velocity() const;
    int fault() const;

private:
    hal::Hardware hw_;
    Config config_;
    angle_tracking::Tracker tracker_;
    alignment::Aligner aligner_;
    algo::PID vel_pid_, pos_pid_;
    algo::SmoothPlanner planner_;
    current_loop::CurrentLoop current_loop_;   // v2 CURRENT；构造时 cfg.iq_pid_/id_pid_
    float planned_prev_;
    float angle_elec_;      // position tick 算电角，current tick 消费（v2 不外推，见 CURRENT_LOOP.md）
    float zero_offset_;
    bool synced_;
    // --- 跨 tick 命令缓冲（position → current） ---
    float uq_ref_;          // VOLTAGE：速度环输出
    float iq_ref_;          // CURRENT：速度环输出 clamp ±iq_limit_
    transforms::ThreePhase align_uvw_;   // 对齐状态机输出（current tick 统一发波）
};

}  // namespace foc
```

```cpp
// ============ src/foc.cpp — FOC 编排实现（双入口） ============
#include "foc.hpp"
#include "svpwm.hpp"
#include <algorithm>
#include <cmath>

namespace foc {

FOC::FOC(hal::Hardware hw, const Config& cfg)
    : hw_(hw), config_(cfg),
      tracker_(angle_tracking::Config{cfg.vel_lpf_tf_}),
      aligner_(alignment::Config{cfg.align_voltage_, cfg.voltage_supply_, cfg.align_ramp_time_,
                                 cfg.settle_max_speed_, cfg.settle_samples_, cfg.settle_timeout_},
               cfg.pole_pairs_, cfg.direction_),
      vel_pid_(cfg.vel_pid_), pos_pid_(cfg.angle_pid_),
      planner_(cfg.traj_vmax_, cfg.traj_tf_),
      current_loop_(cfg.iq_pid_, cfg.id_pid_),
      planned_prev_(0.0f), angle_elec_(0.0f), zero_offset_(0.0f), synced_(false),
      uq_ref_(0.0f), iq_ref_(0.0f), align_uvw_{0.0f, 0.0f, 0.0f} {}

void FOC::enable(bool on) {
    hw_.enable_(hw_.ctx_, on);
    if (!on) { aligner_.abort(); synced_ = false; }
}

void FOC::align_and_sync(float* cmd_out) {
    aligner_.start();
    synced_ = false;
    *cmd_out = 0.0f;
}

// ── position loop（1kHz）：状态推进 + 级联计算，产出命令缓冲 ──
void FOC::tick(float angle_cmd, float dt) {
    if (dt <= 0.0f) return;
    float raw = hw_.get_angle_(hw_.ctx_);

    if (!aligner_.is_locked()) {                       // 对齐：状态机推进，只算不发
        auto ar = aligner_.calc(raw, dt);
        align_uvw_ = {ar.u_, ar.v_, ar.w_};            // 缓冲 → current_tick 统一发波
        return;
    }
    if (!synced_) {                                    // 一次性同步（零位/原点/播种）
        zero_offset_ = aligner_.zero_offset_elec();
        tracker_.reset(raw);
        planner_.set_state(tracker_.angle_abs());
        planned_prev_ = tracker_.angle_abs();
        synced_ = true;
    }
    tracker_.update(raw, dt);
    float planned = planner_.calc(angle_cmd, dt);
    float err_ang = config_.deadzone_.calc(planned - tracker_.angle_abs());
    float vel_cmd = pos_pid_.calc(err_ang, 0.0f, dt);
    vel_cmd += (planned - planned_prev_) / dt;         // vel 前馈
    planned_prev_ = planned;
    if (config_.traj_vmax_ > 0.0f)
        vel_cmd = std::clamp(vel_cmd, -config_.traj_vmax_, config_.traj_vmax_);
    float uq = vel_pid_.calc(vel_cmd, tracker_.velocity(), dt);

    angle_elec_ = tracker_.angle_abs() * config_.pole_pairs_ * config_.direction_ - zero_offset_;
    if (config_.ctrl_mode_ == foc::CtrlMode::CURRENT) {
        iq_ref_ = (config_.iq_limit_ > 0.0f)
                      ? std::clamp(uq, -config_.iq_limit_, config_.iq_limit_)   // D-A3 限幅
                      : uq;
    } else {
        uq_ref_ = uq;                                  // VOLTAGE
    }
}

// ── current loop（PWM 频）：统一发波出口 ──
void FOC::current_tick(float dt) {
    if (!aligner_.is_locked()) {                       // 对齐期：发对齐波（1kHz 阶梯，对齐电压小可接受）
        hw_.set_pwm_(hw_.ctx_, align_uvw_.u_, align_uvw_.v_, align_uvw_.w_);
        return;
    }
    if (config_.ctrl_mode_ == foc::CtrlMode::CURRENT) {
        float ia = 0.0f, ib = 0.0f;
        hw_.get_current_(hw_.ctx_, &ia, &ib);          // v2：HAL 必提供；ic = -ia-ib 内部推算
        auto udq = current_loop_.update(ia, ib, angle_elec_, iq_ref_, dt);
        auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_}, udq, angle_elec_);
        hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
    } else {                                           // VOLTAGE：直接调制速度环输出
        auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_},
                             {0.0f, uq_ref_}, angle_elec_);
        hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
    }
}

// ── open loop（v1） ──
void FOC::tick_velocity(float vel_cmd, float limit_voltage, float dt) {
    angle_elec_ += vel_cmd * config_.pole_pairs_ * config_.direction_ * dt;   // P2 已修：乘 direction
    auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_},
                         {0.0f, limit_voltage}, angle_elec_);
    hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
}

// ── getter ──
bool  FOC::is_aligned() const { return aligner_.is_locked(); }
float FOC::angle()     const { return tracker_.angle_abs(); }
float FOC::velocity()  const { return tracker_.velocity(); }
int   FOC::fault()     const { return static_cast<int>(aligner_.fault()); }

}  // namespace foc
```

```cpp
// ============ inc/current_loop.hpp — 电流环组件（v2 CURRENT；D-A1 独立组件） ============
// 待实现：用户敲完后同步校准（测试：假电流源 + RL 模型收敛，见 §9 锚点扩展）
#pragma once
#include "algo/pid.hpp"
#include "transforms.hpp"

namespace foc::current_loop {

// dq 电流闭环：ia/ib → Clarke → Park → Iq/Id 双 PID（目标 Iq=iq_ref, Id=0）→ {ud, uq}
// 纯算法无 IO（与 alignment 同构）；频率由调用方决定（current_tick）
class CurrentLoop {
public:
    CurrentLoop(const algo::PIDConfig& iq_cfg, const algo::PIDConfig& id_cfg);
    transforms::DQ update(float ia, float ib, float angle_elec, float iq_ref, float dt);  // 返回 {ud, uq}
    void reset();
private:
    algo::PID iq_pid_, id_pid_;
};

}
```

## 8. 单位与命名约定（D7，全文档生效）

- 角度 rad（[0,2π) 原始角；展开角连续）、角速度 rad/s、电流 A、电压 V
- 命名中性（不绑 rpm/deg），单位写进每个 API 注释；锚点测试钉单位
- 0=disabled 语义（wheel 经验）：deadzone/vel_lpf_tf/traj_vmax=0 关闭对应功能

---

## 9. 测试锚点映射（D8 六项 → 组件）

| # | 锚点 | 组件 | 测试要点 |
|---|---|---|---|
| 1 | 变换恒等式 | transforms | 构造已知向量，Clarke→Park→InvPark 往返一致 |
| 2 | SVPWM 不变式 | svpwm | 线电压 ≤ 2·limit；dq=Vdc/√3 全角度不饱和（D13 零序注入） |
| 3 | 多圈回绕 | angle_tracker | ±2π 跳变按 0.8·2π 判据计数正确 |
| 4 | 对齐状态机 | alignment | RAMP→SETTLE→LOCKED + 失败路径（不稳→FAULT） |
| 5 | 速度估计 | angle_tracker | 恒速→LPF 收敛；零速→无漂移 |
| 6 | 级联回归 | foc_core | VOLTAGE 模式 planner→angle→vel 链路输出有界 |

工程约束（D10）：test target 独立编译、-Werror；锚点测行为不测数值细节。

---

## 10. example — PC 电机模型闭环（wheel 电机模型思路复用）

```
example_foc.cpp：
  构造电机模型（PMSM 简化：Vq→Iq 一阶 + 反电动势；角度积分）
  构造 foc::core::FocCore + 假 HAL（读模型角度 / 写模型电压）
  对齐（状态机步进）→ 闭环 tick 循环 → 断言角度收敛到 target
```

---

## 11. 开工顺序（对应 FOC_GUIDE §六，每步带锚点测试）

1. 骨架：CMake 三 target + inc/src 布局 + -Werror（D10）
2. transforms + svpwm + 锚点 1/2
3. angle_tracker + 锚点 3/5
4. alignment 状态机 + 锚点 4
5. foc_core（VOLTAGE + OPEN_LOOP）+ 锚点 6（级联回归）
6. example 闭环仿真（PC 验证全链路）

---

## 12. 拍板记录

> ✅ **全部已决（2026-08-23 ~ 08-26）。** 完整记录见 **[`log/DECISIONS.md`](log/DECISIONS.md)**。
> 本节原是一份"逐条打勾"的清单，现已被那份记录取代（同一事实只留一处）。
