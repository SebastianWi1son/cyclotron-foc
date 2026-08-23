# FOC 核心库 — 开工前置检查 & 指导伪代码

> 日期：2026-08-23 ｜ 状态：**D1~D10/O1 全部已决（2026-08-23，记录表见 FOC_GUIDE.md）→ 按本文件伪代码开工**
> 依据：FOC_GUIDE.md（决策机制/组件边界）+ FOC_DESIGN.md（决策清单）+ REFERENCE_REVIEW.md / FOC_CORE_COMPARISON.md（参考库审阅）
> 数学基线：legacy `lib/foc/`（foc.c / foc.h / foc_transform.h 全文精读，约定照搬）
> 数学精确规格：**FOC_MATH_SPEC.md**（每个组件的具体公式/离散化/边界处理，本文 §3~§10 的细化）

---

## 0. 决断清单（2026-08-23 全部已决，记录表见 FOC_GUIDE.md）

### A. 组件边界决定项（已拍板）

| # | 决策点 | 结论 |
|---|---|---|
| **D1** | namespace | `foc::` 子命名空间分层（transforms/svpwm/angle_tracker/alignment/core/hal） |
| **D2** | v1 范围 | **VOLTAGE + OPEN_LOOP**，CURRENT 留 v2 |
| **D3** | 对齐方式 | **非阻塞状态机**（IDLE→RAMP→SETTLE→LOCKED→FAULT），无 delay_ms_cb |
| **D4** | AngleTracker 归属 | **核心内组件**；HAL get_angle 语义 = 物理角 [0,2π) |
| **D5** | 回调形态 | **函数指针 + ctx 通道**（wheel 同构） |

### B. 默认确认项（已拍板）

| # | 决策点 | 结论 |
|---|---|---|
| D7 | 单位 | rad / rad/s / A / V 全 rad 系；命名中性 |
| D9 | 范围裁剪 | SVPWM 只做线性区（D13 修正后：真 SVPWM = 中心偏置 + 零序注入，上限 Vdc/√3） |
| D10 | 工程约束 | 零依赖（仅 \<cmath\>）、声明/定义分离、-Wall -Wextra -Werror、三 target |
| D6 | PID 来源 | **复用 lunokhod wheel 算法**（`control/wheel/` 的 PID/LPF/Ramp/SmoothPlanner 直接引用源码，不复制；lunokhod 非 git 仓库，submodule 待其转 git + 算法库定案后再议） |
| D8 | 测试锚点 | 六项确认（见 §9） |

### C. 外部依赖状态

- O1 reference 确认 ✅；算法库策略（lunokhod A1~A8）⬜ 待拍板 → 不阻塞（副本过渡已定）

---

## 1. 文件布局（FOC_GUIDE §四，D1 推荐版）

```
foc/
├── CMakeLists.txt            # 库 foc + test_foc + example_foc 三 target
├── inc/foc/                  # 统一头目录（foc 库名）
│   ├── transforms.hpp        # Clarke/Park/InvPark — 零状态纯函数
│   ├── svpwm.hpp             # 中心对齐 + 限幅
│   ├── angle_tracker.hpp     # 多圈展开 + 速度 LPF
│   ├── alignment.hpp         # 对齐状态机（非阻塞）
│   ├── foc_core.hpp          # 编排：模式分发 + 级联（依赖 hal.hpp）
│   ├── hal.hpp               # 回调形态（D5）
│   └── config.hpp            # FocConfig 聚合（wheel 经验）
├── src/                      # 声明/定义分离，一组件一 .cpp
├── test/                     # test_foc.cpp（锚点六项）
└── examples/                 # example_foc.cpp（PC 电机模型闭环）
```

依赖方向（单向，零环）：`config → hal → transforms/svpwm/angle_tracker/alignment → foc_core`

---

## 2. hal.hpp — 硬件抽象（D5：函数指针 + ctx）

```cpp
namespace foc::hal {

// 回调：全部带 ctx（legacy 无 ctx 的痛点修复，wheel 同构）
using GetAngleFn   = float  (*)(void* ctx);                      // 物理角 [0, 2π) rad
using GetCurrentFn = void   (*)(void* ctx, float* ia, float* ib); // v2 预留；v1 置 nullptr
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

## 7. foc_core.hpp — 编排（D2：v1 两模式）

```cpp
namespace foc::core {

// 级联（legacy 已验证，照搬）：target → traj → angle PID → vel 前馈+环 → uq → [CURRENT v2] → svpwm
enum class CtrlMode : uint8_t { VOLTAGE = 0, OPEN_LOOP = 1, CURRENT = 2 /* v2 */ };

struct Config {                       // wheel Config 聚合经验
  // 电气
  float voltage_supply;  // V
  float voltage_limit;   // V（0 = 不输出，安全默认）
  int   pole_pairs;      // 极对数
  int   direction;       // 1 / -1
  float deadzone;        // rad；0 = 关闭（soft deadzone，legacy 沿用）
  float vel_lpf_tf;      // s；0 = 关闭 LPF
  // 对齐（§6 Config 内嵌或平铺）
  float align_voltage; float align_ramp_time; float settle_threshold;
  int settle_samples; float settle_timeout;
  // 轨迹
  float traj_vmax;       // rad/s；0 = 不限速
  float traj_tf;         // s
  // PID（D6：算法库或 wheel 副本，接口见下）
  pid::Config angle_pid; pid::Config vel_pid;
  CtrlMode ctrl_mode;    // v1: VOLTAGE / OPEN_LOOP
};

class FocCore {
public:
  void init(const Config& cfg, hal::Hardware hw);
  void enable(bool on);                      // 包装 hw.enable + 状态复位
  void align_and_sync(float* cmd_target_out); // 非阻塞启动（等效 legacy foc_start_and_sync）
  void tick(float target_angle, float dt);   // 主入口：1kHz ISR 调用（或 OPEN_LOOP 用 tick_velocity）
  // ── OPEN_LOOP（v1） ──
  void tick_velocity(float target_vel, float limit_voltage, float dt); // 等效 legacy foc_open_loop_velocity_tick
  // ── 状态 ──
  bool  aligned() const;
  float angle() const; float velocity() const;
  int   fault() const;                       // 对齐/运行时原因码（v1 至少对齐 FAULT）
};

// tick 伪代码（VOLTAGE 模式，1kHz 假设）：
//   if !alignment.locked():
//       alignment.tick(hw.get_angle(ctx), dt, hw.set_pwm, hw.ctx)
//       return
//   raw = hw.get_angle(ctx)
//   tracker.update(raw, dt)                      // §5：abs_angle + velocity
//   planned = planner.calc(target_angle, dt)     // §7 轨迹规划器（legacy dsp_traj 照搬）
//   err_ang = soft_deadzone(planned - tracker.angle(), cfg.deadzone)
//   vel_cmd = angle_pid.calc(err_ang, dt)
//   vel_cmd += (planned - planned_prev) / dt     // vel 前馈（planner 差分，legacy 照搬）
//   vel_cmd = clamp(vel_cmd, ±traj_vmax)
//   uq = vel_pid.calc(vel_cmd - tracker.velocity(), dt)
//   angle_elec = tracker.angle() * pole_pairs * direction
//   r = svpwm::write(uq, 0, angle_elec - zero_offset_elec, voltage_limit, voltage_supply)
//   hw.set_pwm(ctx, r.ua, r.ub, r.uc)
//   // v2 预留：switch (ctrl_mode) { case CURRENT: ... }（D2 拍 b 则 v1 不写）
}
```

### PID / 轨迹规划器接口（D6：**复用 lunokhod wheel 算法**，不复制；接口以 wheel 源码为权威）

```cpp
// 来源：lunokhod control/wheel/inc|src/（lpf.hpp/ramp.hpp/smooth_planner.hpp/pid.hpp）
// 依赖方向：pid.hpp → lpf.hpp + ramp.hpp；smooth_planner.hpp → ramp.hpp + lpf.hpp
// namespace 包装方式见 FOC_MATH_SPEC §10 决策点 P5；算法库（ALGO_LIB_STRATEGY A 组）落地后原位替换
class LPF  { LPF(float Tf); float calc(float raw, float dt); void reset(); };          // Tf=0 → 直通
class Ramp { Ramp(float max_rate); float calc(float cmd, float dt); void reset(); };    // max_rate=0 → 冻结输出
class SmoothPlanner { SmoothPlanner(float max_rate, float Tf); float calc(float cmd, float dt); void reset(); };
struct PIDConfig { float kp_, ki_, kd_, limit_out_, limit_i_, thresh_i_sep_, max_rate_out_, d_filter_Tf_; };
class PID {
  PID(const PIDConfig&);
  float calc(float cmd, float measure, float dt);  // wheel 接口：cmd=目标, measure=测量（error=cmd-measure）
  void reset();
};
float soft_deadzone(float error, float range);   // 本库自有（legacy dsp_soft_deadzone 数学，公式见 FOC_MATH_SPEC §7）
```

> 注意：wheel `SmoothPlanner`/`Ramp` 在 max_rate=0 时行为是**冻结输出**（非直通），且无对齐后注入初始值的接口（legacy 直接改 planner 内部状态）——这两个接口缺口见 FOC_MATH_SPEC §10 决策点 P1/P3。

---

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

## 12. 拍板记录（2026-08-23 全部通过，开工口令已生效）

- [x] D1 namespace：`foc::` 分层 ✅
- [x] D2 v1 范围：VOLTAGE + OPEN_LOOP ✅
- [x] D3 对齐：非阻塞状态机 ✅
- [x] D4 AngleTracker：核心内 ✅
- [x] D5 回调：函数指针 + ctx ✅
- [x] D6 PID：**复用 lunokhod wheel 算法**（不复制）✅
- [x] D7/D9/D10：按推荐默认通过 ✅
- [x] D8：六项锚点确认 ✅
- [x] D13：SVPWM 语义修正——modulate 加零序注入 v0=-(max+min)/2（carrier-based SVPWM，与经典 7 段式等价）✅ 动机：legacy 与 v1 初版均为 SPWM（无注入，线电压上限 0.866·Vdc），语义偏离修复；代价：锚点 2 不变式改为线电压 ≤ 2·limit（和=3·center 失效）
