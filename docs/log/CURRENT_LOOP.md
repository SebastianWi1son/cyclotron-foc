---
class: log
generated: false
---
# 电流环布置构建对比调研（legacy / odrive / qdrive / simplefoc）

日期：2026-08-23
背景：cyclotron foc v2 引入电流环（`CtrlMode::CURRENT`）前，调研四家电流环的**布置**（结构位置、运行频率、数据流）与**构建**（目标来源、饱和处理、补偿、故障策略）差异，支撑 D-A1~A6 拍板（记录见 FOC_CORE_PSEUDOCODE.md §12）。

---

## 1. 四家总览（一句话）

| 家 | 定位 | 一句话 |
|---|---|---|
| **legacy** | 我们的基线（H743 云台） | 电流环是 FOC 的**小跟班**：1kHz 全同频，static 函数内联，速度环输出**直喂**电流目标 |
| **odrive** | 高端伺服 | 电流环是**主角**：独立类 + PWM 频率运行，测量/控制解耦 + 时间戳校验，P 项常开/饱和锁积分 |
| **qdrive** | 国产云台电机 | 单方法平铺 + ADC 硬件触发采样，**校准文化**（零偏/相电阻/对齐电压自适应） |
| **simplefoc** | 开源教学 | 电流环与位置环**彻底分离**（loopFOC/move），**四档降级阶梯**，电流目标显式限幅 |

---

## 2. 各家解剖

### 2.1 legacy（STM32H743 云台）

```c
static void foc_current_control_step(foc_motor_t *motor, float uq_ref, float angle_elec, float dt) {
  // 读 ia, ib（ADC DMA buffer，TIM TRGO 硬件触发）；ic = -ia-ib
  // Clarke → i_alpha/beta；Park(angle_elec) → iq, id
  // iq_err = uq_ref - iq;   id_err = 0 - id
  // uq = pid_calculate(&iq_pid, iq_err, dt);  ud = pid_calculate(&id_pid, id_err, dt)
  // foc_svpwm_write(uq, ud, angle_elec);
}
```

- **频率**：与位置环同频 1kHz（TIM 中断内联调用）
- **电流目标**：速度环输出 `uq_ref` **直喂** Iq 目标（单位混用、无电流限幅），Id = 0
- **饱和**：PID 内部 limit_out；无矢量饱和处理
- **回退**：`get_current_cb` 为空 → fallthrough VOLTAGE
- **补偿**：无

### 2.2 odrive（FieldOrientedController）

```cpp
// 采样回调：只存测量 + 记时间戳（测量与控制解耦）
on_measurement(vbus_voltage, Ialpha_beta, timestamp);
// 控制环取输出：时间戳超差 → ERROR_BAD_TIMING（数据太旧拒用）
get_alpha_beta_output(output_timestamp, &mod_alpha_beta, &ibus);

// 电流模式核心：
Ierr_d = Id_setpoint - Id;  Ierr_q = Iq_setpoint - Iq;
mod = V_to_mod * (Vdq前馈 + integral + Ierr * p_gain);   // P 项无条件在回路
if (矢量饱和) { mod 整体缩放(0.8·√3/2); integral *= 0.99f; }   // 饱和锁积分 + 衰减
else           { integral += Ierr * (i_gain * period); }
```

- **频率**：电流环 = PWM 更新频率（20-40kHz）；位置环在 controller 独立（downsample）
- **相位外推**：Park 用 `phase + phase_vel·Δt`（采样时刻外推到控制时刻）
- **PI 特色**：P 项常开；**饱和时积分衰减 ×0.99**（anti-windup 的另一种形式）
- **调制饱和**：矢量缩放（保方向不保幅）+ 饱和锁积分
- **前馈**：`Vdq_setpoint_` 在电流模式下作为前馈电压项
- **报告滤波**：Id/Iq 测量 LPF（控制用原始值）
- **故障**：错误码通道（BAD_TIMING / INITIALIZING / UNKNOWN_CURRENT_MEASUREMENT）

### 2.3 qdrive（QD4310）

```cpp
void QDrive::loopCtrl() {                    // ADC 注入转换完成回调（PWM 硬件触发）
  UpdateCurrent(iu, iv, iw);                 // Clarke + Park + CurrentQ/D 滤波
  Angle = wrap(...);  ElectricalAngle = ...; // 电角度 + 速度（同方法内）
  ud = PID_CurrentD.calc(Id); uq = PID_CurrentQ.calc(Iq);
  SetPhaseVoltage(ud, uq, ElectricalAngle);
}
// TIM6 1kHz 中断 → Ctrl_ISR（位置/速度环）——双速率
```

- **频率**：ADC 注入转换由 PWM 硬件触发 → 电流环 = PWM 频率；TIM6 1kHz 位置环
- **采样防尖峰**：`SetPhaseVoltage` 里 ud/uq × 0.99（堵转时某相 Ux=0 导致采样尖峰、电机抽搐的工程坑）
- **矢量缩放**：`ud²+uq² > 1` 整体 scale（保方向）
- **校准体系**（v4.x 更新日志）：电流偏置校准（开机等 30ms 再校）、相电阻测量、电角度校准时自动调整硬拖电压
- **电流滤波**：CurrentQFilter/CurrentDFilter（测量 LPF）

### 2.4 simplefoc（FOCMotor）

```cpp
void FOCMotor::loopFOC() {   // 高频循环：电角度 + 电流环（torque_controller 四档）
  case voltage:           voltage.q = constrain(current_sp, ±voltage_limit) + ff; break;
  case estimated_current: current.q = LPF(current_sp);                          // 无传感器：I·R + BEMF 猜
                          voltage.q = current.q * R + voltage_bemf; break;
  case dc_current:        current.q = LPF(getDCCurrent(angle));                 // 单电流采样
                          voltage.q = PID_current_q(current_sp - current.q); break;
  case foc_current:       current = getFOCCurrents(angle);                      // 完整 dq 环
                          voltage.q = PID_current_q(current_sp - current.q);
                          voltage.d = PID_current_d(0 - current.d); break;
}
void FOCMotor::move() {    // 独立循环：位置/速度环；motion_downsample 显式降采样
```

- **结构**：`loopFOC()` 与 `move()` **完全分离**，天然双速率（loopFOC 高频、move 低频）
- **电流目标**：`current_sp` + **`current_limit` 显式限幅** + `feed_forward_current`
- **四档降级阶梯**：voltage → estimated_current（无传感器）→ dc_current（单采样）→ foc_current（全闭环）
- **补偿**：BEMF 估计（KV）+ **d/q 交叉耦合补偿**（`-i·ω·Lq` / `+i·ω·Ld`）
- **测量滤波**：current.q/d 均 LPF

---

## 3. 对比表

| 维度 | legacy | odrive | qdrive | simplefoc |
|---|---|---|---|---|
| 结构 | FOC 内 static 函数 | 独立类（测量/控制解耦） | 类成员方法平铺 | loopFOC/move 分离 |
| 频率 | 1kHz 同频 | PWM 频率 | ADC 硬件触发（PWM 频） | 分离 + downsample |
| 电流目标 | uq_ref 直喂（无限幅） | Idq_setpoint + Vdq 前馈 | 内隐 | current_sp + **current_limit** |
| PI 饱和 | PID 内部 limit_out | **P 常开 + 饱和锁积分衰减** | 标准 PID | 标准 PID |
| 调制饱和 | 无（svpwm 内隐） | 矢量缩放 0.8·√3/2 | 矢量缩放 + 0.99 防尖峰 | 逐项 clamp |
| 补偿 | 无 | **相位外推** | 校准体系 | **BEMF + 交叉耦合** |
| 故障 | cb 空 → fallthrough VOLTAGE | 错误码通道 | — | 四档降级阶梯 |

---

## 4. 关键差异通俗版

1. **频率**：legacy 是"全公司一个打卡钟"（1kHz 什么都干）；三家都是"流水线分车间"——电流环跟 PWM 节奏（快），位置环慢慢跑（慢）。电流变化毫秒级、位置变化几十毫秒级，本就不该一个节奏。
2. **结构**：legacy 电流环是 FOC 的一个步骤；三家全是独立对象/独立循环——电流环有自己的生命周期。
3. **目标**：legacy 速度环输出多大电流目标就多大（无安全带）；simplefoc 有 `current_limit` 安全带。
4. **饱和**：legacy 无专门处理；odrive/qdrive 都是**矢量缩放**（保方向），odrive 还"锅糊了停止加料"（锁积分 + 衰减）。
5. **补偿**：legacy 裸奔；odrive 采样-控制不同步用相位外推补；simplefoc 用 BEMF/电感耦合补；qdrive 靠校准（零偏/相电阻）补。
6. **故障**：legacy 单点回退；odrive 错误码；simplefoc 降级阶梯（从全闭环一路退到开环电压）。

---

## 5. 对我们的启示与决策映射（已拍板）

| 调研发现 | 我们的决策 | 状态 |
|---|---|---|
| 三家均为独立结构 | **D-A1** `current_loop::CurrentLoop` 独立组件（纯算法无 IO） | ✅ 拍板 |
| 双速率是主流（legacy 同频是简化特例） | **D-A2** 方案 b 双入口：`tick()` position loop + `current_tick()` current loop 统一发波 | ✅ 拍板 |
| simplefoc `current_limit`（低成本高价值） | **D-A3** `iq_limit_` 电流目标限幅 | ✅ 拍板 |
| legacy fallthrough / simplefoc 降级阶梯 | **D-A4** v2 不做（HAL 电流必提供）；降级阶梯留 v3 | ✅ 拍板 |
| odrive 双速率同源问题 | v2 电流环用 position tick 的电角度（不外推）；相位外推留 v3 | ✅ 拍板 |
| odrive/qdrive 矢量缩放 | 不需要——svpwm::calc 内部 D13 已处理调制饱和 | ✅ 关闭 |
| qdrive 校准体系 / simplefoc 补偿 | v3 候选清单 | ⏳ |

## 6. v3 候选清单

- [ ] 相位外推（odrive：Park 用 phase + ω·Δt）
- [ ] BEMF 估计 + d/q 交叉耦合补偿（simplefoc）
- [ ] 电流模式降级阶梯（simplefoc：estimated/dc_current）
- [ ] 校准体系（qdrive：电流零偏 / 相电阻 / 对齐电压自适应）
- [ ] 饱和矢量缩放 + 锁积分衰减（odrive/qdrive 风格 anti-windup）
