# foc-core

零依赖 C++17 FOC（磁场定向控制）核心库：坐标变换、SVPWM 调制、多圈角度跟踪、非阻塞对齐状态机、位置/速度/电流级联控制。**PC 可测、纯算法与硬件隔离**，可直接移植到任意 MCU（STM32/G431 等）。

## 特性

- **零依赖**：仅 `<cmath>`，C++17
- **算法/IO 解耦**：硬件经 `hal` 函数指针回调注入（角度/电流/发波/使能）
- **双入口实时架构**：`pos_tick`（位置环 1kHz）+ `modulation_tick`（调制发波 / 电流环，PWM 频率）
- **非阻塞对齐**：状态机 RAMP→SETTLE→LOCKED，速度判稳 + 超时 FAULT
- **三种控制模式**：`OPEN_LOOP` / `VOLTAGE` / `CURRENT`（v2 dq 电流闭环）
- **全组件单元测试**（ASan/UBSan 加持）+ PC 仿真 example

## 组件

| 组件 | 职责 |
|---|---|
| `transforms` | Clarke / Park / 逆变换（纯函数） |
| `svpwm` | 载波 SVPWM（零序注入） |
| `angle_tracking` | 多圈展开 + 速度估计 + LPF |
| `alignment` | 非阻塞对齐状态机 |
| `current_loop` | dq 电流闭环（v2 CURRENT） |
| `algo` | PID / LPF / Ramp / SmoothPlanner / Deadzone |
| `FOC` | 编排门面（唯一对外 API） |

## 构建与测试

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build            # 5 个测试 target，全部零失败
```

## 快速开始

```cpp
#include "foc/foc.hpp"

// 1. 配置（含电气/对齐/轨迹/PID/电流环参数）
foc::Config cfg{12.0f, 3.0f, 7, 1, /* ... */ foc::CtrlMode::VOLTAGE};

// 2. 构造（hal 回调注入硬件）
foc::FOC motor(hw, cfg);

// 3. 非阻塞启动对齐
float cmd = 0.0f;
motor.align_and_sync(&cmd);

// 4. 实时循环（1kHz 位置环 + PWM 频调制）
motor.pos_tick(cmd, 0.001f);          // 对齐/同步/级联计算
motor.modulation_tick(0.00005f);      // 统一发波；CURRENT 内嵌电流环
```

完整生命周期演示（对齐 → VOLTAGE 阶跃 → CURRENT 电流闭环 → OPEN_LOOP 开环）见 [`examples/example_foc.cpp`](examples/example_foc.cpp)。

## 文档

- [`docs/FOC_CORE_PSEUDOCODE.md`](docs/FOC_CORE_PSEUDOCODE.md) — 主设计文档（架构、接口、决策记录 D1~D13 / D-A1~A6）
- [`docs/ALIGNMENT.md`](docs/ALIGNMENT.md) — 对齐状态机专题
- [`docs/CURRENT_LOOP.md`](docs/CURRENT_LOOP.md) — 电流环四家调研对比（legacy/odrive/qdrive/simplefoc）
- [`docs/ROADMAP.md`](docs/ROADMAP.md) — v3+ 升级路线（控制增强/架构/平台）

## License

MIT
