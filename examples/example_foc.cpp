// examples/example_foc.cpp — FOC 完整生命周期演示（PC 仿真，零硬件）
//
// 演示链路：
//   1. Config 配置（含 v2 电流环字段）
//   2. 构造 FOC → 非阻塞对齐（pos_tick + modulation_tick 双入口）
//   3. VOLTAGE 闭环：阶跃目标跟踪（观察角度/速度/输出）
//   4. CURRENT 模式：dq 电流闭环（RL 电机模型，观察 Iq 收敛）
//   5. OPEN_LOOP：tick_velocity 开环速度积分
//
// 构建运行：
//   cmake --build cmake-build-debug --target example_foc
//   ./cmake-build-debug/example_foc
//
// 假 HAL 说明：传感器角静止（对齐用）；CURRENT 阶段启用 dq 域 RL 电流模型
// （R=0.5Ω L=1mH，tau=2ms，angle_elec≈0 假设）。

#include "foc.hpp"

#include <cmath>
#include <cstdio>

namespace {

// ──────────────────────────────────────────────
// 假电机 + 假 HAL（演示用，与测试同思路）
// ──────────────────────────────────────────────
struct SimMotor {
    float angle = 1.2f;       // 传感器角 [0, 2π)
    bool  current_model = false;
    float i_d = 0.0f, i_q = 0.0f;   // RL 模型电流（dq 域）
    int   pwm_calls = 0;
    float pwm[3] = {0, 0, 0};
};

SimMotor g_motor;

float get_angle(void*) { return g_motor.angle; }

float get_current(void*, float* ia, float* ib) {
    if (g_motor.current_model) {
        // dq(angle≈0) → αβ → abc
        float iu = g_motor.i_d;
        float iv = (1.73205080757f * g_motor.i_q - g_motor.i_d) * 0.5f;
        *ia = iu; *ib = iv;
        return -iu - iv;
    }
    *ia = 0.0f; *ib = 0.0f;
    return 0.0f;
}

void set_pwm(void*, float u, float v, float w) {
    g_motor.pwm_calls++;
    g_motor.pwm[0] = u; g_motor.pwm[1] = v; g_motor.pwm[2] = w;
    if (g_motor.current_model) {
        // abc 去中心 → αβ（angle≈0：u_d=u_alpha, u_q=u_beta）→ RL 步进
        float uu = u - 6.0f, uv = v - 6.0f;
        float u_alpha = uu;
        float u_beta = (uu + 2.0f * uv) * 0.577350269189626f;
        constexpr float dt = 0.00005f, R = 0.5f, L = 1e-3f;
        g_motor.i_d += (u_alpha - R * g_motor.i_d) / L * dt;
        g_motor.i_q += (u_beta - R * g_motor.i_q) / L * dt;
    }
}

void enable(void*, bool) {}

foc::hal::Hardware make_hw() {
    return {nullptr, get_angle, get_current, set_pwm, enable};
}

// ──────────────────────────────────────────────
// 配置工厂（逐字段注释，演示 Config 全貌）
// ──────────────────────────────────────────────
foc::Config make_config(foc::CtrlMode mode) {
    return foc::Config{
        12.0f,                              // voltage_supply_  V（母线）
        3.0f,                               // voltage_limit_   V（输出安全上限）
        7,                                  // pole_pairs_      极对数
        1,                                  // direction_       转向
        0.0f,                               // vel_lpf_tf_      s（0 = 关闭速度 LPF）
        1.0f, 0.02f, 0.5f, 5, 0.2f,         // 对齐：电压/斜坡时间/判稳速度/样本数/超时
        2.0f, 0.05f,                        // traj_vmax_（0=冻结）/ traj_tf_
        foc::algo::PIDConfig{}.kp(0.5f).limit_out(3.0f),     // angle_pid_（kp ki kd limit_out limit_i ...）
        foc::algo::PIDConfig{}.kp(0.1f).limit_out(3.0f),     // vel_pid_
        foc::algo::PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f),  // iq_pid_（v2 CURRENT）
        foc::algo::PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f),  // id_pid_（v2 CURRENT）
        0.5f,                               // iq_limit_        A（v2 电流目标限幅）
        {0.0f, true},                       // deadzone_        {range, soft}
        mode                                // ctrl_mode_
    };
}

// 对齐 + 打印完成状态
bool align_and_report(foc::FOC& foc, float target, float dt_pos, float dt_mod) {
    foc.align_and_sync(&target);
    int guard = 0;
    do {
        foc.pos_tick(target, dt_pos);
        foc.modulation_tick(dt_mod);
    } while (!foc.is_aligned() && ++guard < 500);
    std::printf("  对齐: %s%s  angle=%.3f rad  fault=%d\n",
                foc.is_aligned() ? "LOCKED" : "FAILED",
                foc.is_aligned() ? " ✓" : "", foc.angle(), foc.fault());
    return foc.is_aligned();
}

float pwm_dev() {   // 三相输出相对中心 6V 的最大偏差
    return std::fmax(std::fabs(g_motor.pwm[0] - 6.0f),
           std::fmax(std::fabs(g_motor.pwm[1] - 6.0f), std::fabs(g_motor.pwm[2] - 6.0f)));
}

}  // namespace

int main() {
    std::printf("=== foc 库示例（PC 仿真）===\n\n");

    // ── 1. VOLTAGE 模式：对齐 + 闭环阶跃 ──
    std::printf("[1] VOLTAGE 模式：对齐 → 闭环阶跃\n");
    g_motor = SimMotor{};
    foc::FOC foc_v(make_hw(), make_config(foc::CtrlMode::VOLTAGE));
    if (!align_and_report(foc_v, 0.0f, 0.001f, 0.001f)) return 1;

    const float target = 2.2f;               // 绝对角系：对齐位置 1.2 + 前方 1 rad
    std::printf("  阶跃目标 %.1f rad（对齐位置前方 1 rad）\n", target);
    for (int i = 1; i <= 50; i++) {
        foc_v.pos_tick(target, 0.001f);
        foc_v.modulation_tick(0.001f);
        if (i % 10 == 0)
            std::printf("    t=%.3fs  angle=%7.3f rad  vel=%6.3f rad/s  out=%5.2f V±%.3f\n",
                        i * 0.001f, foc_v.angle(), foc_v.velocity(), 6.0f, pwm_dev());
    }
    std::printf("\n");

    // ── 2. CURRENT 模式：dq 电流闭环（RL 模型）──
    std::printf("[2] CURRENT 模式：dq 电流闭环（RL 模型 R=0.5Ω L=1mH，iq_limit=0.5A）\n");
    g_motor = SimMotor{};
    g_motor.current_model = true;
    foc::FOC foc_c(make_hw(), make_config(foc::CtrlMode::CURRENT));
    if (!align_and_report(foc_c, 0.0f, 0.001f, 0.00005f)) return 1;

    for (int i = 1; i <= 2000; i++) {
        if (i % 20 == 0) foc_c.pos_tick(target, 0.001f);   // 位置环 1kHz
        foc_c.modulation_tick(0.00005f);                   // 电流环 20kHz
        if (i % 400 == 0)
            std::printf("    t=%.4fs  iq=%6.3f A  id=%6.3f A  out=%5.2f V±%.3f\n",
                        i * 0.00005f, g_motor.i_q, g_motor.i_d, 6.0f, pwm_dev());
    }
    std::printf("    （iq 收敛至 vel 环输出限幅 ≈0.2A；若 vel_pid kp 更大则触发 iq_limit 饱和）\n");
    std::printf("\n");

    // ── 3. OPEN_LOOP：开环速度积分 ──
    std::printf("[3] OPEN_LOOP：tick_velocity 开环（2 rad/s，限压 1.5V）\n");
    g_motor = SimMotor{};
    foc::FOC foc_o(make_hw(), make_config(foc::CtrlMode::OPEN_LOOP));
    for (int i = 1; i <= 10; i++) {
        foc_o.tick_velocity(2.0f, 1.5f, 0.001f);
        std::printf("    t=%.3fs  电角度积分推进 → out=(%5.2f, %5.2f, %5.2f) V\n",
                    i * 0.001f, g_motor.pwm[0], g_motor.pwm[1], g_motor.pwm[2]);
    }

    std::printf("\n=== 示例结束（%d 次发波）===\n", g_motor.pwm_calls);
    return 0;
}
