// test_foc — foc 库锚点测试（D8 锚点 1/6 + Deadzone 工具 + FOC 黑盒集成 + v2 CURRENT 电流环）
// 黑盒：只经公开 API（FOC / Deadzone / transforms / CurrentLoop），不碰实现细节
// fake_motor：独立电机模拟器（静止 / 正弦抖动 / RL 电流模型），经 hal 回调接入 FOC
// 双入口：pos_tick（位置环 1kHz）+ modulation_tick（调制发波；测试 1:1 交替，CURRENT 用 20kHz）
// 工程约束：-Wall -Wextra -Werror + ASan/UBSan；退出码 = 失败数

#include "foc.hpp"
#include "transforms.hpp"

#include <cmath>
#include <cstdio>

namespace {

int g_fails = 0;

void check(const char* name, float got, float want, float tol = 1e-3f) {
    if (std::fabs(got - want) > tol) {
        std::printf("FAIL %s: got %.6f want %.6f\n", name, got, want);
        ++g_fails;
    }
}

void check_int(const char* name, int got, int want) {
    if (got != want) {
        std::printf("FAIL %s: got %d want %d\n", name, got, want);
        ++g_fails;
    }
}

// ──────────────────────────────────────────────
// fake_motor：假电机 + 假 HAL
//   - 静止 / 正弦抖动（对齐测试）
//   - RL 电流模型（v2 CURRENT：dq 域，静止时 angle_elec≈0）
// ──────────────────────────────────────────────
struct FakeMotor {
    float angle = 1.2f;      // 传感器角 [0,2π)
    float wobble = 0.0f;     // 抖动幅度（0 = 静止）
    int   tick_n = 0;
    int   pwm_calls = 0;
    float pwm[3] = {0, 0, 0};
    bool  enabled = false;
    // RL 电流模型（dq 域；u 为相对中心 6V 的相电压）
    bool  current_model = false;
    float i_d = 0.0f, i_q = 0.0f;
    float R = 0.5f, L = 1e-3f;
};

FakeMotor g_motor;

float fake_get_angle(void*) {
    return g_motor.angle + g_motor.wobble * std::sin(0.1f * g_motor.tick_n);
}

float fake_get_current(void*, float* ia, float* ib) {
    if (g_motor.current_model) {
        // dq → αβ（angle≈0）：i_alpha=i_d, i_beta=i_q；αβ → abc（inv_clarke）
        float iu = g_motor.i_d;
        float iv = (1.73205080757f * g_motor.i_q - g_motor.i_d) * 0.5f;
        *ia = iu; *ib = iv;
        return -iu - iv;                       // ic
    }
    *ia = 0.0f; *ib = 0.0f;
    return 0.0f;
}

void fake_set_pwm(void*, float u, float v, float w) {
    g_motor.pwm_calls++;
    g_motor.pwm[0] = u; g_motor.pwm[1] = v; g_motor.pwm[2] = w;
    if (g_motor.current_model) {
        // abc 相电压（去中心 6V）→ αβ → dq（angle≈0）：u_d=u_alpha, u_q=u_beta
        float uu = u - 6.0f, uv = v - 6.0f;
        float u_alpha = uu;
        float u_beta = (uu + 2.0f * uv) * 0.577350269189626f;
        const float dt = 0.00005f;             // 20kHz（与 CURRENT 测试的 modulation_tick 一致）
        g_motor.i_d += (u_alpha - g_motor.R * g_motor.i_d) / g_motor.L * dt;
        g_motor.i_q += (u_beta - g_motor.R * g_motor.i_q) / g_motor.L * dt;
    }
}

void fake_enable(void*, bool on) { g_motor.enabled = on; }

foc::hal::Hardware make_hw() { return {nullptr, fake_get_angle, fake_get_current, fake_set_pwm, fake_enable}; }

// ──────────────────────────────────────────────
// make_cfg：可调对齐参数的 FOC 配置工厂
// ──────────────────────────────────────────────
foc::Config make_cfg(float ramp_time = 0.01f, float max_speed = 0.5f,
                     int samples = 5, float timeout = 0.2f) {
    return foc::Config{
        12.0f, 3.0f, 7, 1,                              // supply, limit, pp, dir
        0.0f,                                           // vel_lpf_tf
        1.0f, ramp_time, max_speed, samples, timeout,   // align
        2.0f, 0.05f,                                    // traj vmax, tf
        foc::algo::PIDConfig{}.kp(0.5f).limit_out(3.0f),                 // angle pid
        foc::algo::PIDConfig{}.kp(0.1f).limit_out(3.0f),                 // vel pid
        foc::algo::PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f),          // iq pid (v2)
        foc::algo::PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f),          // id pid (v2)
        0.0f,                                           // iq_limit（0 = 不限制）
        {0.0f, true},                                   // deadzone {range, soft}
        foc::CtrlMode::VOLTAGE
    };
}

void reset_motor() { g_motor = FakeMotor{}; }

// ──────────────────────────────────────────────
// A. transforms 锚点（D8-1）：uvw → clarke → park → inv_park → inv_clarke 往返
// ──────────────────────────────────────────────
void test_transforms_roundtrip() {
    const float uvw[][3] = {{1.0f, 0.0f, -1.0f}, {0.3f, -0.7f, 0.4f}, {-0.5f, 0.2f, 0.3f}};
    const float angles[] = {0.0f, 0.3f, 1.5f, 3.0f};
    char name[64];
    for (auto& abc : uvw) {
        for (float th : angles) {
            auto ab = foc::transforms::clarke(abc[0], abc[1]);
            auto dq = foc::transforms::park(ab, th);
            auto ab2 = foc::transforms::inv_park(dq, th);
            auto uvw2 = foc::transforms::inv_clarke(ab2);
            std::snprintf(name, sizeof name, "roundtrip u=%.1f th=%.1f", abc[0], th);
            check(name, uvw2.u_, abc[0], 1e-4f);
            check(name, uvw2.v_, abc[1], 1e-4f);
            check(name, uvw2.w_, abc[2], 1e-4f);
        }
    }
}

// ──────────────────────────────────────────────
// B. Deadzone 工具锚点（无状态纯算法；range=0 直通）
// ──────────────────────────────────────────────
void test_deadzone() {
    foc::algo::Deadzone soft(0.1f, true), hard(0.1f, false), off(0.0f, true), def;
    check("dz off 直通", off.calc(0.3f), 0.3f);
    check("dz soft 死区外直通", soft.calc(0.15f), 0.15f);
    check("dz soft 死区内缩放", soft.calc(0.05f), 0.025f);
    check("dz soft 负误差对称", soft.calc(-0.05f), -0.025f);
    check("dz soft 边界直通", soft.calc(0.1f), 0.1f);
    check("dz hard 死区内归零", hard.calc(0.05f), 0.0f);
    check("dz hard 死区外直通", hard.calc(0.15f), 0.15f);
    check("dz 默认构造直通", def.calc(0.2f), 0.2f);
}

// ──────────────────────────────────────────────
// C. FOC 黑盒集成（双入口：pos_tick 算 + modulation_tick 发）
// ──────────────────────────────────────────────

// C1+C2：静止电机 → 对齐成功 → 零位同步 → 闭环稳态有界
void test_align_and_closed_loop() {
    reset_motor();
    foc::FOC foc(make_hw(), make_cfg());
    float target = 0.0f;
    foc.align_and_sync(&target);

    int guard = 0;
    do { foc.pos_tick(target, 0.001f); foc.modulation_tick(0.001f); }
    while (!foc.is_aligned() && ++guard < 200);

    check_int("align 成功", foc.is_aligned(), 1);
    check_int("align 无 fault", foc.fault(), 0);
    check("对齐发波（modulation_tick）", g_motor.pwm_calls > 0 ? 1.0f : 0.0f, 1.0f);

    int pwm_after_align = g_motor.pwm_calls;
    for (int i = 0; i < 100; i++) { foc.pos_tick(target, 0.001f); foc.modulation_tick(0.001f); }
    float dev = std::fmax(std::fabs(g_motor.pwm[0] - 6.0f),
                std::fmax(std::fabs(g_motor.pwm[1] - 6.0f), std::fabs(g_motor.pwm[2] - 6.0f)));
    check("闭环稳态输出有界", dev, 0.0f, 3.0f + 1e-3f);
    check_int("闭环每拍发波", g_motor.pwm_calls, pwm_after_align + 100);
    check("零位同步 angle=raw（绝对展开系，synced_ 后）", foc.angle(), 1.2f, 1e-4f);
}

// C3：锚点 6 级联回归——阶跃目标，级联输出全程有界（clamp 生效）
void test_step_cascade_bounded() {
    reset_motor();
    foc::FOC foc(make_hw(), make_cfg());
    float target = 1.0f;
    foc.align_and_sync(&target);
    int guard = 0;
    do { foc.pos_tick(target, 0.001f); foc.modulation_tick(0.001f); }
    while (!foc.is_aligned() && ++guard < 200);

    float max_dev = 0.0f;
    for (int i = 0; i < 200; i++) {
        foc.pos_tick(target, 0.001f);
        foc.modulation_tick(0.001f);
        for (int c = 0; c < 3; c++)
            max_dev = std::fmax(max_dev, std::fabs(g_motor.pwm[c] - 6.0f));
    }
    check("阶跃级联输出有界", max_dev, 0.0f, 3.0f + 1e-3f);
}

// C4：dt<=0 防御——pos_tick 整拍跳过，PWM 不更新
void test_dt_guard() {
    reset_motor();
    foc::FOC foc(make_hw(), make_cfg());
    foc.pos_tick(0.0f, 0.0f);
    check_int("dt=0 不更新 pwm", g_motor.pwm_calls, 0);
}

// C5：对齐失败透传——抖动电机恒不稳 → 超时 FAULT
void test_align_fault_passthrough() {
    reset_motor();
    g_motor.wobble = 0.1f;
    foc::FOC foc(make_hw(), make_cfg(0.01f, 0.5f, 5, 0.05f));
    float target = 0.0f;
    foc.align_and_sync(&target);

    int guard = 0;
    do { foc.pos_tick(target, 0.001f); g_motor.tick_n++; foc.modulation_tick(0.001f); }
    while (!foc.is_aligned() && ++guard < 200);

    check_int("抖动对齐失败", foc.is_aligned(), 0);
    check_int("fault 透传 SETTLE_TIMEOUT",
              foc.fault(), static_cast<int>(foc::alignment::Fault::SETTLE_TIMEOUT));
}

// C6：enable 透传 + tick_velocity 开环（独立发波，方案 a）
void test_open_loop_and_enable() {
    reset_motor();
    foc::FOC foc(make_hw(), make_cfg());
    foc.enable(true);
    check_int("enable(true) 透传", g_motor.enabled, 1);
    foc.enable(false);
    check_int("enable(false) 透传", g_motor.enabled, 0);

    foc.tick_velocity(2.0f, 1.0f, 0.001f);
    foc.tick_velocity(2.0f, 1.0f, 0.001f);
    check_int("开环发波", g_motor.pwm_calls, 2);
    check("开环不碰角度", foc.angle(), 0.0f, 1e-4f);
}

// C7：v2 CURRENT 集成冒烟——CURRENT 模式全链路跑通（收敛细节见 test_current_loop）
void test_current_smoke() {
    reset_motor();
    g_motor.current_model = true;
    auto cfg = make_cfg();
    cfg.ctrl_mode_ = foc::CtrlMode::CURRENT;
    foc::FOC foc(make_hw(), cfg);

    float target = 2.2f;                          // 绝对角系：对齐位置(1.2) + 前方 1 rad
    foc.align_and_sync(&target);
    int guard = 0;
    do { foc.pos_tick(target, 0.001f); foc.modulation_tick(0.00005f); }
    while (!foc.is_aligned() && ++guard < 200);

    float max_dev = 0.0f;
    for (int i = 0; i < 2000; i++) {              // 0.1s @ 20kHz
        if (i % 20 == 0) foc.pos_tick(target, 0.001f);   // 位置环 1kHz
        foc.modulation_tick(0.00005f);
        for (int c = 0; c < 3; c++)
            max_dev = std::fmax(max_dev, std::fabs(g_motor.pwm[c] - 6.0f));
    }
    check("CURRENT 链路输出有界", max_dev, 0.0f, 3.0f + 1e-3f);
    if (std::fabs(g_motor.i_q) <= 0.001f) {          // 阶跃目标应有激励电流（反向断言）
        std::printf("FAIL CURRENT 电流有激励: got |iq|=%.6f\n", std::fabs(g_motor.i_q));
        ++g_fails;
    }
}

}  // namespace

int main() {
    test_transforms_roundtrip();
    test_deadzone();
    test_align_and_closed_loop();
    test_step_cascade_bounded();
    test_dt_guard();
    test_align_fault_passthrough();
    test_open_loop_and_enable();
    test_current_smoke();

    std::printf("=== %s (test_foc, %d fails) ===\n", g_fails ? "FAILED" : "ALL PASS", g_fails);
    return g_fails;
}
