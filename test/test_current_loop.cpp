// test_current_loop — v2 CURRENT 电流环组件锚点测试（D-A1 独立组件独立验证）
// 黑盒：直接构造 current_loop::CurrentLoop（不经 FOC、不对齐）
//        + dq 域 RL 电机模型（angle_elec=0 假设：dq↔abc 无旋转，ia 与 i_d 同相）
// 工程约束：-Wall -Wextra -Werror + ASan/UBSan；退出码 = 失败数

#include "current_loop.hpp"

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

foc::algo::PIDConfig make_pid(float kp = 1.0f, float ki = 50.0f) {
    return foc::algo::PIDConfig{}.kp(kp).ki(ki).limit_out(3.0f).limit_i(3.0f);
}

// ──────────────────────────────────────────────
// RlPlant：dq 域一阶 RL 电机模型（独立实现，非 CurrentLoop 代码）
//   u_d = L·di_d/dt + R·i_d；u_q = L·di_q/dt + R·i_q（静止，无反电动势）
//   tau = L/R = 2ms
// ──────────────────────────────────────────────
struct RlPlant {
    float i_d = 0.0f, i_q = 0.0f;
    const float R = 0.5f, L = 1e-3f;
    void step(float ud, float uq, float dt) {
        i_d += (ud - R * i_d) / L * dt;
        i_q += (uq - R * i_q) / L * dt;
    }
    void sample(float* ia, float* ib) const {
        // dq(angle=0) → αβ：i_alpha=i_d, i_beta=i_q；αβ → abc（inv_clarke）
        *ia = i_d;
        *ib = (1.73205080757f * i_q - i_d) * 0.5f;
    }
};

// 闭环跑 N 拍：CurrentLoop 输出 {ud,uq} → plant 步进 → 采样回喂
void run(foc::current_loop::CurrentLoop& loop, RlPlant& plant,
         float iq_ref, int n, float dt = 1e-4f) {
    for (int i = 0; i < n; i++) {
        float ia = 0.0f, ib = 0.0f;
        plant.sample(&ia, &ib);
        auto udq = loop.update(ia, ib, 0.0f, iq_ref, dt);
        plant.step(udq.d_, udq.q_, dt);
    }
}

// ──────────────────────────────────────────────
// 用例
// ──────────────────────────────────────────────

// 1. 正目标跟踪：Iq 收敛 1.0A，Id 压 0
void test_iq_tracking() {
    foc::current_loop::CurrentLoop loop(make_pid(), make_pid());
    RlPlant plant;
    run(loop, plant, 1.0f, 5000);                 // 0.5s（tau=2ms，远够收敛）
    check("Iq 收敛 1.0A", plant.i_q, 1.0f, 0.02f);
    check("Id 压 0", plant.i_d, 0.0f, 0.02f);
}

// 2. 负目标：Iq 收敛 -1.0A（方向对称）
void test_negative_target() {
    foc::current_loop::CurrentLoop loop(make_pid(), make_pid());
    RlPlant plant;
    run(loop, plant, -1.0f, 5000);
    check("Iq 收敛 -1.0A", plant.i_q, -1.0f, 0.02f);
    check("Id 压 0", plant.i_d, 0.0f, 0.02f);
}

// 3. 初始 Id 扰动被压回（d 轴调节能力）
void test_id_disturbance_rejected() {
    foc::current_loop::CurrentLoop loop(make_pid(), make_pid());
    RlPlant plant;
    plant.i_d = 0.5f;                             // 注入 d 轴扰动
    run(loop, plant, 0.5f, 5000);
    check("Id 扰动收敛 0", plant.i_d, 0.0f, 0.02f);
    check("Iq 收敛 0.5A", plant.i_q, 0.5f, 0.02f);
}

// 4. 零目标：电流衰减到 0
void test_zero_target_decay() {
    foc::current_loop::CurrentLoop loop(make_pid(), make_pid());
    RlPlant plant;
    plant.i_q = 0.8f;                             // 预充电
    run(loop, plant, 0.0f, 3000);
    check("Iq 衰减 0", plant.i_q, 0.0f, 0.02f);
}

// 5. reset：积分清零后仍能重新收敛
void test_reset_restarts() {
    foc::current_loop::CurrentLoop loop(make_pid(), make_pid());
    RlPlant plant;
    run(loop, plant, 0.0f, 1000);                 // 先稳定在 0
    loop.reset();
    run(loop, plant, 1.0f, 5000);
    check("reset 后重新收敛", plant.i_q, 1.0f, 0.02f);
}

}  // namespace

int main() {
    test_iq_tracking();
    test_negative_target();
    test_id_disturbance_rejected();
    test_zero_target_decay();
    test_reset_restarts();

    std::printf("=== %s (test_current_loop, %d fails) ===\n",
                g_fails ? "FAILED" : "ALL PASS", g_fails);
    return g_fails;
}
