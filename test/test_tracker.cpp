// test_tracker — 锚点 3（多圈回绕）+ 锚点 5（速度估计）（D8）
// 黑盒测试：只经公开 API（Tracker 公开方法），不依赖/不复用 tracker 实现细节
// enc_fake：独立编码器模拟器（恒定角速度 → raw 物理角 [0,2π)），可变换极性
// 工程约束：-Wall -Wextra -Werror + ASan/UBSan；退出码 = 失败数

#include "angle_tracking.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

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

constexpr float k2PI = 6.28318530717958647692f;

// ──────────────────────────────────────────────
// enc_fake：编码器模拟器（独立实现，非 tracker 代码）
// omega > 0 正转（raw 递增回绕）；omega < 0 反转
// ──────────────────────────────────────────────
class EncFake {
public:
    EncFake(float omega, float dt, float start = 0.0f) : omega_(omega), dt_(dt), raw_(start) {}
    float next() {                       // 推进 dt，返回 raw 物理角 [0, 2π)
        raw_ += omega_ * dt_;
        raw_ = std::fmod(raw_, k2PI);
        if (raw_ < 0.0f) raw_ += k2PI;
        return raw_;
    }
private:
    float omega_;
    float dt_;
    float raw_;
};

// 采集一段正/反转运行，返回 (abs 序列, velocity 末段均值, full_rotations 终值)
struct RunResult { std::vector<float> abs; float vel_last; int rotations; };

RunResult run_tracker(float omega, float dt, float seconds, float lpf_tf = 0.0f) {
    foc::angle_tracking::Config cfg{lpf_tf};
    foc::angle_tracking::Tracker tr(cfg);
    tr.reset(0.0f);
    EncFake enc(omega, dt, 1.0f);        // 起始 1.0 rad：避开 raw=0 边界（起始跨圈属编码器边界效应，不测）
    RunResult r;
    const int steps = static_cast<int>(seconds / dt);
    float vsum = 0.0f;
    for (int i = 0; i < steps; ++i) {
        tr.update(enc.next(), dt);
        r.abs.push_back(tr.angle_abs());
        if (i >= steps - 100) vsum += tr.velocity();   // 末段 100 步均值
    }
    r.vel_last = vsum / 100.0f;
    r.rotations = tr.full_rotations();
    return r;
}

// ──────────────────────────────────────────────
// 锚点 3：多圈回绕
// ──────────────────────────────────────────────
void test_wraparound() {
    const float dt = 0.001f;

    // ① 正转多圈：abs 单调递增连续（无 2π 跳变），full_rotations 为正且与 abs 自洽
    //    omega=6.0, 6s，start=1.0 → abs 终值 ≈ 37.0（≈ 5.9 圈）
    auto fwd = run_tracker(6.0f, dt, 6.0f);
    check("fwd abs end", fwd.abs.back(), 37.0f, 0.02f);
    if (!(fwd.rotations > 0)) { std::printf("FAIL fwd rotation polarity: %d\n", fwd.rotations); ++g_fails; }
    // 自洽：full_rotations·2π ≤ abs < (full_rotations+1)·2π
    float lo = fwd.rotations * k2PI, hi = (fwd.rotations + 1) * k2PI;
    if (!(fwd.abs.back() >= lo - 1e-3f && fwd.abs.back() < hi + 1e-3f)) {
        std::printf("FAIL fwd self-consistency: rot=%d abs=%.4f range=[%.4f,%.4f)\n",
                    fwd.rotations, fwd.abs.back(), lo, hi);
        ++g_fails;
    }
    bool monotonic = true;
    for (size_t i = 1; i < fwd.abs.size(); ++i) {
        float step = fwd.abs[i] - fwd.abs[i - 1];
        if (step <= 0.0f || step > 0.5f) {   // 正常步进 0.006；任何跳变/回退即失败
            std::printf("FAIL fwd continuity @%zu: step=%.4f\n", i, step);
            monotonic = false;
            break;
        }
    }
    if (monotonic) { /* ok */ } else { ++g_fails; }

    // ② 反转多圈：abs 单调递减连续，full_rotations 为负且自洽（start=1.0 → 终值 ≈ -35.0）
    auto rev = run_tracker(-6.0f, dt, 6.0f);
    check("rev abs end", rev.abs.back(), -35.0f, 0.02f);
    if (!(rev.rotations < 0)) { std::printf("FAIL rev rotation polarity: %d\n", rev.rotations); ++g_fails; }
    lo = rev.rotations * k2PI; hi = (rev.rotations + 1) * k2PI;
    if (!(rev.abs.back() >= lo - 1e-3f && rev.abs.back() < hi + 1e-3f)) {
        std::printf("FAIL rev self-consistency: rot=%d abs=%.4f range=[%.4f,%.4f)\n",
                    rev.rotations, rev.abs.back(), lo, hi);
        ++g_fails;
    }
    monotonic = true;
    for (size_t i = 1; i < rev.abs.size(); ++i) {
        float step = rev.abs[i] - rev.abs[i - 1];
        if (step >= 0.0f || step < -0.5f) {
            std::printf("FAIL rev continuity @%zu: step=%.4f\n", i, step);
            monotonic = false;
            break;
        }
    }
    if (monotonic) { /* ok */ } else { ++g_fails; }

    // ③ 跳变判据边界（0.8·2π）：每个场景独立 tracker（避免序列污染 raw_prev）
    foc::angle_tracking::Config cfg{0.0f};
    // ③a 步长 0.7·2π：不触发
    {
        foc::angle_tracking::Tracker tr(cfg);
        tr.reset(0.3f);
        tr.update(0.3f + 0.7f * k2PI, dt);
        check_int("thresh no-trigger rot", tr.full_rotations(), 0);
        check("thresh no-trigger abs", tr.angle_abs(), 0.3f + 0.7f * k2PI);
    }
    // ③b 反向回绕：raw 0.3 → 5.655（d=+5.355 > 5.027）→ 计数 -1
    {
        foc::angle_tracking::Tracker tr(cfg);
        tr.reset(0.3f);
        tr.update(5.655f, dt);
        check_int("thresh neg-trigger rot", tr.full_rotations(), -1);
        check("thresh neg-trigger abs", tr.angle_abs(), -k2PI + 5.655f);
    }
    // ③c 正向回绕：raw 5.9 → 0.2（d=-5.7 < -5.027）→ 计数 +1
    {
        foc::angle_tracking::Tracker tr(cfg);
        tr.reset(5.9f);
        tr.update(0.2f, dt);
        check_int("thresh pos-trigger rot", tr.full_rotations(), 1);
        check("thresh pos-trigger abs", tr.angle_abs(), k2PI + 0.2f);
    }
}

// ──────────────────────────────────────────────
// 锚点 5：速度估计
// ──────────────────────────────────────────────
void test_velocity() {
    const float dt = 0.001f;

    // ④ 恒速收敛：omega=2.0, LPF Tf=0.05，5s 后收敛到 ≈2.0
    auto conv = run_tracker(2.0f, dt, 5.0f, 0.05f);
    check("vel converge", conv.vel_last, 2.0f, 0.15f);

    // ⑤ 零速无漂移：跑 1s，velocity ≈ 0，abs 恒定（= enc 起始角 1.0）
    auto idle = run_tracker(0.0f, dt, 1.0f);
    check("vel idle", idle.vel_last, 0.0f, 1e-3f);
    check("abs idle", idle.abs.back(), 1.0f, 1e-3f);

    // ⑥ 极性：正转速度 > 0，反转速度 < 0（不同极性编码器）
    auto fwd = run_tracker(3.0f, dt, 2.0f);
    auto rev = run_tracker(-3.0f, dt, 2.0f);
    if (!(fwd.vel_last > 0.0f)) { std::printf("FAIL polarity fwd: vel=%.4f\n", fwd.vel_last); ++g_fails; }
    if (!(rev.vel_last < 0.0f)) { std::printf("FAIL polarity rev: vel=%.4f\n", rev.vel_last); ++g_fails; }
    check("polarity |vel|", std::fabs(fwd.vel_last), std::fabs(rev.vel_last), 0.15f);
}

// ──────────────────────────────────────────────
// 快照一致性（现算语义）+ reset（对齐同步语义）
// ──────────────────────────────────────────────
void test_snapshot_and_reset() {
    const float dt = 0.001f;
    foc::angle_tracking::Config cfg{0.0f};
    foc::angle_tracking::Tracker tr(cfg);

    // ⑦ 快照一致性：update 一次后，同一周期内多次 angle_abs() 全部相等
    tr.reset(1.0f);
    tr.update(1.5f, dt);
    float a0 = tr.angle_abs();
    bool consistent = true;
    for (int i = 0; i < 10; ++i)
        if (tr.angle_abs() != a0) { consistent = false; break; }
    if (!consistent) { std::printf("FAIL snapshot consistency\n"); ++g_fails; }

    // ⑧ reset 语义（对齐成功后同步）：abs 置为传入角、速度清零、圈数清零
    tr.reset(2.5f);
    check("reset abs", tr.angle_abs(), 2.5f);
    check("reset vel", tr.velocity(), 0.0f);
    check_int("reset rot", tr.full_rotations(), 0);
    // reset 后首拍速度正常（无 0 起步爆表）：2.5 → 2.6，1 拍 dt=0.1 → v=1.0
    tr.update(2.6f, 0.1f);
    check("reset first vel", tr.velocity(), 1.0f, 0.01f);
}

}  // namespace

int main() {
    test_wraparound();
    test_velocity();
    test_snapshot_and_reset();

    if (g_fails == 0) {
        std::printf("=== ALL PASS (锚点 3/5, angle_tracking) ===\n");
        return 0;
    }
    std::printf("=== %d FAILS ===\n", g_fails);
    return g_fails;
}
