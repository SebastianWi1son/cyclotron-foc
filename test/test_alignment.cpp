// test_alignment — 对齐状态机全路径 + 健壮性（D8-4 锚点）
// 黑盒：只经公开 API（Aligner 公开方法）；波形对比用公开组件 svpwm::write 独立计算
// 覆盖：初始态/RAMP 斜坡与波形/RAMP→SETTLE 转移/SETTLE 判稳（通过·清零·超时）/
//       零位极性（dir/pp）/LOCKED 零输出/abort 复位/故障恢复/settle_samples 边界
// 工程约束：-Wall -Wextra -Werror + ASan/UBSan；退出码 = 失败数

#include "alignment.hpp"
#include "svpwm.hpp"

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
void check_state(const char* name, foc::alignment::State got, foc::alignment::State want) {
    if (got != want) {
        std::printf("FAIL %s: got %d want %d\n", name, static_cast<int>(got), static_cast<int>(want));
        ++g_fails;
    }
}
void check_fault(const char* name, foc::alignment::Fault got, foc::alignment::Fault want) {
    if (got != want) {
        std::printf("FAIL %s: got %d want %d\n", name, static_cast<int>(got), static_cast<int>(want));
        ++g_fails;
    }
}
void check_true(const char* name, bool cond) {
    if (!cond) { std::printf("FAIL %s: expected true\n", name); ++g_fails; }
}

constexpr float kDT = 0.001f;         // 1kHz
constexpr float k3PI_2 = 4.71238898038f;

// 默认测试参数：ramp 10ms；判稳 2.0 rad/s × 2 次；超时 0.5s
// 注：Aligner 构造目前为非 const Config&（审阅项），测试用左值局部变量绑定
foc::alignment::Config make_cfg(float ramp_time = 0.01f, float max_speed = 2.0f,
                                int samples = 2, float timeout = 0.5f) {
    return {3.0f, 12.0f, ramp_time, max_speed, samples, timeout};
}

// 喂 N tick 相同 raw（静止）。N 取大余量：float 累加 10×0.001f 可能 < 0.01f，
// 不数精确 tick（浮点边缘），20 tick（0.020s）必已跨过 RAMP→SETTLE 转移
foc::alignment::TickResult feed_still(foc::alignment::Aligner& a, float raw, int n = 20) {
    foc::alignment::TickResult r{};
    for (int i = 0; i < n; ++i) r = a.tick(raw, kDT);
    return r;
}

// ──────────────────────────────────────────────
void test_initial_state() {
    auto cfg = make_cfg();
    foc::alignment::Aligner a(cfg, 7, 1);
    auto r = a.tick(1.0f, kDT);
    check_state("init state", r.state_, foc::alignment::State::IDLE);
    check("init zero wave u", r.u_, 0.0f);
    check("init zero wave v", r.v_, 0.0f);
    check("init zero wave w", r.w_, 0.0f);
    check_true("init not locked", !a.is_locked());
    check("init zero offset", a.zero_offset_elec(), 0.0f);
    check_fault("init fault none", a.fault(), foc::alignment::Fault::NONE);
}

// RAMP：波形与 svpwm::write 独立计算逐 tick 对比（k = n·dt/ramp_time）；转移 tick 波为满电压
void test_ramp_waveform_and_transition() {
    const float ramp_time = 0.01f;
    auto cfg = make_cfg(ramp_time);
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    check_state("ramp entry", a.tick(0.0f, kDT).state_, foc::alignment::State::RAMP);

    int n = 1;
    foc::alignment::TickResult r{};
    for (; n < 200; ++n) {
        r = a.tick(0.0f, kDT);
        if (r.state_ != foc::alignment::State::RAMP) break;
        // 注意：转移 tick 返回 RAMP 快照（do_ramp 构造 r 时 state_ 尚未改），break 发生在 SETTLE 首拍
        float k = (n + 1) * kDT / ramp_time;       // 第 n 次循环 = 总第 (n+1) 次 tick（ramp entry 已 tick 一次）
        auto want = foc::svpwm::calc({3.0f, 12.0f}, {0.0f, 3.0f * k}, k3PI_2);
        check("ramp wave u", r.u_, want.u_, 1e-3f);
        check("ramp wave v", r.v_, want.v_, 1e-3f);
        check("ramp wave w", r.w_, want.w_, 1e-3f);
    }
    check_state("ramp→settle", r.state_, foc::alignment::State::SETTLE);
    // 满电压波（k=1）已在循环最后对比覆盖（n=9 时 k=10·dt/ramp_time=1.0）；break 后 r 为 SETTLE 首拍零波
    check("settle first zero u", r.u_, 0.0f);
}

// SETTLE 判稳通过 → LOCKED；零位 = raw·pp·dir（dir 极性 ×2 + pp 倍数）
void test_settle_locked_and_zero_offset() {
    const float raw = 1.234f;
    // dir=+1, pp=7
    {
        auto cfg = make_cfg();
        foc::alignment::Aligner a(cfg, 7, 1);
        a.start();
        feed_still(a, raw);
        auto r = feed_still(a, raw, 2);            // SETTLE 稳 2 次 → LOCKED
        check_state("locked dir+", r.state_, foc::alignment::State::LOCKED);
        check_true("locked flag", a.is_locked());
        check("zero offset dir+", a.zero_offset_elec(), raw * 7);
        check("locked zero wave u", r.u_, 0.0f);
    }
    // dir=-1, pp=7：零位取负
    {
        auto cfg = make_cfg();
        foc::alignment::Aligner a(cfg, 7, -1);
        a.start();
        feed_still(a, raw);
        feed_still(a, raw, 2);
        check("zero offset dir-", a.zero_offset_elec(), raw * 7 * (-1));
    }
    // pp=1, dir=1
    {
        auto cfg = make_cfg();
        foc::alignment::Aligner a(cfg, 1, 1);
        a.start();
        feed_still(a, raw);
        feed_still(a, raw, 2);
        check("zero offset pp1", a.zero_offset_elec(), raw);
    }
}

// 判稳连续性：A) 连续 2 次稳定即 LOCKED；B) 跳变清零——稳 1 次后跳变，再稳 1 次仍 SETTLE（清零生效），再稳 1 次才 LOCKED
void test_settle_requires_consecutive() {
    auto cfg = make_cfg();
    // A：连续 2 次 → LOCKED
    {
        foc::alignment::Aligner a(cfg, 1, 1);
        a.start();
        // 推进到 SETTLE：转移 tick 返回 RAMP 快照，break 在 SETTLE 首拍（首拍 d=0 已稳 1 次）
        foc::alignment::TickResult r{};
        int guard = 0;
        do { r = a.tick(1.0f, kDT); } while (r.state_ == foc::alignment::State::RAMP && ++guard < 200);
        check_state("settle entered", r.state_, foc::alignment::State::SETTLE);
        r = a.tick(1.0f, kDT);                     // 稳第 2 次 → count=2 → LOCKED（转移在 tick 内，断言用实时 getter）
        check_true("locked after 2", a.is_locked());
    }
    // B：跳变清零（关键区分断言：跳变后只稳 1 次必须不 LOCKED）
    {
        foc::alignment::Aligner a(cfg, 1, 1);
        a.start();
        foc::alignment::TickResult r{};
        int guard = 0;
        do { r = a.tick(1.0f, kDT); } while (r.state_ == foc::alignment::State::RAMP && ++guard < 200);
        r = a.tick(1.5f, kDT);                     // 跳变 +0.5 rad > 2.0·0.001 → 清零
        check_state("jump stays settle", r.state_, foc::alignment::State::SETTLE);
        r = a.tick(1.5f, kDT);                     // 稳 1 次（count=1）→ 未达 samples → 仍 SETTLE
        check_state("jump+1 settle", r.state_, foc::alignment::State::SETTLE);
        r = a.tick(1.5f, kDT);                     // 稳 2 次 → LOCKED（转移拍：断言实时 getter）
        check_true("jump+2 locked", a.is_locked());
        check("jump+2 offset", a.zero_offset_elec(), 1.5f);
    }
}

// 超时：进入 SETTLE 后持续不稳（每 tick 大步长）→ FAULT(SETTLE_TIMEOUT)；输出 0
void test_settle_timeout_fault() {
    auto cfg = make_cfg(0.01f, 2.0f, 2, 0.05f);
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    // 推进到 SETTLE（转移 tick 返回 RAMP 快照；break 在 SETTLE 首拍，d=0 稳 1 次但不达标）
    foc::alignment::TickResult r{};
    int guard = 0;
    do { r = a.tick(0.0f, kDT); } while (r.state_ == foc::alignment::State::RAMP && ++guard < 200);
    // 之后每 tick +0.5 rad（持续不稳）跑 60ms > 50ms 超时
    for (int i = 0; i < 60; ++i) r = a.tick(0.5f * (i + 1), kDT);
    check_fault("timeout code", a.fault(), foc::alignment::Fault::SETTLE_TIMEOUT);
    check_true("timeout not locked", !a.is_locked());
    r = a.tick(0.5f * 61, kDT);                    // 下一拍 state 已是 FAULT
    check_state("timeout fault", r.state_, foc::alignment::State::FAULT);
    check("fault zero wave u", r.u_, 0.0f);
}

// LOCKED 后持续零输出（喂什么都零波）
void test_locked_steady_zero() {
    auto cfg = make_cfg();
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    feed_still(a, 1.0f);
    feed_still(a, 1.0f, 2);
    auto r = feed_still(a, 2.0f, 5);
    check_state("locked steady", r.state_, foc::alignment::State::LOCKED);
    check("locked zero u", r.u_, 0.0f);
    check("locked zero v", r.v_, 0.0f);
    check("locked zero w", r.w_, 0.0f);
}

// abort：RAMP 中急停 → IDLE + 零波 + fault 清
void test_abort() {
    auto cfg = make_cfg();
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    feed_still(a, 0.0f, 3);
    a.abort();
    auto r = a.tick(0.0f, kDT);
    check_state("abort idle", r.state_, foc::alignment::State::IDLE);
    check("abort zero u", r.u_, 0.0f);
    check_fault("abort fault none", a.fault(), foc::alignment::Fault::NONE);
}

// 故障恢复：FAULT 后 start 重新对齐 → 再次 LOCKED
void test_restart_from_fault() {
    auto cfg = make_cfg(0.01f, 2.0f, 2, 0.05f);
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    foc::alignment::TickResult r{};
    int guard = 0;
    do { r = a.tick(0.0f, kDT); } while (r.state_ == foc::alignment::State::RAMP && ++guard < 200);
    for (int i = 0; i < 60; ++i) a.tick(0.5f * (i + 1), kDT);
    check_fault("pre restart fault", a.fault(), foc::alignment::Fault::SETTLE_TIMEOUT);

    a.start();                               // 故障后重新对齐
    check_state("restart ramp", a.tick(0.0f, kDT).state_, foc::alignment::State::RAMP);
    feed_still(a, 2.0f);                     // RAMP 结束（基准 2.0）
    feed_still(a, 2.0f, 2);                  // 稳 2 次 → LOCKED
    check_true("restart locked", a.is_locked());
    check("restart zero offset", a.zero_offset_elec(), 2.0f);
    check_fault("restart fault none", a.fault(), foc::alignment::Fault::NONE);
}

// 边界：settle_samples=1 → 一次稳定即 LOCKED
void test_samples_one() {
    auto cfg = make_cfg(0.01f, 2.0f, 1);
    foc::alignment::Aligner a(cfg, 1, 1);
    a.start();
    feed_still(a, 1.0f);
    auto r = a.tick(1.0f, kDT);              // SETTLE 首拍即 LOCKED（转移拍：断言实时 getter）
    check_true("samples1 locked", a.is_locked());
    check("samples1 offset", a.zero_offset_elec(), 1.0f);
    (void)r;
}

}  // namespace

int main() {
    test_initial_state();
    test_ramp_waveform_and_transition();
    test_settle_locked_and_zero_offset();
    test_settle_requires_consecutive();
    test_settle_timeout_fault();
    test_locked_steady_zero();
    test_abort();
    test_restart_from_fault();
    test_samples_one();

    if (g_fails == 0) {
        std::printf("=== ALL PASS (alignment 状态机全路径) ===\n");
        return 0;
    }
    std::printf("=== %d FAILS ===\n", g_fails);
    return g_fails;
}
