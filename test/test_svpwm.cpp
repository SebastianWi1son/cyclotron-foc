// test_svpwm — 锚点 2：SVPWM 语义测试（D8）
// 目标语义 = 修复后：modulate 含零序注入 v0 = -(max+min)/2（carrier-based SVPWM）
//   [共同] = SPWM/SVPWM 都应成立
//   [目标] = SVPWM 特有语义——当前实现（纯中心偏置 = SPWM）预期红，修复后绿
// 工程约束：-Wall -Wextra -Werror + ASan/UBSan；退出码 = 失败数

#include "svpwm.hpp"
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

constexpr float SQRT3   = 1.73205080757f;
constexpr float TWO_PI  = 6.28318530718f;

// ──────────────────────────────────────────────
// [共同] 语义：SPWM 与 SVPWM 都成立
// ──────────────────────────────────────────────

void test_common_invariants() {
    foc::svpwm::Config cfg{1.0f, 12.0f};
    const float center = cfg.voltage_supply_ * 0.5f;

    // 零输出 → 三相全 center
    foc::transforms::ThreePhase z = foc::svpwm::calc(cfg, {0.0f, 0.0f}, 1.0f);
    check("zero u", z.u_, center);
    check("zero v", z.v_, center);
    check("zero w", z.w_, center);

    // 限幅等价：dq.q 超 limit 与恰好 limit 输出一致
    foc::transforms::ThreePhase over = foc::svpwm::calc(cfg, {0.0f, 5.0f}, 0.0f);
    foc::transforms::ThreePhase at   = foc::svpwm::calc(cfg, {0.0f, 1.0f}, 0.0f);
    check("clamp u", over.u_, at.u_);
    check("clamp v", over.v_, at.v_);
    check("clamp w", over.w_, at.w_);

    // limit=0 安全默认 → 全 center（0 = 不输出）
    foc::svpwm::Config zl{0.0f, 12.0f};
    foc::transforms::ThreePhase z2 = foc::svpwm::calc(zl, {0.0f, 1.0f}, 0.5f);
    check("limit0 u", z2.u_, center);
    check("limit0 v", z2.v_, center);
    check("limit0 w", z2.w_, center);

    // 角度 wrap：θ 与 θ+2π·k 输出一致（多圈等价）
    foc::transforms::ThreePhase w0 = foc::svpwm::calc(cfg, {0.0f, 1.0f}, 0.0f);
    foc::transforms::ThreePhase w1 = foc::svpwm::calc(cfg, {0.0f, 1.0f}, TWO_PI * 3.0f);
    check("wrap u", w1.u_, w0.u_, 1e-4f);
    check("wrap v", w1.v_, w0.v_, 1e-4f);
    check("wrap w", w1.w_, w0.w_, 1e-4f);

    // 线电压 ≤ 2·limit（VOLTAGE 路径 d≡0，全角度；零序不影响线电压，两种实现都成立）
    for (float th = 0.0f; th < TWO_PI; th += 0.1f) {
        foc::transforms::ThreePhase v = foc::svpwm::calc(cfg, {0.0f, 1.0f}, th);
        float l1 = std::fabs(v.u_ - v.v_);
        float l2 = std::fabs(v.v_ - v.w_);
        float l3 = std::fabs(v.w_ - v.u_);
        if (l1 > 2.0f + 1e-3f || l2 > 2.0f + 1e-3f || l3 > 2.0f + 1e-3f) {
            std::printf("FAIL line-voltage @th=%.2f: %.4f %.4f %.4f\n", th, l1, l2, l3);
            ++g_fails;
            break;
        }
    }

    // modulate 不改变线电压（中心偏置/零序的公共性质）
    foc::transforms::ThreePhase ac{0.8f, -1.0f, 0.2f};   // 交流分量，和为 0
    foc::transforms::ThreePhase out = foc::svpwm::modulate(cfg, ac);
    check("modulate line uv", out.u_ - out.v_, ac.u_ - ac.v_);
    check("modulate line vw", out.v_ - out.w_, ac.v_ - ac.w_);
    check("modulate line wu", out.w_ - out.u_, ac.w_ - ac.u_);
}

// ──────────────────────────────────────────────
// [目标] SVPWM 语义：当前 SPWM 实现预期红，修复后绿
// ──────────────────────────────────────────────

void test_svpwm_target() {
    const float supply = 12.0f;
    const float limit  = supply / SQRT3;   // 6.9282 = SVPWM 线性区上限（Vdc/√3）
    foc::svpwm::Config cfg{limit, supply};
    const float center = supply * 0.5f;

    // ① 线性区上边界不饱和：dq={0, Vdc/√3} 全角度输出 ∈ [0, Vdc]
    //    数学：零序注入后调制波峰值 = (max-min)/2 = √3·A/2 = Vdc/2 = center
    //    SPWM（无注入）：峰值 = A = 6.93 > 6 → 输出越界（削波）→ 红
    bool unsaturated = true;
    for (float th = 0.0f; th < TWO_PI; th += 0.05f) {
        foc::transforms::ThreePhase r = foc::svpwm::calc(cfg, {0.0f, limit}, th);
        if (r.u_ < -1e-2f || r.u_ > supply + 1e-2f ||
            r.v_ < -1e-2f || r.v_ > supply + 1e-2f ||
            r.w_ < -1e-2f || r.w_ > supply + 1e-2f) {
            std::printf("FAIL [target] saturation @th=%.2f: %.4f %.4f %.4f\n",
                        th, r.u_, r.v_, r.w_);
            unsaturated = false;
            break;
        }
    }
    if (!unsaturated) ++g_fails;

    // ② 零序注入对称化：max(u-center) + min(u-center) ≈ 0
    //    min-max 注入把调制波压成关于 0 对称；SPWM 下 max/min 不对称 → 红
    foc::transforms::ThreePhase r = foc::svpwm::calc(cfg, {0.0f, limit}, 1.0f);
    float mx = std::fmax(r.u_, std::fmax(r.v_, r.w_)) - center;
    float mn = std::fmin(r.u_, std::fmin(r.v_, r.w_)) - center;
    check("[target] zero-seq symmetry", mx + mn, 0.0f, 1e-2f);

    // ③ 峰值对比（信息性，不作失败依据）：修复后峰值应显著 < 输入幅值
    float pk = std::fmax(std::fabs(r.u_ - center),
                         std::fmax(std::fabs(r.v_ - center), std::fabs(r.w_ - center)));
    std::printf("info: peak deviation = %.4f V (input amplitude %.4f V)\n", pk, limit);
}

}  // namespace

int main() {
    test_common_invariants();
    test_svpwm_target();

    if (g_fails == 0) {
        std::printf("=== ALL PASS (锚点 2, SVPWM 语义) ===\n");
        return 0;
    }
    std::printf("=== %d FAILS ===\n", g_fails);
    return g_fails;
}
