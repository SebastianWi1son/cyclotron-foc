#pragma once

#include "ctl/pid_types.hpp"
#include "ctl/lpf.hpp"
#include "ctl/ramp.hpp"

#if defined(__cplusplus) && __cplusplus >= 201703L
#  define CTL_NODISCARD [[nodiscard]]
#elif defined(__GNUC__) || defined(__clang__)
#  define CTL_NODISCARD __attribute__((warn_unused_result))
#else
#  define CTL_NODISCARD
#endif

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::PID（其自身复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/pid.md ｜ 优化路线：docs/roadmap.md
// 工业级 PID：微分先行（无微分冲击）+ 梯形积分 + 积分分离 + 抗饱和 + 输出斜坡
// 支撑类型（PIDConfig / PIDPorts / PIDState / PIDStatus）在 ctl/pid_types.hpp

namespace ctl {

class PID {
public:
    explicit PID(const PIDConfig &cfg);     // 显式确保PIDConfig作为参数参与构造
    CTL_NODISCARD float calc(float cmd, float measure, float dt, const PIDPorts *ports = nullptr);
    void reset();
    void set_integral(float x);
    void set_gains(const PIDGains &g);
    const PIDState &get_state() const;
    PIDStatus status() const { return status_; }
    bool input_fault() const { return input_fault_; }   // 粘滞：置位后保持到 reset()（与 status() 的本拍瞬态不同）

private:
    // ----- Math Tools -----
    static float fabs(float val);
    static float constrainf(float val, float limit);
    static bool is_finite(float x);
    // --- property ---
    PIDConfig cfg_;
    float integral_;
    float error_prev_;
    float measure_prev_;
    float last_output_;
    // --- observe ---
    PIDState state_;
    PIDStatus status_;
    bool input_fault_ = false;   // 粘滞 fault（A 方案拆出）：只有 reset() 清
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
};

}  // namespace ctl
