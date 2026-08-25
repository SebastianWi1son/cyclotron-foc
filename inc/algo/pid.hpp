#pragma once

#include "algo/lpf.hpp"
#include "algo/ramp.hpp"

// 来源：lunokhod control/wheel/inc/pid.hpp（复用搬运，行为不变）
// 工业级 PID：微分先行（无微分冲击）+ 梯形积分 + 积分分离 + 抗饱和 + 输出斜坡
// 规格：FOC_MATH_SPEC.md §3.4（含与 legacy 新旧两版 pid 的关系说明）

namespace foc::algo {

struct PIDConfig {
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    float limit_out_ = 0.0f;
    // --- i_term method property ---
    float limit_i_ = 0.0f;
    float thresh_i_sep_ = 0.0f;
    // --- dsp tools property ---
    float max_rate_out_ = 0.0f;
    float d_filter_Tf_ = 0.0f;
};

class PID {
public:
    explicit PID(const PIDConfig &cfg);     // 显式确保PIDConfig作为参数参与构造
    float calc(float cmd, float measure, float dt);
    void reset();
private:
    // ----- Math Tools -----
    static float fabs(float val);
    static float constrainf(float val, float limit);

    // --- property ---
    PIDConfig cfg_;
    float integral_;
    float error_prev_;
    float measure_prev_;
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
};

}  // namespace foc::algo
