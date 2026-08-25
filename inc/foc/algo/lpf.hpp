#pragma once

// 来源：lunokhod control/wheel/inc/lpf.hpp（复用搬运，行为不变）
// 一阶低通滤波器：alpha = dt/(Tf+dt)；Tf=0 → 直通（0=disabled 语义）
// 规格：FOC_MATH_SPEC.md §3.1

namespace foc::algo {

class LPF {
public:
    LPF(float Tf);
    float calc(float raw, float dt);
    void reset();
    // 状态注入（bumpless transfer）：prev_ = x，后续 calc 从 x 连续起步
    void set_state(float x);
private:
    float Tf_;
    float prev_;
};

}  // namespace foc::algo
