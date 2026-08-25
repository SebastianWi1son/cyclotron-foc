#pragma once

// 来源：lunokhod control/wheel/inc/ramp.hpp（复用搬运，行为不变）
// 斜率限制器：每帧 clamp 到 [prev ± max_rate·dt]；max_rate=0 → 冻结输出
// 规格：FOC_MATH_SPEC.md §3.2

namespace foc::algo {

class Ramp {
public:
    Ramp(float max_rate);
    float calc(float cmd, float dt);
    void reset();
    // 状态注入（bumpless transfer）：prev_ = x，后续 calc 从 x 连续起步
    void set_state(float x);
private:
    float max_rate_;
    float prev_;
};

}  // namespace foc::algo
