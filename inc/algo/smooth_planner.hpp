#pragma once

#include "algo/ramp.hpp"
#include "algo/lpf.hpp"

// 来源：lunokhod control/wheel/inc/smooth_planner.hpp（复用搬运，行为不变）
// 二阶轨迹规划：Ramp（梯形限速）+ 两级 LPF（S 曲线圆角）
// 规格：FOC_MATH_SPEC.md §3.3
// set_state：状态注入（bumpless transfer）接口，行为不变，为对齐/模式切换同步新增（待同步回上游）

namespace foc::algo {

class SmoothPlanner {
public:
    SmoothPlanner(float max_rate, float Tf);
    float calc(float cmd, float dt);
    void reset();
    // 状态注入（bumpless transfer）：ramp/f1/f2 三层状态统一置 x，后续 calc 从 x 连续起步
    // 用途：对齐完成/模式切换时同步规划器到当前物理量
    // 等价 legacy dsp_traj 的 ramp_target/filter1/filter2 = settled 三行赋值（foc.c 对齐尾部）
    void set_state(float x);
private:
    Ramp ramp_;
    LPF f1_, f2_;
};

}  // namespace foc::algo
