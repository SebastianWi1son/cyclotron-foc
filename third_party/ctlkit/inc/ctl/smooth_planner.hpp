#pragma once

#include "ctl/ramp.hpp"
#include "ctl/lpf.hpp"

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::SmoothPlanner（复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/smooth_planner.md
// 二阶轨迹规划：Ramp（梯形限速）+ 两级 LPF（S 曲线圆角）
// set_state：状态注入（bumpless transfer），对齐/模式切换时同步规划器到当前物理量

namespace ctl {

class SmoothPlanner {
public:
    SmoothPlanner(float max_rate, float Tf);
    float calc(float cmd, float dt);
    void reset();
    // 状态注入（bumpless transfer）：ramp/f1/f2 三层状态统一置 x，后续 calc 从 x 连续起步
    void set_state(float x);
private:
    Ramp ramp_;
    LPF f1_, f2_;
};

}  // namespace ctl
