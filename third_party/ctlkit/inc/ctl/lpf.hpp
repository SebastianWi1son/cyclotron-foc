#pragma once

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::LPF（复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/lpf.md
// 一阶低通滤波器：alpha = dt/(Tf+dt)；Tf=0 → 直通（0=disabled 语义）
// 注意：内部无 dt 守卫，由调用方保证 dt > 0

namespace ctl {

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

}  // namespace ctl
