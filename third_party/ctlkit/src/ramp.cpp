#include "ctl/ramp.hpp"

namespace ctl {

Ramp::Ramp(float max_rate): max_rate_(max_rate), prev_(0.0f) {}

float Ramp::calc(float cmd, float dt) {
    float step = max_rate_ * dt;
    float out = cmd;
    if (cmd > prev_ + step) { out = prev_ + step; }
    else if (cmd < prev_ - step) { out = prev_ - step; }
    prev_ = out;    // 关键：每帧都写回状态（return 提前退出会漏）
    return out;     // caution: 端点语义
}

void Ramp::reset() { prev_ = 0.0f; }

void Ramp::set_state(float x) { prev_ = x; }

}  // namespace ctl
