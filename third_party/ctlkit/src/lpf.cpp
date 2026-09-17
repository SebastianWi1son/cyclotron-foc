#include "ctl/lpf.hpp"

namespace ctl {

LPF::LPF(float Tf): Tf_(Tf), prev_(0.0f) {}

float LPF::calc(float raw, float dt) {
    float alpha = dt / (Tf_ + dt);
    prev_ = alpha * raw + (1.0f - alpha) * prev_;
    return prev_;
}

void LPF::reset() { prev_ = 0.0f; }

void LPF::set_state(float x) { prev_ = x; }

}  // namespace ctl
