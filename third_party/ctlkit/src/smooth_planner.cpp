#include "ctl/smooth_planner.hpp"

namespace ctl {

SmoothPlanner::SmoothPlanner(float max_rate, float Tf): ramp_(max_rate), f1_(Tf), f2_(Tf) {}

float SmoothPlanner::calc(float cmd, float dt) {
    float ramped = ramp_.calc(cmd, dt);
    return f2_.calc(f1_.calc(ramped, dt), dt);
}

void SmoothPlanner::reset() {ramp_.reset(); f1_.reset(); f2_.reset();}

void SmoothPlanner::set_state(float x) { ramp_.set_state(x); f1_.set_state(x); f2_.set_state(x); }

}  // namespace ctl
