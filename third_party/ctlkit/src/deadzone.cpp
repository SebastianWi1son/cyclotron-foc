#include "ctl/deadzone.hpp"

namespace ctl {

Deadzone::Deadzone(float range, bool soft) : range_(range), soft_(soft) {}

float Deadzone::calc(float error) const {
    float abs_error = std::fabs(error);
    if (abs_error < range_) return soft_ ? (error * (abs_error / range_)) : 0.0f;
    return error;
}

}  // namespace ctl
