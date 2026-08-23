#pragma once

#include "transforms.hpp"

namespace foc::svpwm {
    struct Config {
        float voltage_limit_;           // clamp for security
        float voltage_supply_;          // center voltage
    };

    transforms::ThreePhase modulate(const Config& cfg, transforms::ThreePhase uvw);
    transforms::ThreePhase write(const Config& cfg, transforms::DQ dq, float angle_elec);

}