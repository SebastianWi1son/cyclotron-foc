#include "svpwm.hpp"
#include <cmath>

namespace foc::svpwm {
    namespace {     // internal linkage
        constexpr float k2PI = 6.28318530717958647692f;

        // --- clamp tool ---
        float constrainf(float val, float limit) {
            if (val > limit) return limit;
            if (val < -limit) return -limit;
            return val;
        }

        // ----- -----
        transforms::ThreePhase compose(transforms::DQ dq, float angle_elec, float voltage_limit) {
            // --- clamp ---
            dq.d_ = constrainf(dq.d_, voltage_limit);
            dq.q_ = constrainf(dq.q_, voltage_limit);
            // --- nomalization ---
            float theta = std::fmod(angle_elec, k2PI);
            if (theta < 0.0f) { theta += k2PI; }
            // --- inv clarke/park ---
            return transforms::inv_clarke(transforms::inv_park(dq, theta));
        }
    }

    // ----- modulate by center voltage------
    transforms::ThreePhase modulate(const Config& cfg, transforms::ThreePhase uvw) {
        float center = cfg.voltage_supply_ * 0.5f;
        float vmax = std::fmax(uvw.u_, std::fmax(uvw.v_, uvw.w_));
        float vmin = std::fmin(uvw.u_, std::fmin(uvw.v_, uvw.w_));
        float zero_seq = -(vmax + vmin) * 0.5f;     // zero seq injection
        return { uvw.u_ + center + zero_seq,
                   uvw.v_ + center + zero_seq,
                  uvw.w_ + center + zero_seq };
    }

    // ----- final write entrance -----
    transforms::ThreePhase write(const Config& cfg, transforms::DQ dq, float angle_elec) {
        return modulate(cfg, compose(dq, angle_elec, cfg.voltage_limit_));
    }
}