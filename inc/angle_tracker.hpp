#pragma once
#include "algo/lpf.hpp"

namespace foc::angle_tracker {
    struct Config { float vel_lpf_Tf_; };

    class Tracker {
    public:
        explicit Tracker(const Config& cfg);
        void reset(float angle_raw);                // ?
        void update(float angle_raw, float dt);
        float angle_abs() const;                    // abs_angle
        float velocity() const;                     // rad/s
        int full_rotations() const;
    private:
        Config cfg_;
        float raw_prev_;
        float abs_prev_;
        int full_rotations_;
        float vel_;
        algo::LPF vel_lpf_;
    };
}


