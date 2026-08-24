#pragma once
#include "algo/lpf.hpp"

namespace foc::angle_tracking {


struct Config { float vel_lpf_Tf_; };           // Tracker Config

class Tracker {
public:
    explicit Tracker(const Config& cfg);
    void reset(float angle_raw);                // reset state
    void update(float angle_raw, float dt);     // update state
    float angle_abs() const;                    // get angle_abs
    float velocity() const;                     // get vel (rad/s)
    int full_rotations() const;                 // get full rotations (round)
private:
    Config cfg_;
    float raw_prev_;
    float abs_prev_;
    int full_rotations_;
    float vel_;
    algo::LPF vel_lpf_;
};




}


