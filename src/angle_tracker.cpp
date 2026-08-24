#include "angle_tracker.hpp"

namespace foc::angle_tracker {

    namespace { constexpr float k2PI = 6.28318530717958647692f; }

    Tracker::Tracker(const Config& cfg)
        : cfg_(cfg), raw_prev_(0.0f), abs_prev_(0.0f), full_rotations_(0),
          vel_(0.0f), vel_lpf_(cfg_.vel_lpf_Tf_) {}

    void Tracker::reset(float angle_raw) {
        raw_prev_ = angle_raw;
        abs_prev_ = angle_raw;
        full_rotations_ = 0;
        vel_ = 0.0f;
        vel_lpf_.reset();
    }

    void Tracker::update(float angle_raw, float dt) {
        float d_raw = angle_raw - raw_prev_;
        if (d_raw > 0.8f * k2PI) { full_rotations_--; }
        else if (d_raw < -0.8f * k2PI) { full_rotations_++; }
        raw_prev_ = angle_raw;

        float abs = full_rotations_ * k2PI + angle_raw;
        if (dt > 0.0f) {
            float raw_vel = (abs - abs_prev_) / dt;
            vel_ = (cfg_.vel_lpf_Tf_ > 0.0f) ? vel_lpf_.calc(raw_vel, dt) : raw_vel;
        }
        abs_prev_ = abs;
    }

    float Tracker::angle_abs() const { return full_rotations_ * k2PI + raw_prev_; }

    float Tracker::velocity() const { return vel_; }

    int Tracker::full_rotations() const { return full_rotations_; }

}

