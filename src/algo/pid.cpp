#include "algo/pid.hpp"

namespace foc::algo {

PID::PID(const PIDConfig &cfg): cfg_(cfg),      // PID Unified Register Entrance
        integral_(0.0f), error_prev_(0.0f), measure_prev_(0.0f),
        d_filter_(cfg.d_filter_Tf_), ramp_out_(cfg.max_rate_out_) {}

float PID::calc(float cmd, float measure, float dt) {
    if (dt <= 0.0f || dt > 0.5f) { dt = 0.001f; }
    float error = cmd - measure;
    // ----- P-Term -----
    float p_term = cfg_.kp_ * error;
    // ----- I-Term ------
    float i_term_temp = integral_ + cfg_.ki_ * dt * 0.5f * (error + error_prev_);
    i_term_temp = constrainf(i_term_temp, cfg_.limit_i_);                                 // Integral Windup Limit
    if (cfg_.thresh_i_sep_ <= 0.0f || fabs(error) <= cfg_.thresh_i_sep_) { integral_ = i_term_temp; } // Integral Separation
    // ----- D-Term -----
    float inv_dt = 1.0f / dt;
    float d_term_raw = -cfg_.kd_ * inv_dt * (measure - measure_prev_);
    float d_term = d_filter_.calc(d_term_raw, dt);                                      // d term lpf
    // ----- Update State-----
    error_prev_ = error;
    measure_prev_ = measure;
    // ----- Integrate Output-----
    float output = constrainf((p_term + d_term + integral_), cfg_.limit_out_); // limit output
    if (cfg_.max_rate_out_ > 0.0f) { output = ramp_out_.calc(output, dt); }   // ramp output (disable at value 0.0f)
    return output;
}

void PID::reset() {
    integral_ = 0.0f;
    error_prev_ = 0.0f;
    measure_prev_ = 0.0f;
    d_filter_.reset();
    ramp_out_.reset();
}

// ----- Math Tools -----
float PID::fabs(float val) { return (val > 0.0f) ? val : -val; }       // float abs

float PID::constrainf(float val, float limit) {                        // float constrain
    if (val > limit) { return limit; }
    if (val < -limit) { return -limit; }
    return val;
}

}  // namespace foc::algo
