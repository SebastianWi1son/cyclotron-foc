#include "ctl/pid.hpp"

namespace ctl {

PID::PID(const PIDConfig &cfg): cfg_(cfg),      // PID Unified Register Entrance
        integral_(0.0f), error_prev_(0.0f), measure_prev_(0.0f), last_output_(0.0f),
        d_filter_(cfg.tunings_.d_filter_Tf_),
        // 0 语义统一（roadmap D-1）：PID 层 0 = 关闭输出斜坡；Ramp 自身 0 = 冻结输出。
        // 这里把“关闭”归一化成无上限速率（与 is_finite 上界同一常量），斜坡恒直通且语义不串层。
        ramp_out_((cfg.tunings_.max_rate_out_ > 0.0f) ? cfg.tunings_.max_rate_out_ : 3.402823466e+38f) {}

float PID::calc(float cmd, float measure, float dt, const PIDPorts *ports) {
    // ----- NaN Guard -----
    if (!is_finite(cmd) || !is_finite(measure) || !is_finite(dt)) {
        input_fault_ = true;   // 粘滞：reset() 才清
        return last_output_;
    }
    // ----- ports unpack -----
    const float *meas_dot = (ports != nullptr) ? ports->meas_dot_ : nullptr;
    if (meas_dot != nullptr && !is_finite(*meas_dot)) {
        input_fault_ = true;
        return last_output_;
    }
    // ----- dt Guard（含下界：dt < 1e-9 会让 1/dt 溢出、静默污染 D 状态 —— TODO T2）-----
    status_.dt_rejected_ = (dt < 1e-9f || dt > 0.5f);
    if (status_.dt_rejected_) { dt = 0.001f; }

    float error = cmd - measure;
    // ----- P-Term -----
    float p_term = cfg_.gains_.kp_ * error;
    // ----- I-Term ------
    float i_term_temp = integral_ + cfg_.gains_.ki_ * dt * 0.5f * (error + error_prev_);
    float i_term_limited = constrainf(i_term_temp, cfg_.limits_.limit_i_);
    // 积分分离：大误差时冻结（候选值丢弃）
    const bool i_commit = (cfg_.tunings_.thresh_i_sep_ <= 0.0f || fabs(error) <= cfg_.tunings_.thresh_i_sep_);
    if (i_commit) { integral_ = i_term_limited; }
    // 两个 I 出口，各回答一个问题（诚实口径）：冻结 = 本拍丢弃候选（设计意图）；饱和 = 被采纳的值真被削过
    status_.i_frozen_ = !i_commit;                                        // 积分分离生效（本拍未积分）
    status_.i_saturated_ = i_commit && (i_term_limited != i_term_temp);   // i_saturated flag
    // ----- D-Term -----
    float inv_dt = 1.0f / dt;
    float d_term_raw = (meas_dot != nullptr)
            ? -cfg_.gains_.kd_ * (*meas_dot)
            : -cfg_.gains_.kd_ * inv_dt * (measure - measure_prev_);
    float d_term = d_filter_.calc(d_term_raw, dt);                                      // d term lpf
    // ----- Update State-----
    error_prev_ = error;
    measure_prev_ = measure;
    // ----- Integrate Output-----
    float output_unclamped = p_term + d_term + integral_;
    float output = constrainf(output_unclamped, cfg_.limits_.limit_out_); // limit output（<= 0 = 不限幅）
    status_.out_saturated_ = (output != output_unclamped);
    output = ramp_out_.calc(output, dt);   // 斜坡恒开启；关闭时速率已归一化为“无上限”（见构造函数）
    // --- observe cache ---
    state_.error_ = error;
    state_.p_term_ = p_term;
    state_.d_term_ = d_term;
    state_.integral_ = integral_;
    state_.output_ = output;
    // --- final output ---
    last_output_ = output;
    return output;
}

void PID::reset() {
    integral_ = 0.0f;
    error_prev_ = 0.0f;
    measure_prev_ = 0.0f;
    last_output_ = 0.0f;
    d_filter_.reset();
    ramp_out_.reset();
    state_ = PIDState();
    status_ = PIDStatus();
    input_fault_ = false;    // 粘滞 fault 只有这里清
}

void PID::set_integral(float x) { integral_ = constrainf(x, cfg_.limits_.limit_i_); }

void PID::set_gains(const PIDGains &g) { cfg_.gains_ = g; }

const PIDState &PID::get_state() const { return state_; }   // 零拷贝：引用内部缓存，内容 = 最近一次 calc

// ----- Math Tools -----
float PID::fabs(float val) { return (val > 0.0f) ? val : -val; }       // float abs

// float constrain —— 0 语义统一（roadmap D-1）：limit <= 0 表示“不限幅”（直通）
float PID::constrainf(float val, float limit) {
    if (limit <= 0.0f) { return val; }
    if (val > limit) { return limit; }
    if (val < -limit) { return -limit; }
    return val;
}

bool PID::is_finite(float x) {
    return (x == x) && (x <= 3.402823466e+38f) && (x >= -3.402823466e+38f);
}



}  // namespace ctl
