#include "alignment.hpp"
#include "svpwm.hpp"
#include <cmath>

namespace foc::alignment {


namespace { constexpr float k3PI_2 = 4.71238898038f; }

Aligner::Aligner(const Config &cfg, int pole_pairs, int direction)
    : cfg_(cfg), pp_(pole_pairs), dir_(direction),              // hw
      state_(State::IDLE), fault_(Fault::NONE),                 // state & fault
      elapsed_(0.0f), settle_count_(0), settle_raw_prev_(0.0f),    // settle
      zero_offset_elec_(0.0f) {}                                // zero_offset

void Aligner::start() {
    elapsed_ = 0.0f;
    settle_count_ = 0;
    settle_raw_prev_ = 0.0f;
    state_ = State::RAMP;
    fault_ = Fault::NONE;
}

void Aligner::abort() {
    elapsed_ = 0.0f;
    settle_count_ = 0;
    state_ = State::IDLE;
    fault_ = Fault::NONE;
}

//   IDLE: 无动作，输出 0
//   RAMP: 电角度固定 1.5π；v = align_voltage·(t/ramp_time) 每 tick 递增（svpwm::write 计算 uvw）
//         到 t≥ramp_time → SETTLE（t 由 dt 累积）
//   SETTLE: 采样 raw_angle，连续 settle_samples 次 |Δ|<settle_threshold → LOCKED；
//           超时 → FAULT(SETTLE_TIMEOUT)；角度发散(Δ>2π·0.8?) → FAULT(UNSTABLE)
//   LOCKED: 计算 zero_offset_elec = raw·pole_pairs·direction，输出 0（撤电压由 foc_core 统一出口）
//   FAULT:  原因码可查，输出 0（断电由调用方决定）
TickResult Aligner::calc(float angle_raw, float dt) {
    TickResult r{ state_, 0.0f, 0.0f, 0.0f };
    switch (state_) {
        case State::IDLE  : { break; }
        case State::RAMP  : { r = do_ramp(angle_raw, dt); break; }
        case State::SETTLE: { do_settle(angle_raw, dt);   break; }
        case State::LOCKED: { break; }
        case State::FAULT : { break; }
    }
    return r;
}

// RAMP:
TickResult Aligner::do_ramp(float angle_raw, float dt) {
    elapsed_ += dt;                                                   // ramp time elapsed
    float k = std::fmin(elapsed_ / cfg_.align_ramp_time_, 1.0f);  // (progress, 100%) ramp
    auto uvw = svpwm::calc({cfg_.align_voltage_, cfg_.voltage_supply_},
                                       {0.0f, cfg_.align_voltage_ * k}, k3PI_2);
    TickResult r{ state_, uvw.u_, uvw.v_, uvw.w_ };

    // --- transition to SETTLE ---
    // --- when ramp finish ---
    if (elapsed_ >= cfg_.align_ramp_time_) {
        state_ = State::SETTLE;
        elapsed_ = 0.0f;
        settle_count_ = 0;
        settle_raw_prev_ = angle_raw;
    }
    return r;
}

void Aligner::do_settle(float angle_raw, float dt) {
    elapsed_ += dt;                                     // ramp time elapsed
    // --- fault: alignment timeout ---
    if (elapsed_ >= cfg_.settle_timeout_) {
        state_ = State::FAULT;
        fault_ = Fault::SETTLE_TIMEOUT;
        return;
    }
    // --- delta displacement ---
    float d = angle_raw - settle_raw_prev_;
    settle_raw_prev_ = angle_raw;
    // --- settled by threshold ---
    if (std::fabs(d) < cfg_.settle_max_speed_ * dt) {
        // --- fetch zero offset consecutive samples ---
        if (++settle_count_ >= cfg_.settle_samples_) {
            zero_offset_elec_ = angle_raw * pp_ * dir_;
            // --- transition to LOCKED ---
            state_ = State::LOCKED;
        }
    } else { settle_count_ = 0; }

}

// ----- getter -----
bool Aligner::is_locked() const { return state_ == State::LOCKED; }
float Aligner::zero_offset_elec() const { return zero_offset_elec_; }
Fault Aligner::fault() const { return fault_; }




}
