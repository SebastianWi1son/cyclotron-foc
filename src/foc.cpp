#include "foc.hpp"
#include "svpwm.hpp"
#include <algorithm>
#include <cmath>

namespace foc {


FOC::FOC(hal::Hardware hw, const Config &cfg)
    : hw_(hw), config_(cfg),
      tracker_(angle_tracking::Config{cfg.vel_lpf_tf_}),
      aligner_(alignment::Config{
          cfg.align_voltage_, cfg.voltage_supply_, cfg.align_ramp_time_,
          cfg.settle_max_speed_, cfg.settle_samples_, cfg.settle_timeout_},
          cfg.pole_pairs_, cfg.direction_),
      vel_pid_(cfg.vel_pid_), pos_pid_(cfg.angle_pid_),
      planner_(cfg.traj_vmax_, cfg.traj_tf_),
      current_loop_(cfg.iq_pid_, cfg.id_pid_),
      planned_prev_(0.0f), angle_elec_(0.0f), zero_offset_(0.0f), synced_(false),
      uq_ref_(0.0f), iq_ref_(0.0f), align_uvw_{0.0f, 0.0f, 0.0f} {}

void FOC::enable(bool on) {
    hw_.enable_(hw_.ctx_, on);
    if (!on) { aligner_.abort(); synced_ = false; }
}

void FOC::align_and_sync(float* cmd_out) {         // ?
    aligner_.start();
    synced_ = false;
    *cmd_out = 0.0f;
}

void FOC::pos_tick(float angle_cmd, float dt) {

    if (dt <= 0.0f) return;
    // --- one sample for whole tick ---
    float raw = hw_.get_angle_(hw_.ctx_);

    // ----- non-block alignment -----
    if (!aligner_.is_locked()) {                               // no locked no closed loop entrance
        auto ar = aligner_.calc(raw, dt);
        align_uvw_ = {ar.u_, ar.v_, ar.w_};  // power up to align
        return;
    }
    // ----- sync once -----
    if (!synced_) {
        zero_offset_ = aligner_.zero_offset_elec();
        tracker_.reset(raw);
        planner_.set_state(tracker_.angle_abs());
        planned_prev_ = tracker_.angle_abs();
        synced_ = true;
    }

    // ----- closed loop -----
    tracker_.update(raw, dt);                                           // update measure
    float planned = planner_.calc(angle_cmd, dt);                       // process cmd with planner

    float err_ang = config_.deadzone_.calc(planned - tracker_.angle_abs());// process err with deadzone
    float vel_cmd = pos_pid_.calc(err_ang, 0.0f, dt);    // angle pid -> vel_cmd
    vel_cmd += (planned - planned_prev_) / dt;          // forward feedback?
    planned_prev_ = planned;
    if (config_.traj_vmax_ > 0.0f) {
        vel_cmd = std::clamp(vel_cmd, -config_.traj_vmax_, config_.traj_vmax_);
    }
    float uq = vel_pid_.calc(vel_cmd, tracker_.velocity(), dt);

    // --- cascaded pid ---
    // pos_loop -> vel_loop
    angle_elec_ = tracker_.angle_abs() * config_.pole_pairs_ * config_.direction_ - zero_offset_;
    if (config_.ctrl_mode_ == foc::CtrlMode::CURRENT) {
        iq_ref_ = (config_.iq_limit_ > 0.0f) ? std::clamp(uq, -config_.iq_limit_, config_.iq_limit_) : uq;
    }
    else { uq_ref_ = uq; }

}

// ----- modulation tick with current closed loop ------
void FOC::modulation_tick(float dt) {
    if (dt <= 0.0f) return;
    
    if (!aligner_.is_locked()) {
        hw_.set_pwm_(hw_.ctx_, align_uvw_.u_, align_uvw_.v_, align_uvw_.w_);
        return;
    }
    if (config_.ctrl_mode_ == CtrlMode::CURRENT) {
        float iu = 0.0f, iv = 0.0f;
        hw_.get_current_(hw_.ctx_, &iu, &iv);
        auto udq = current_loop_.update(iu, iv, angle_elec_, iq_ref_, dt);
        auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_}, udq, angle_elec_ );
        hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
    }
    else {
        auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_},
                                         {0.0f, uq_ref_}, angle_elec_);
        hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
    }
}


// ----- open loop -----
void FOC::tick_velocity(float vel_cmd, float limit_voltage, float dt) {
    angle_elec_ += vel_cmd * config_.pole_pairs_ * config_.direction_ * dt;
    auto r = svpwm::calc({config_.voltage_limit_, config_.voltage_supply_},
                          {0.0f, limit_voltage}, angle_elec_);
    hw_.set_pwm_(hw_.ctx_, r.u_, r.v_, r.w_);
}

// ----- getter -----
bool FOC::is_aligned() const { return aligner_.is_locked(); }
float FOC::angle()     const { return tracker_.angle_abs(); }
float FOC::velocity()  const { return tracker_.velocity(); }
int FOC::fault()       const { return static_cast<int>(aligner_.fault()); }




}
