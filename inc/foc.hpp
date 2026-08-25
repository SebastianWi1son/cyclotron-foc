#pragma once

#include "angle_tracking.hpp"
#include "alignment.hpp"
#include "current_loop.hpp"
#include "algo/smooth_planner.hpp"
#include "algo/deadzone.hpp"
#include "algo/pid.hpp"
#include "hal.hpp"
#include <cstdint>

namespace foc {


// ----- ctrl mode options -----
enum class CtrlMode : uint8_t { OPEN_LOOP, VOLTAGE, CURRENT };
// ----- foc internal config -----
struct Config {
    // --- electric ---
    float voltage_supply_;
    float voltage_limit_;
    int pole_pairs_;
    int direction_;
    float vel_lpf_tf_;
    // --- align ---
    float align_voltage_; float align_ramp_time_; float settle_max_speed_;
    int settle_samples_; float settle_timeout_;
    // --- trajectory ---
    float traj_vmax_;
    float traj_tf_;
    // --- pid ---
    algo::PIDConfig angle_pid_;
    algo::PIDConfig vel_pid_;
    algo::PIDConfig iq_pid_;
    algo::PIDConfig id_pid_;

    float iq_limit_;
    algo::Deadzone deadzone_;

    CtrlMode ctrl_mode_;
};


class FOC {
public:
    FOC(hal::Hardware hw, const Config &cfg);
    void enable(bool on);                           // hw.enable wrapper
    void align_and_sync(float* cmd_out);            // non-block startup
    void pos_tick(float angle_cmd, float dt);           // main entrance
    void modulation_tick(float dt);
    // --- open loop ---
    void tick_velocity(float vel_cmd, float limit_voltage, float dt);
    // --- getter ---
    bool is_aligned() const;
    float angle() const;
    float velocity() const;
    int fault() const;

private:
    hal::Hardware hw_;
    Config config_;
    angle_tracking::Tracker tracker_;
    alignment::Aligner aligner_;

    algo::PID vel_pid_, pos_pid_;
    algo::SmoothPlanner planner_;

    current_loop::CurrentLoop current_loop_;

    float planned_prev_;
    float angle_elec_;
    float zero_offset_;
    bool synced_;

    // --- tick across cmd cache ---
    // position --> current
    float uq_ref_;
    float iq_ref_;
    transforms::ThreePhase align_uvw_;
};




}


