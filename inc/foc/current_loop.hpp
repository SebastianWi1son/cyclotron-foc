#pragma once

#include "algo/pid.hpp"
#include "transforms.hpp"

namespace foc::current_loop {


class CurrentLoop {
public:
    CurrentLoop(const algo::PIDConfig &iq_cfg, const algo::PIDConfig &id_cfg);
    transforms::DQ update(float ia, float ib, float angle_elec, float iq_ref, float dt);
    void reset();
private:
    algo::PID iq_pid_, id_pid_;
};




}