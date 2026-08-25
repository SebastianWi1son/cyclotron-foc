#include "current_loop.hpp"

namespace foc::current_loop {

CurrentLoop::CurrentLoop(const algo::PIDConfig& iq_cfg, const algo::PIDConfig& id_cfg)
    : iq_pid_(iq_cfg), id_pid_(id_cfg) {}

transforms::DQ CurrentLoop::update(float iu, float iv, float angle_elec, float iq_ref, float dt) {
    auto uv = transforms::clarke(iu, iv);
    auto idq = transforms::park(uv, angle_elec);
    float uq = iq_pid_.calc(iq_ref, idq.q_, dt);
    float ud = id_pid_.calc(0.0f,   idq.d_, dt);
    return { ud, uq };
}

void CurrentLoop::reset() {
    iq_pid_.reset();
    id_pid_.reset();
}




}