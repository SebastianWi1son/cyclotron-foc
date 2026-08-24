#pragma once

#include <cmath>

constexpr float SQRT3  = 1.73205080757f;
constexpr float INV_SQRT3  = 0.577350269189626;

namespace foc::transforms {


struct AlphaBeta  { float alpha_, beta_; };
struct DQ         { float d_, q_; };
struct ThreePhase { float u_, v_, w_; };

inline AlphaBeta clarke(float u, float v) {
    return { u, (u + 2.0f * v) * INV_SQRT3};
}

inline DQ park(AlphaBeta ab, float angle_elec) {
    float cos_a = std::cos(angle_elec);
    float sin_a = std::sin(angle_elec);
    return {  ab.alpha_ * cos_a + ab.beta_ * sin_a,
               -ab.alpha_ * sin_a + ab.beta_ * cos_a };
}

inline AlphaBeta inv_park(DQ dq, float angle_elec) {
    float cos_a = std::cos(angle_elec);
    float sin_a = std::sin(angle_elec);
    return { -dq.q_ * sin_a + dq.d_ * cos_a,
                 dq.q_ * cos_a + dq.d_ * sin_a };
}

inline ThreePhase inv_clarke(AlphaBeta ab) {
    return { ab.alpha_,
              (SQRT3 * ab.beta_ - ab.alpha_) * 0.5f,
             (-ab.alpha_ - SQRT3 * ab.beta_) * 0.5f };
}




}