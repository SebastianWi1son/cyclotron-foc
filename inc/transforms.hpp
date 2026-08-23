#pragma once

#include <cmath>

constexpr float SQRT3  = 1.73205080757f;
constexpr float INV_SQRT3  = 0.577350269189626;

namespace foc::transforms {

    struct AlphaBeta  { float a_, b_;     };
    struct DQ         { float d_, q_;     };
    struct ThreePhase { float u_, v_, w_; };


    inline AlphaBeta clarke(float ia, float ib) {
        return { ia, (ia + 2.0f * ib) * INV_SQRT3};
    }

    inline DQ park(AlphaBeta ab, float angle_elec) {
        float cos_a = std::cos(angle_elec);
        float sin_a = std::sin(angle_elec);
        return {  ab.a_ * cos_a + ab.b_ * sin_a,
                   -ab.a_ * sin_a + ab.b_ * cos_a };
    }

    inline AlphaBeta inv_park(DQ dq, float angle_elec) {
        float cos_a = std::cos(angle_elec);
        float sin_a = std::sin(angle_elec);
        return { -dq.q_ * sin_a + dq.d_ * cos_a,
                    dq.q_ * cos_a + dq.d_ * sin_a };
    }

    inline ThreePhase inv_clarke(AlphaBeta ab) {
        return { ab.a_,
                  (SQRT3 * ab.b_ - ab.a_) * 0.5f,
                 (-ab.a_ - SQRT3 * ab.b_) * 0.5f };
    }
}