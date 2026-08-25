#pragma once

#include <cmath>

namespace foc::algo {


class Deadzone {
public:
    Deadzone(float range = 0.0f, bool soft = true);
    float calc(float error) const;
private:
    float range_;
    bool soft_;
};




}