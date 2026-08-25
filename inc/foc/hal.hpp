#pragma once

// hardware callback interface

namespace foc::hal {


// --- CallBack ---
using GetAngleFn   = float (*)(void* ctx);                                  // get raw angle rad (0, 2pi)
using GetCurrentFn = float (*)(void* ctx, float* ia, float* ib);            // get current
using SetPwmFn     = void  (*)(void* ctx, float ua, float ub, float uc);   // set phase voltage
using EnableFn     = void  (*)(void* ctx, bool enable);                     // pwm enable

struct Hardware {
    void* ctx_;          // pass through Driver Layer
    GetAngleFn get_angle_;
    GetCurrentFn get_current_;
    SetPwmFn set_pwm_;
    EnableFn enable_;
};




}