#pragma once

#include <cstdint>

namespace foc::alignment {


struct Config {
    float align_voltage_;
    float voltage_supply_;
    float align_ramp_time_;
    float settle_max_speed_;
    int settle_samples_;
    float settle_timeout_;
};

enum class State : uint8_t { IDLE, RAMP, SETTLE, LOCKED, FAULT };
enum class Fault : uint8_t { NONE, NO_SENSOR, RAMP_TIMEOUT, UNSTABLE, SETTLE_TIMEOUT };
struct TickResult { State state_; float u_, v_, w_; };

class Aligner {
public:
    explicit Aligner (const Config &cfg, int pole_pairs, int direction);      // Construct/Init
    void start();                                                       // IDLE     -> RAMP
    void abort();                                                       // AnyState -> IDLE
    TickResult calc(float angle_raw, float dt);                         // tick update to result
    bool is_locked() const;                                                // is_locked
    float zero_offset_elec() const;                                     // get zero_offset
    Fault fault() const;                                                // return fault
private:
    TickResult do_ramp(float angle_raw, float dt);               // RAMP:   斜坡发波计算 + RAMP→SETTLE 转移
    void do_settle(float angle_raw, float dt);                   // SETTLE: 判稳 + →LOCKED/FAULT 转移

    Config cfg_;                // Aligner Config
    int    pp_;                 // pole pairs
    int    dir_;                // direction 1/-1

    State  state_;
    Fault fault_;

    float  elapsed_;                  // RAMP/SETTLE Timer（dt accelerate）
    int    settle_count_;       // 连续判稳计数
    float  settle_raw_prev_;    // 上次采样（Δ 判据）
    float  zero_offset_elec_;   // LOCKED 时冻结：raw·pp·dir
};




}