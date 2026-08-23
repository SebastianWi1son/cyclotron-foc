#pragma once

struct Config { float vel_lpf_Tf_; };

class Tracker {
public:
    void init(const Config& cfg);
    void reset(float raw_angle);                // ?
    void update(float raw_angle, float dt);
    float abs_angle() const;                    // abs_angle
    float velocity() const;                     // rad/s
    int full_rotation() const;
};