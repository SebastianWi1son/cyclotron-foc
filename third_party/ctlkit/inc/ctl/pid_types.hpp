#pragma once

// ctlkit —— 嵌入式实时控制原语（上游库）
// PID 支撑类型（配置 / 每拍端口 / 观测记录）：与类定义分离，下游读类型不必翻类实现
// 行为契约：docs/spec/pid.md ｜ 设计：docs/design/pid_config_and_ports.md
//
// 命名与冻结：对外数据记录（配置 / 端口 / 观测）字段沿用尾下划线；
// 自 v0.1.0 起字段名与语义**冻结** —— 只增不改，破坏进 major（见 README「兼容政策」）。

namespace ctl {

// --- 配置（构造期一次注入；构造后只有 gains_ 可在线上改（PID::set_gains），其余字段不可变）---
struct PIDGains {
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
};

struct PIDLimits {
    float limit_out_ = 0.0f;   // 输出对称限幅；<= 0 = 不限幅（0 语义统一，roadmap D-1）
    float limit_i_ = 0.0f;     // 积分项预限幅；<= 0 = 不限幅
};

struct PIDTunings {
    float thresh_i_sep_ = 0.0f;
    float max_rate_out_ = 0.0f; // 输出斜坡速率；0 = 关闭（构造函数内归一化为“无上限速率”传给 Ramp：PID 层 0=关闭，Ramp 层 0=冻结，语义不串层）
    float d_filter_Tf_ = 0.0f;
};

struct PIDConfig {
    PIDGains gains_;
    PIDLimits limits_;
    PIDTunings tunings_;

    // --- 具名链式设置器（v0.1.1 只增不改）---
    // 为什么：`PIDConfig{1.0f, 50.0f, 0, 3.0f, 3.0f, 0, 0, 0}` 只能靠“数位置”读，且往组中间插字段就静默错位；
    //         C++20 的指定初始化（`.kp_ = 1.0f`）在本库的 C++11 底线不可用 → 用具名设置器代替。
    // 规则：设置器名 = 字段名去掉尾下划线；返回 *this 支持链式；未设置的字段保持默认（全 0）。
    // 用法：const PIDConfig cfg = PIDConfig{}.kp(1.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f);
    // 注意：本结构保持**聚合类型**（位置初始化与下游 `Config` 聚合仍可用）→ 只加成员函数、不加构造函数。
    PIDConfig &kp(float v)           { gains_.kp_ = v;             return *this; }
    PIDConfig &ki(float v)           { gains_.ki_ = v;             return *this; }
    PIDConfig &kd(float v)           { gains_.kd_ = v;             return *this; }
    PIDConfig &limit_out(float v)    { limits_.limit_out_ = v;     return *this; }
    PIDConfig &limit_i(float v)      { limits_.limit_i_ = v;       return *this; }
    PIDConfig &thresh_i_sep(float v) { tunings_.thresh_i_sep_ = v; return *this; }
    PIDConfig &max_rate_out(float v) { tunings_.max_rate_out_ = v; return *this; }
    PIDConfig &d_filter_Tf(float v)  { tunings_.d_filter_Tf_ = v;  return *this; }
};

// --- 每拍端口（模块 5 起；calc 签名冻结，新特性只往这里加字段）---
// 全部可缺省：ports == nullptr 或字段 nullptr = 旧行为
struct PIDPorts {
    const float *meas_dot_ = nullptr;   // external derivative source injection（外部微分：观测器提供，量纲/符号 = d(measure)/dt）
};

// --- 观测记录：数值快照 ---
struct PIDState {
    float error_ = 0.0f;
    float p_term_ = 0.0f;
    float d_term_ = 0.0f;
    float integral_ = 0.0f;
    float output_ = 0.0f;
};

// 本拍瞬态布尔量（A 方案）：粘滞的 input_fault 不在此，见 PID::input_fault()
struct PIDStatus {
    bool out_saturated_ = false;            // is_out_limited
    bool i_saturated_ = false;              // 被采纳的积分值真被限幅器削过（分离冻结不算 —— TODO T3）
    bool i_frozen_ = false;                 // 本拍因积分分离而冻结（|error| > thresh_i_sep_）：候选值被丢弃、积分未更新
    bool dt_rejected_ = false;              // 本拍 dt 非法（< 1e-9 或 > 0.5，含 <= 0）已被替换为 1ms（TODO T2）
};

}  // namespace ctl
