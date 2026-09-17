#pragma once

// ctlkit-forwarder —— 机器可读标记：本文件是转发头，不是上游副本（校验脚本据此跳过逐字比对）
// 算法原语的上游是 ctlkit —— vendor 在 third_party/ctlkit/（VERSION 记来源 sha）。
// 保留本路径与 foc::algo 名字，只为不改下游调用点（含应用侧）；LPF 行为契约见上游 docs/spec/lpf.md（未随 vendor 拷贝）
// 血缘：lunokhod control/wheel → cyclotron foc::algo → ctlkit ctl

#include "ctl/lpf.hpp"

namespace foc::algo {
using ctl::LPF;
}  // namespace foc::algo
