#pragma once

#include <cmath>

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::Deadzone（其自身复用搬运自 lunokhod）
// 行为契约：docs/spec/deadzone.md
// 抛物线软死区：|e| < range 时按 e·(|e|/range) 衰减（0 处增益 0，|e|=range 处连续）；
// 区外原样；range <= 0 时软分支恒不成立 → 天然直通，无除零。
// 注意：这是库内唯一直接使用 <cmath> 的组件（其余组件自带 float 原语）。

namespace ctl {

class Deadzone {
public:
    Deadzone(float range = 0.0f, bool soft = true);
    float calc(float error) const;
private:
    float range_;
    bool soft_;
};



}  // namespace ctl
