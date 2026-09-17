# FOC 核心库设计 — 决策清单（cyclotron）

> 日期：2026-08-23 ｜ 状态：**待用户拍板，未进入实现**
> 定位：C++17 重写 FOC 核心（非重构 gimbal_2axis），纯算法层、零依赖、PC 可测
> 路线：核心库 → G431 单电机固件（可移植）→ 真实云台（长期）
> 原则：复用 wheel 全部经验（Config 聚合 / 0=disabled / 回调+dt 穿透 / 单位约定 / 锚点测试 / -Werror / 声明定义分离）
> 参考：legacy `Lib/foc/`（foc.h/c + foc_transform.h，已全部读完）

## 现状（legacy 可继承的好东西）

- 硬件回调抽象（get_angle/get_current/set_pwm/enable）——与 wheel 回调同构
- 三模式：VOLTAGE（无电流感）/ CURRENT / OPEN_LOOP + 电流感离线自动回退
- 级联：traj → angle PID → vel 前馈+环 → [iq/id PID] → SVPWM
- 变换：Clarke/Park/InvPark（等幅值约定）
- 对齐 + 多圈展开 + 速度 LPF

## C 时代的问题（重写动机）

1. `foc_motor_t` 上帝对象（一切状态一个 struct）
2. 对齐阻塞 2s（delay_ms_cb 循环）
3. 无状态机/错误上报（对齐失败仅 retry 3）
4. gimbal 层 extern 全局硬连（本阶段不做，但设计留解耦位）

---

## 决策点（请逐条拍板，拍完我才写实现）

### D1. 项目命名
- 选项：a) `cyclotron`（工作目录名）b) 另起中性名 c) `foc`（直白）
- 影响：仓库名、namespace、文档标题

### D2. v1 范围（关键裁剪）
- 选项：
  - a) **三模式全做**（VOLTAGE/CURRENT/OPEN_LOOP）——CURRENT 需要电流采样硬件（G431 才有），PC 测试只能测 VOLTAGE 路径
  - b) **先 VOLTAGE + OPEN_LOOP**（纯软件可测，对齐/电压闭环/开环全部 PC 锚点可覆盖），CURRENT 留 v2
  - 推荐：b（最小可验证；电流环是 v2/G431 阶段）
- 影响：测试范围、G431 固件阶段才碰电流

### D3. 对齐方式
- 选项：a) **非阻塞状态机**（IDLE→RAMP→SETTLE→LOCKED，tick 驱动，PC 可测）b) 保留阻塞（delay_ms_cb 简单）
- 推荐：a（消除 2s 阻塞，状态机可测；G431 上对齐期间还能响应急停）
- 影响：对齐是核心状态机的第一个实例

### D4. 多圈展开/速度估计归属
- 选项：a) 核心内 `AngleTracker` 组件（可测）b) 放驱动层（HAL 回调直接给展开角）
- 推荐：a（多圈回绕逻辑是纯数学，PC 锚点可测；驱动层只管读原始角）
- 影响：HAL 回调语义 = "返回物理角 [0,2π)"

### D5. 回调形态
- 选项：a) 函数指针 + motor_id 参数通道（wheel 同构）b) C++ 虚接口 c) std::function
- 推荐：a（单电机 G431 够用，零开销，与 wheel 一致；升级触发器同 wheel 定案）
- 影响：与 wheel 代码风格统一

### D6. PID 来源（联动算法库决策）
- 选项：a) 从算法库拉取（若算法库定案）b) cyclotron 自带副本（当前 wheel 版）
- 推荐：a（这就是用户算法库想法的第一个应用场景）
- 影响：依赖算法库的成熟度；算法库未定案前可先用副本过渡

### D7. 单位约定
- 角度 rad / 角速度 rad/s / 电流 A / 电压 V；命名中性（不绑 rpm 类单位）
- 选项：a) 沿用 legacy（rad 系）b) 其他
- 推荐：a，文档声明 + 锚点钉单位

### D8. 测试锚点范围（行为锚点，不测数值细节）
1. 变换恒等式：Clarke/Park/InvPark 往返（构造已知向量验证）
2. SVPWM：限幅后三相幅值 ≤ voltage_limit、中心对齐不变式（ua+ub+uc = 3·center）
3. 多圈回绕：角度跳变 ±2π 判据（0.8·2π 阈值）计数正确
4. 对齐状态机：RAMP→SETTLE→LOCKED 迁移 + 失败路径（角度不稳定→重试→超时错误）
5. 速度估计：恒定角速度 → LPF 收敛；零速静止 → 无漂移
6. 级联回归：planner → angle → vel 链路在 VOLTAGE 模式的输出有界
- 决策点：以上 6 项是否覆盖 v1 验收？缺/多的说

### D9. 过调制/弱磁/死区补偿
- 选项：a) v1 不做（保持最小）b) 预留接口
- 推荐：a（最小核心原则；弱磁是 v2+）
- 影响：SVPWM 只做中心对齐线性区

### D10. 工程约束
- 零依赖（仅 <cmath>）、声明/定义分离、-Wall -Wextra -Werror、CMake（PC 测试 target + 库 target）
- 决策点：确认沿用，无异议默认

---

## 明确不做（本阶段）

- gimbal 层（云台坐标/限位/最短路径）——v2 或云台项目阶段
- motor_param 参数注册表 / motor_tune 调试协议——G431 固件阶段
- 电流环（CURRENT 模式）——若 D2 选 b 则 v2
- 真实硬件联调——PC 仿真先行（wheel example 同款电机模型可复用）
