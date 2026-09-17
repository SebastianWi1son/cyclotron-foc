---
class: log
generated: false
---
# CyclotronFOC — Handoff 文档

> 生成日期：2026-08-27 ｜ 用途：交接给新环境/新 agent 接手继续开发
> 主文档：`docs/FOC_CORE_PSEUDOCODE.md`（设计权威，必读）

## 0. 这是什么

**CyclotronFOC**：单电机可移植固件项目（cyclotron）的 FOC 核心库——把 legacy（STM32H743 云台双轴 C 裸机）的 FOC 数学链路用 C++17 重写为零依赖、PC 可测的独立库。

- GitHub：`https://github.com/SebastianWi1son/CyclotronFOC`
- SSH：`git@github.com:SebastianWi1son/CyclotronFOC.git`（本机 SSH key 已配置，**HTTPS 无凭据勿用**）
- LICENSE：MIT ｜ CI：GitHub Actions 已配（build/test + ASan/UBSan 双 job）

## 1. 接手第一步（新机器）

```bash
git clone git@github.com:SebastianWi1son/CyclotronFOC.git foc
cd foc
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure   # 5 个测试 target 应全绿
```

## 2. 结构与当前状态

**目录**：`inc/foc/`（12 头文件，前缀目录防撞名）/ `src/foc/`（11 源文件）/ `test/`（5 测试）/ `examples/` / `docs/`

**组件**：`algo`（pid/lpf/ramp/smooth_planner/deadzone）→ `transforms`（Clarke/Park）→ `svpwm`（D13 零序注入）→ `angle_tracking`（多圈展开+速度）→ `alignment`（非阻塞状态机）→ `current_loop`（v2 dq 电流环）→ `FOC`（编排门面，唯一对外 API）

**架构**（双入口）：`pos_tick`（1kHz：对齐/同步/位置-速度级联 → 命令缓冲）+ `modulation_tick`（PWM 频：统一发波；CURRENT 内嵌电流闭环）；OPEN_LOOP 独立发波。模式 `OPEN_LOOP/VOLTAGE/CURRENT` 构造期固定。

**状态**：v1（VOLTAGE/OPEN_LOOP）+ v2（CURRENT 电流环）**全部完成，5 测试全绿**（ASan/UBSan 零报错）。git 历史 main 单线，已推 GitHub，CI 绿。

## 3. 关键文档索引

| 文档 | 角色 |
|---|---|
| `docs/FOC_CORE_PSEUDOCODE.md` | **主设计文档**：架构 / 接口 / 各组件数学规格 + 实现 |
| `docs/log/DECISIONS.md` | **决策记录**（D1~D17 / D-A1~A6）—— ⚠️ 2026-09-14 补：交接时决策记录还在 PSEUDOCODE §0/§12，现已抽出独立成文 |
| `docs/ROADMAP.md` | v3+ 升级路线（含外部评审补项与优先级） |
| `docs/ALIGNMENT.md` | 对齐状态机专题 |
| `docs/CURRENT_LOOP.md` | 电流环四家调研对比（legacy/odrive/qdrive/simplefoc） |
| `docs/FOC_MATH_SPEC.md` | **已废弃**，待处理（勿引用） |
| `README.md` | 快速上手 |

知识笔记（Vault）：`/home/wilson/Dev/Notes/Vault/cyclotron/`（8 篇：模块组织/Config 值语义/回调设计/命名/状态机经验等）

## 4. 协作铁律（与用户约定，接手后必须遵守）

1. **绝对只读**：未经用户**逐文件明确授权**不得写入/覆盖/删除/新建任何文件；授权例外：`test/`、`examples/`、`docs/` 新文档
2. **可撤销性**：任何覆盖/删除/移动前必须可回退（git 已提交 / 已备份 /tmp），三者皆无 = 禁止执行
3. **分工**：AI 给设计/参考代码/测试，**用户敲实现代码**；发现 bug 报告用户改（如 id_pid 目标、CMakeLists 漏源）
4. **命名**：用户起的名字与文档不一致时按命名评估框架判断——合理则接受并同步文档，不合理则提醒
5. **决策点前置**：需拍板的事项先列清单（含推荐与理由），等用户拍板再执行
6. **改完必回归**：任何改动（含注释级）必须重编译 + 全量测试，验证通过才算完成
7. **全程中文**；先讲原理再动手；汇报简洁、路径写清

## 5. 构建与测试

```bash
# 标准（CI 同款）
cmake -S . -B build && cmake --build build -j && ctest --test-dir build

# 快速手动回归（单测试 target）
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -I inc/foc \
  test/test_<name>.cpp src/foc/foc.cpp src/foc/current_loop.cpp src/foc/alignment.cpp \
  src/foc/angle_tracking.cpp src/foc/svpwm.cpp src/foc/algo/*.cpp -o /tmp/t && /tmp/t

# 完整生命周期演示
cmake --build build --target example_foc && ./build/example_foc
```

依赖：仅 `cmake` + `g++`（C++17，零第三方库）。

## 6. 进度与下一步

**已完成**：v1+v2 全绿 → 双入口架构 → 结构迁移（`inc/foc/` 前缀）→ LICENSE/README/CI → 死代码清理（fault 码 5→2、`// ?` 残留）→ GitHub 上线。

**下一步（ROADMAP 优先级）**：
1. **C2：G431 真机点亮**（固件骨架阶段）——优先于一切，硬件验证是可信度根基
2. **A6：判稳 2π 边界鲁棒**（实现 UNSTABLE 发散检测，与 D4 同步；当前 SETTLE 用 raw 域差分，raw≈0 有跳变误判风险）
3. **A8：modulate 最终 [0, Vdc] 钳位**（低成本防御，不依赖 voltage_limit_ 配置）
4. **A5 矢量缩放饱和 → A1 相位外推 → A2 BEMF/交叉耦合**（性能线）
5. **B 组架构**：B2 运行时切换 / B1 OPEN_LOOP 出口统一 / B4 中断原子性（随 G431）/ B6 Config 可配性

## 7. 用户习惯与工作流

- **CLion 开发**：用户勾选提交（AI 提供 commit message）；曾踩坑漏勾文件 → commit 前核对暂存列表
- **git 日常**：dev 分支开发 → 完成回 main 快进合并（历史保持单线）；`git mv` 移动文件
- 文档集中：FOC_CORE_PSEUDOCODE.md 唯一主文档，决策编号记录（D1~D17/D-A1~A6）
- 用户会限定改动范围（"只允许动 X 不可动其他"）——严格按范围执行，不顺手改其他
- 外部评审/意见：先验证再采纳（本项目外部评审 9 条经验证全部属实，已闭环：死代码清理 ✓ CI ✓ 其余入 ROADMAP）

## 8. 已知坑

- checkout/merge/reset 前工作区必须干净（曾覆盖未提交文档）
- HTTPS push 无凭据 → 用 SSH（已配置）
- 对齐判稳在 raw≈0 附近可能被 2π 跳变干扰（A6 修复）
- `docs/FOC_MATH_SPEC.md` 已废弃，勿引用
- Vault 笔记路径 `/home/wilson/Dev/Notes/Vault/cyclotron/` 在旧机器，新机器需同步
