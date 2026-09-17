cmake_minimum_required(VERSION 3.16)
project(foc LANGUAGES CXX)

# cyclotron foc 库 — PC 可测优先（D10）
# 零依赖（仅 <cmath>）；声明/定义分离；-Wall -Wextra -Werror
# 三 target：foc（库）/ test_foc（锚点测试）/ example_foc（PC 仿真）

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# 算法原语（pid/lpf/ramp/smooth_planner/deadzone）自 v0.1.1 起来自上游库 ctlkit（vendor，见 third_party/ctlkit/VERSION）
add_library(foc STATIC
    third_party/ctlkit/src/lpf.cpp
    third_party/ctlkit/src/ramp.cpp
    third_party/ctlkit/src/pid.cpp
    third_party/ctlkit/src/smooth_planner.cpp
    third_party/ctlkit/src/deadzone.cpp

    src/foc/foc.cpp
    src/foc/angle_tracking.cpp
    src/foc/alignment.cpp
    src/foc/svpwm.cpp
    src/foc/current_loop.cpp
)

target_include_directories(foc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/foc                  # 本仓头（含 algo/ 转发头）
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ctlkit/inc   # 上游库头（ctl/...）
)
target_compile_options(foc PRIVATE -Wall -Wextra -Werror)

# ── 锚点测试（D8 六项，ASan/UBSan 加持）──
enable_testing()
add_executable(test_foc test/test_foc.cpp)
target_link_libraries(test_foc PRIVATE foc)
target_compile_options(test_foc PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_foc PRIVATE -fsanitize=address,undefined)
add_test(NAME test_foc COMMAND test_foc)

# ── current_loop 组件测试（v2 CURRENT；D-A1 独立组件独立验证）──
add_executable(test_current_loop test/test_current_loop.cpp)
target_link_libraries(test_current_loop PRIVATE foc)
target_compile_options(test_current_loop PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_current_loop PRIVATE -fsanitize=address,undefined)
add_test(NAME test_current_loop COMMAND test_current_loop)

# ── svpwm 语义测试（锚点 2，含 SVPWM 目标语义）──
add_executable(test_svpwm test/test_svpwm.cpp)
target_link_libraries(test_svpwm PRIVATE foc)
target_compile_options(test_svpwm PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_svpwm PRIVATE -fsanitize=address,undefined)
add_test(NAME test_svpwm COMMAND test_svpwm)

# ── angle_tracking 测试（锚点 3/5，含 enc_fake 极性验证）──
add_executable(test_tracker test/test_tracker.cpp)
target_link_libraries(test_tracker PRIVATE foc)
target_compile_options(test_tracker PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_tracker PRIVATE -fsanitize=address,undefined)
add_test(NAME test_tracker COMMAND test_tracker)

# ── alignment 状态机测试（D8-4 锚点 + 健壮性全路径）──
add_executable(test_alignment test/test_alignment.cpp)
target_link_libraries(test_alignment PRIVATE foc)
target_compile_options(test_alignment PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_alignment PRIVATE -fsanitize=address,undefined)
add_test(NAME test_alignment COMMAND test_alignment)

# ── PC 闭环仿真演示 ──
add_executable(example_foc examples/example_foc.cpp)
target_link_libraries(example_foc PRIVATE foc)
target_compile_options(example_foc PRIVATE -Wall -Wextra -Werror)
