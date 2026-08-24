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

add_library(foc STATIC
    src/algo/lpf.cpp
    src/algo/ramp.cpp
    src/algo/pid.cpp
    src/algo/smooth_planner.cpp
    src/angle_tracking.cpp
    src/alignment.cpp
    src/foc_core.cpp
    src/svpwm.cpp
)

target_include_directories(foc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_compile_options(foc PRIVATE -Wall -Wextra -Werror)

# ── 锚点测试（D8 六项，ASan/UBSan 加持）──
enable_testing()
add_executable(test_foc test/test_foc.cpp)
target_link_libraries(test_foc PRIVATE foc)
target_compile_options(test_foc PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(test_foc PRIVATE -fsanitize=address,undefined)
add_test(NAME test_foc COMMAND test_foc)

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
