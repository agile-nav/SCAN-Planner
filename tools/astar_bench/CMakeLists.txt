# ============================================================================
# 文件：tools/astar_bench/CMakeLists.txt
#
# 【这是什么】
#   A* 行为基准程序的独立构建脚本。它**不属于** colcon 工作区的任何包，
#   只依赖已编译安装的 path_searching / plan_env 两个包。
#
# 【为什么独立】
#   它不做任何断言，只打印统计（expanded 分布、失败率）。放进 gtest 会
#   变成一个"永远不会失败"的用例，混在测试清单里误导人。单独放这里，
#   语义就是"测量工具"。
#
# 【怎么用】
#   见同目录下 astar_bench.sh（推荐），或手动：
#     cmake -S tools/astar_bench -B build/astar_bench
#     cmake --build build/astar_bench
#     ./build/astar_bench/astar_bench <pcd> <mode> <repeats>
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(astar_bench CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)

add_executable(astar_bench main.cpp)
ament_target_dependencies(astar_bench rclcpp plan_env path_searching)
target_link_libraries(astar_bench Eigen3::Eigen)
