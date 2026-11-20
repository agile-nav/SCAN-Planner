# ============================================================================
# 文件：src/simulator/Utils/go2w_description/CMakeLists.txt
#
# 本包没有 C++ 代码，只安装官方 URDF 与 DAE 网格。
# 模型来源：unitreerobotics/unitree_ros 的 robots/go2w_description
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(go2w_description)

find_package(ament_cmake REQUIRED)

foreach(directory dae urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
