
cmake_minimum_required(VERSION 3.5)

project(ros2_ws)

# 查找并包含每个包的 CMakeLists.txt
add_subdirectory(src/safety_shield)
add_subdirectory(src/local_mapper)