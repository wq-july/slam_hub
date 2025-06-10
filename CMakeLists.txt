cmake_minimum_required(VERSION 3.0.2)
project(slam_hub)

# 开启 compile_commands.json 生成，便于 clangd 支持
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
)

## 包定义
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slam_hub
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters
)

## 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译 slam_hub 库
add_library(slam_hub
  src/slam_hub.cpp
)

## 编译主节点
add_executable(localization_node
  src/localization_node.cpp
)

## 链接库
target_link_libraries(localization_node
  slam_hub
  ${catkin_LIBRARIES}
)
