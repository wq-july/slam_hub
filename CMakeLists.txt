cmake_minimum_required(VERSION 3.0.2)
project(slam_hub)

# 开启 compile_commands.json 生成，便于 clangd 支持
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 设置 C++ 标准与 slam_sdk 一致
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  message_generation
)

## 自定义的消息类型
add_message_files(
  FILES
  can.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


## 查找 slam_sdk 需要的依赖库
find_package(Eigen3 REQUIRED)

## 设置 slam_sdk 路径
set(SLAM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../slam_sdk)

## 查找 slam_sdk 的动态库 - 库文件生成在 slam_hub/libs 目录下
set(SLAM_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)

# 检查库文件是否存在
if(NOT EXISTS "${SLAM_SDK_LIB_DIR}/libslam_core.so" OR 
   NOT EXISTS "${SLAM_SDK_LIB_DIR}/libcommon.so" OR 
   NOT EXISTS "${SLAM_SDK_LIB_DIR}/libproto_lib.so")
  message(FATAL_ERROR "slam_sdk libraries not found. Please build slam_sdk first")
endif()


## 包定义
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slam_hub
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs message_filters message_runtime
  DEPENDS EIGEN3
)

## 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SLAM_SDK_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

## 编译 slam_hub 库
add_library(slam_hub
  src/slam_hub.cpp
)

## 编译主节点
add_executable(localization_node
  src/localization_node.cpp
)

## 设置运行时动态库路径
set_target_properties(slam_hub PROPERTIES
  INSTALL_RPATH "${SLAM_SDK_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

set_target_properties(localization_node PROPERTIES
  INSTALL_RPATH "${SLAM_SDK_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

## 链接库 - 注意链接顺序很重要
target_link_libraries(slam_hub
  ${catkin_LIBRARIES}
  ${SLAM_SDK_LIB_DIR}/libcommon.so
  ${SLAM_SDK_LIB_DIR}/libproto_lib.so
  ${SLAM_SDK_LIB_DIR}/libslam_core.so
)

target_link_libraries(localization_node
  slam_hub
  ${SLAM_SDK_LIB_DIR}/libcommon.so
  ${SLAM_SDK_LIB_DIR}/libproto_lib.so
  ${SLAM_SDK_LIB_DIR}/libslam_core.so
  ${catkin_LIBRARIES}
)
