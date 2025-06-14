#include "slam_hub.h"

using namespace SLAM::Localization;

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_node");

  // 初始化异步日志系统，日志等级、输出目录等在 LogManager::Init 中控制
  // 参数说明：
  // 1. 日志输出目录
  // 2. 单个日志文件的最大大小（MB）
  // 3. 最大保留的日志文件数量
  // 目录：logs，单个文件最大50MB，最多保留5个文件
  LogManager::Init("logs", 50, 5);

  SlamHub localization_node;

  ros::waitForShutdown();
  return 0;
}