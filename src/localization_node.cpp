#include "slam_hub.h"

using namespace SLAM::Localization;

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_node");

  SlamHub localization_node;

  ros::waitForShutdown();
  return 0;
}