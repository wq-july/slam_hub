#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "slam_hub/can.h"
#include <memory>

// 引入 slam_sdk 的 DeadReckoning 类
#include "common/config/pb/dr_config.pb.h"
#include "modules/dead_reckoning/dead_reckoning.h"

namespace SLAM {
namespace Localization {

class SlamHub {

public:
  SlamHub();
  ~SlamHub() = default;

  // IMU数据处理
  void SetImu(const sensor_msgs::ImuConstPtr &msg);

  // Can数据处理
  void SetCan(const slam_hub::can::ConstPtr &msg);

  // 图像数据处理
  void SetImage(const sensor_msgs::ImageConstPtr &msg);

  // 激光雷达数据处理
  void SetPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;

  // 订阅各个传感器数据
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  message_filters::Subscriber<slam_hub::can> can_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;

  // SLAM算法
  std::unique_ptr<SLAM::Localization::DeadReckoning> dead_reckoning_;
};

} // namespace Localization
} // namespace SLAM