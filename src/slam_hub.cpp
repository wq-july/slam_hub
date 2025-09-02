#include "slam_hub.h"
#include "common/config/config_loader.hpp"
#include "common/config/pb/Localiztion.pb.h"
#include "common/data_type/can.hpp"
#include "common/tools/log/logger.hpp"
#include <unordered_map>

using namespace SLAM::Common;

namespace SLAM {
namespace Localization {

SlamHub::SlamHub() : nh_("~"), spinner_(10) {
  // 加载参数
  std::string image_topic, can_topic, imu_topic, lidar_topic;
  nh_.param("image_topic", image_topic, std::string("/camera/image_raw"));
  nh_.param("imu_topic", imu_topic, std::string("/imu/data_raw"));
  nh_.param("can_topic", can_topic, std::string("/can/wheel_speed"));
  nh_.param("lidar_topic", lidar_topic, std::string("/velodyne_points"));

  SLAM::Localization::LocalizationConfig config;
  if (!LoadConfigFromTextFile(
          "/home/wq/Project/slam/src/slam_hub/config/localization.conf",
          &config)) {
    WQ_ERROR(WQ) << "Read config failed!";
    return;
  }

  // 构造函数完成时，各个模块就已经在运行了
  system_ = std::make_unique<SLAM::Localization::LocSystem>(config);

  // 设置消息过滤器和订阅者
  image_sub_.subscribe(nh_, image_topic, 10);
  imu_sub_.subscribe(nh_, imu_topic, 100);
  can_sub_.subscribe(nh_, can_topic, 50);
  lidar_sub_.subscribe(nh_, lidar_topic, 10);

  // 各个数据单开线程去订阅原始的传感器数据
  imu_sub_.registerCallback(&SlamHub::SetImu, this);
  can_sub_.registerCallback(&SlamHub::SetCan, this);
  image_sub_.registerCallback(&SlamHub::SetImage, this);
  lidar_sub_.registerCallback(&SlamHub::SetPointCloud, this);

  // 启动异步spinner
  spinner_.start();
}

// IMU回调函数
void SlamHub::SetImu(const sensor_msgs::ImuConstPtr &msg) {
  // 将ROS IMU消息转换为SLAM SDK的IMU类型
  SLAM::Localization::Imu imu_data;
  // 正确设置时间戳
  imu_data.meas_ts_s = msg->header.stamp.toSec();   // 秒单位
  imu_data.meas_ts_ns = msg->header.stamp.toNSec(); // 纳秒单位
  imu_data.recv_ts_ns = ros::Time::now().toNSec();

  // 设置角速度
  imu_data.gyr.x() = msg->angular_velocity.x;
  imu_data.gyr.y() = msg->angular_velocity.y;
  imu_data.gyr.z() = msg->angular_velocity.z;

  // 设置线加速度
  imu_data.acc.x() = msg->linear_acceleration.x;
  imu_data.acc.y() = msg->linear_acceleration.y;
  imu_data.acc.z() = msg->linear_acceleration.z;

  if (system_) {
    system_->SetImu(msg->header.stamp.toNSec(), imu_data);
  }
}

// Can数据处理
void SlamHub::SetCan(const slam_hub::can::ConstPtr &msg) {
  SLAM::Localization::Can can;

  // 正确设置时间戳
  can.meas_ts_ns = msg->header.stamp.toNSec(); // 纳秒单位
  can.meas_ts_s = msg->header.stamp.toSec();   // 秒单位
  can.recv_ts_ns = ros::Time::now().toNSec();

  const static std::unordered_map<int, double> dir_map = {
      {0, 0.0}, {1, 1.0}, {2, -1.0}};

  can.velocity_rl = msg->rear_left_speed * dir_map.at(msg->rear_left_direction);
  can.velocity_rr =
      msg->rear_right_speed * dir_map.at(msg->rear_right_direction);

  if (system_) {
    system_->SetCan(msg->header.stamp.toNSec(), can);
  }
}

// 图像数据处理
void SlamHub::SetImage(const sensor_msgs::ImageConstPtr &msg) {}

// 激光雷达数据处理
void SlamHub::SetPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // 这里可以处理激光雷达数据，如果slam_sdk支持的话
}

} // namespace Localization

} // namespace SLAM
