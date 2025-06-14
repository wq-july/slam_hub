#include "slam_hub.h"
#include "common/data_type/can.hpp"
#include <unordered_map>

namespace SLAM {
namespace Localization {

SlamHub::SlamHub() : nh_("~"), spinner_(10) {
  // 加载参数
  std::string image_topic, can_topic, imu_topic, lidar_topic;
  nh_.param("image_topic", image_topic, std::string("/camera/image_raw"));
  nh_.param("imu_topic", imu_topic, std::string("/imu/data_raw"));
  nh_.param("can_topic", can_topic, std::string("/can/wheel_speed"));
  nh_.param("lidar_topic", lidar_topic, std::string("/velodyne_points"));

  // 初始化SDK，SDK初始化函数就一直在跑，一直等待数据过来进行处理
  // 创建DR配置
  SLAM::Localization::DRConfig dr_config;
  auto *imu_config = dr_config.mutable_imu_predict_config();

  // 初始化DeadReckoning对象
  dead_reckoning_ =
      std::make_unique<SLAM::Localization::DeadReckoning>(dr_config);

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
  imu_data.meas_ts_ms = msg->header.stamp.sec;
  imu_data.recv_time = ros::Time::now().toNSec();

  // 设置角速度
  imu_data.gyr.x() = msg->angular_velocity.x;
  imu_data.gyr.y() = msg->angular_velocity.y;
  imu_data.gyr.z() = msg->angular_velocity.z;

  // 设置线加速度
  imu_data.acc.x() = msg->linear_acceleration.x;
  imu_data.acc.y() = msg->linear_acceleration.y;
  imu_data.acc.z() = msg->linear_acceleration.z;

  // 传递给Dead Reckoning处理
  if (dead_reckoning_) {
    dead_reckoning_->SetIMU(imu_data);
  }
}

// Can数据处理
void SlamHub::SetCan(const slam_hub::can::ConstPtr &msg) {
  SLAM::Localization::Can can;

  can.mea_ts_ms = msg->header.stamp.sec;
  can.recv_ts_ns = ros::Time::now().toNSec();

  const static std::unordered_map<int, double> dir_map = {
      {0, 0.0}, {1, 1.0}, {2, -1.0}};

  can.velocity_rl = msg->rear_left_speed * dir_map.at(msg->rear_left_direction);
  can.velocity_rr =
      msg->rear_right_speed * dir_map.at(msg->rear_right_direction);

  // 传递给Dead Reckoning处理
  if (dead_reckoning_) {
    dead_reckoning_->SetCan(can);
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
