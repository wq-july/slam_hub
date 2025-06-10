#include "slam_hub.h"

namespace SLAM {
namespace Localization {

SlamHub::SlamHub() : nh_("~"), spinner_(10) {
  // 加载参数
  std::string image_topic, imu_topic, lidar_topic;
  nh_.param("image_topic", image_topic, std::string("/camera/image_raw"));
  nh_.param("imu_topic", imu_topic, std::string("/imu/data"));
  nh_.param("lidar_topic", lidar_topic, std::string("/velodyne_points"));

  // 初始化SDK，SDK初始化函数就一直在跑，一直等待数据过来进行处理

  // 设置消息过滤器和订阅者
  image_sub_.subscribe(nh_, image_topic, 10);
  imu_sub_.subscribe(nh_, imu_topic, 100);
  lidar_sub_.subscribe(nh_, lidar_topic, 10);

  // 各个数据单开线程去订阅原始的传感器数据
  imu_sub_.registerCallback(&SlamHub::SetImu, this);
  image_sub_.registerCallback(&SlamHub::SetImage, this);
  lidar_sub_.registerCallback(&SlamHub::SetPointCloud, this);

  // 启动异步spinner
  spinner_.start();
}

// IMU回调函数
void SlamHub::SetImu(const sensor_msgs::ImuConstPtr &msg) {}

// 图像数据处理
void SlamHub::SetImage(const sensor_msgs::ImageConstPtr &msg) {}

// 激光雷达数据处理
void SlamHub::SetPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {}

} // namespace Localization

} // namespace SLAM
