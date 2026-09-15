#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

// 此节点为需要传感器坐标系扫描及精确采集时刻位姿的算法适配 map 坐标系已配准扫描。
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>()); // 存储从上游接收到的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>()); // 存储转换到传感器坐标系的点云

double robotX = 0; // 机器人在地图坐标系中的 X 位置
double robotY = 0; // 机器人在地图坐标系中的 Y 位置
double robotZ = 0; // 机器人在地图坐标系中的 Z 位置
double roll = 0; // 机器人在地图坐标系中的滚转角
double pitch = 0; // 机器人在地图坐标系中的俯仰角
double yaw = 0; // 机器人在地图坐标系中的偏航角

bool newTransformToMap = false; // 标记是否有新的从传感器到地图的变换

nav_msgs::Odometry odometryIn; // 存储最新的里程计信息
ros::Publisher *pubOdometryPointer = NULL; // 指向里程计发布者的指针，方便在回调中使用
tf::StampedTransform transformToMap; // 存储从传感器到地图的变换
tf::TransformBroadcaster *tfBroadcasterPointer = NULL; //指向TF变换发布者的指针，方便在回调中使用

ros::Publisher pubLaserCloud; // 发布转换到传感器坐标系的点云

/**
 * @brief 处理来自上游的里程计和点云消息，将点云转换到传感器坐标系并发布
 * @param odometry 上游里程计消息
 * @param laserCloud2 上游点云消息
 */
void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  // ApproximateTime 已配对这两条消息。输出时间戳始终采用扫描时间，代表物理采集时刻。
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf::Vector3 vec;

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    // registered_scan 位于 map 中。应用 inverse(map -> sensor) 恢复 sensor_at_scan 测得的点坐标。
    vec = transformToMap.inverse() * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = "map";
  transformToMap.child_frame_id_ = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformToMap);

  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 通常无法精确同步；从长度为 100 的队列中接受时间接近的位姿/扫描对，而非任意配对最新消息。
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, "/state_estimation", 1);
  subLaserCloud.subscribe(nh, "/registered_scan", 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 2);

  ros::spin();

  return 0;
}
