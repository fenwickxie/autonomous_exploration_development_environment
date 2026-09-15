#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

// 参数定义运动学车辆和地形跟随近似；Gazebo 负责传感器渲染，本节点维护导航状态。
bool use_gazebo_time = false; // 是否使用 Gazebo 时间戳匹配扫描与位姿, 避免点云拖影,如果使用 Gazebo 时间则扫描会根据时间戳与历史位姿对齐，保证点云与车辆状态的一致性。
double cameraOffsetZ = 0; // 相机在车辆坐标系下的 Z 方向偏移
double sensorOffsetX = 0; // 传感器在车辆坐标系下的 X 方向偏移
double sensorOffsetY = 0; // 传感器在车辆坐标系下的 Y 方向偏移
double vehicleHeight = 0.75; // 车辆高度
double terrainVoxelSize = 0.05; // 地形体素滤波尺寸，地形体素是用于简化地形点云的体素网格大小
double groundHeightThre = 0.1; // 地面高度阈值
bool adjustZ = false; // 是否调整 Z 方向
double terrainRadiusZ = 0.5; // Z 方向地形半径
int minTerrainPointNumZ = 10; // Z 方向地形最小点数
double smoothRateZ = 0.2; // Z 方向平滑系数
bool adjustIncl = false; // 是否调整地形倾角
double terrainRadiusIncl = 1.5; // 倾角地形半径
int minTerrainPointNumIncl = 500; // 倾角地形最小点数
double smoothRateIncl = 0.2; // 倾角平滑系数
double InclFittingThre = 0.2; // 倾角拟合阈值
double maxIncl = 30.0; // 最大倾角

const int systemDelay = 5; // 系统初始化延迟，单位为帧数，用于保证系统在接收到足够的传感器数据后再开始处理
int systemInitCount = 0;  // 系统初始化计数器，用于记录已经接收到的传感器数据帧数
bool systemInited = false; // 系统是否初始化完成，用于判断是否可以开始处理传感器数据

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>()); // 存储扫描数据
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 存储地形点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>()); // 存储地形倾角点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 存储地形下采样点云

std::vector<int> scanInd;

ros::Time odomTime;

float vehicleX = 0; // 车辆在世界坐标系下的 X 位置
float vehicleY = 0; // 车辆在世界坐标系下的 Y 位置
float vehicleZ = 0; // 车辆在世界坐标系下的 Z 位置
float vehicleRoll = 0; // 车辆在世界坐标系下的滚转角
float vehiclePitch = 0; // 车辆在世界坐标系下的俯仰角
float vehicleYaw = 0; // 车辆在世界坐标系下的偏航角

float vehicleYawRate = 0; // 车辆偏航角速度
float vehicleSpeed = 0; // 车辆速度

float terrainZ = 0; // 地形在世界坐标系下的 Z 位置
float terrainRoll = 0; // 地形在世界坐标系下的滚转角
float terrainPitch = 0; // 地形在世界坐标系下的俯仰角

const int stackNum = 400; // 历史状态栈的大小
// 历史状态将延迟的 Gazebo 传感器扫描与最近的仿真位姿对齐，用于时间同步。
float vehicleXStack[stackNum]; // 车辆历史 X 位置栈
float vehicleYStack[stackNum]; // 车辆历史 Y 位置栈
float vehicleZStack[stackNum]; // 车辆历史 Z 位置栈
float vehicleRollStack[stackNum]; // 车辆历史滚转角栈
float vehiclePitchStack[stackNum]; // 车辆历史俯仰角栈
float vehicleYawStack[stackNum]; // 车辆历史偏航角栈
float terrainRollStack[stackNum]; // 地形历史滚转角栈
float terrainPitchStack[stackNum]; // 地形历史俯仰角栈
double odomTimeStack[stackNum]; // 里程计时间戳栈
int odomSendIDPointer = -1; // 里程计发送指针
int odomRecIDPointer = 0; // 里程计接收指针

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter; // 地形下采样滤波器

ros::Publisher* pubScanPointer = NULL; // 扫描数据发布器指针

/**
 * 处理输入的点云消息，更新车辆和地形的状态信息。
 * @param scanIn 输入的点云消息
 */
void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn)
{
  // 如果系统尚未初始化，直接返回，等待足够的传感器数据帧数
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
    return;
  }

  double scanTime = scanIn->header.stamp.toSec(); // 扫描时间戳

  if (odomSendIDPointer < 0) // 如果里程计发送指针无效，直接返回
  {
    return;
  }
  // 更新里程计接收指针，使其指向不晚于当前扫描时间戳的最新记录
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  double odomRecTime = odomTime.toSec(); // 记录的里程计时间戳
  float vehicleRecX = vehicleX; // 记录的车辆 X 位置
  float vehicleRecY = vehicleY; // 记录的车辆 Y 位置
  float vehicleRecZ = vehicleZ; // 记录的车辆 Z 位置
  float vehicleRecRoll = vehicleRoll; // 记录的车辆滚转角
  float vehicleRecPitch = vehiclePitch; // 记录的车辆俯仰角
  float vehicleRecYaw = vehicleYaw; // 记录的车辆偏航角
  float terrainRecRoll = terrainRoll; // 记录的地形滚转角
  float terrainRecPitch = terrainPitch; // 记录的地形俯仰角

  if (use_gazebo_time)
  {
    // 用扫描时间戳匹配记录的车辆/地形状态，而非最新位姿，避免已配准点云发生空间拖影。
    odomRecTime = odomTimeStack[odomRecIDPointer]; // 用扫描时间戳匹配的里程计时间戳
    vehicleRecX = vehicleXStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆 X 位置
    vehicleRecY = vehicleYStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆 Y 位置
    vehicleRecZ = vehicleZStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆 Z 位置
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆滚转角
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆俯仰角
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer]; // 用扫描时间戳匹配的车辆偏航角
    terrainRecRoll = terrainRollStack[odomRecIDPointer]; // 用扫描时间戳匹配的地形滚转角
    terrainRecPitch = terrainPitchStack[odomRecIDPointer]; // 用扫描时间戳匹配的地形俯仰角
  }

  float sinTerrainRecRoll = sin(terrainRecRoll); // 地形滚转角的正弦值
  float cosTerrainRecRoll = cos(terrainRecRoll); // 地形滚转角的余弦值
  float sinTerrainRecPitch = sin(terrainRecPitch); // 地形俯仰角的正弦值
  float cosTerrainRecPitch = cos(terrainRecPitch); // 地形俯仰角的余弦值

  scanData->clear(); // 清空点云数据容器
  pcl::fromROSMsg(*scanIn, *scanData); // 将 ROS 消息转换为 PCL 点云
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd); // 移除点云中的 NaN 点

  int scanDataSize = scanData->points.size(); // 点云数据的点数
  for (int i = 0; i < scanDataSize; i++) // 遍历每个点
  {
    // 撤销地形对齐旋转，再将每个传感器系射线终点平移到 map，形成已配准扫描。
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
    float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    float pointX3 = pointX2 + vehicleRecX;
    float pointY3 = pointY2 + vehicleRecY;
    float pointZ3 = pointZ2 + vehicleRecZ;

    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
  }

  // 发布 5 Hz 已配准扫描消息
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
}

/**
 * 处理输入的地形点云消息，更新地形高度和斜率信息。
 * @param terrainCloud2 输入的地形点云消息
 * @note 仅在 adjustZ 或 adjustIncl 为 true 时才会处理地形点云。
 * @note 地形高度和斜率信息将用于车辆的姿态调整和路径规划。
 * @note 该函数会根据地形点云更新全局变量 terrainZ、terrainPitch 和 terrainRoll。
 */
void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  // 选择车辆附近低代价地形，用于高度和斜率拟合。
  pcl::PointXYZI point; // 临时存储地形点云中的点
  terrainCloudIncl->clear(); // 清空地形倾角点云容器
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0; // 临时存储车辆附近地形点的平均高度
  int elevCount = 0; // 车辆附近地形点的数量
  bool terrainValid = true; // 地形数据是否有效
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    // 计算点到车辆的水平距离
    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
    // 如果点在车辆附近的高度有效区域内，则用于高度拟合
    if (dis < terrainRadiusZ)
    {
      if (point.intensity < groundHeightThre) // 点的强度小于地面高度阈值，认为是地面点
      {
        elevMean += point.z;
        elevCount++;
      }
      else // 点的强度大于等于地面高度阈值，认为是非地面点，地形数据无效
      {
        terrainValid = false;
        break; // 发现非地面点，地形数据无效，跳出循环
      }
    }

    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre) // 如果点在车辆附近的倾角有效区域内且为地面点，则用于倾角拟合
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (!terrainValid)  // 一旦发现无效，直接返回
  {
    return;
  }

  if (elevCount >= minTerrainPointNumZ) // 如果车辆附近地形点数量足够，则计算平均高度
  {
    elevMean /= elevCount;
  }
  else
  {
    terrainValid = false; // 如果车辆附近地形点数量不足，则地形数据无效
    return; // 如果车辆附近地形点数量不足，则直接返回
  }

  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  terrainCloudDwz->clear(); // 清空地形下采样点云容器
  terrainDwzFilter.setInputCloud(terrainCloudIncl); // 设置地形下采样滤波器的输入点云
  terrainDwzFilter.filter(*terrainCloudDwz); // 执行地形下采样滤波
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  if (terrainCloudDwzSize < minTerrainPointNumIncl) // 如果下采样后的地形点云数量不足，则返回
  {
    return;
  }

  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 A
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 A 的转置矩阵
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 A 的转置矩阵与 A 的乘积
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 B
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 A 的转置矩阵与 B 的乘积
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0)); // 构建地形倾角拟合矩阵 X，用于存储拟合结果

  int inlierNum = 0; // 内点数量
  matX.at<float>(0, 0) = terrainPitch; // 初始化地形倾角拟合矩阵 X 的 pitch 分量
  matX.at<float>(1, 0) = terrainRoll; // 初始化地形倾角拟合矩阵 X 的 roll 分量
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    // 围绕局部平均高程迭代拟合 z = a*x + b*y；首次求解后移除残差离群点以排除障碍物。
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX; // 构建地形倾角拟合矩阵 A 的第一列
      matA.at<float>(i, 1) = point.y - vehicleY; // 构建地形倾角拟合矩阵 A 的第二列
      matB.at<float>(i, 0) = point.z - elevMean; // 构建地形倾角拟合矩阵 B 的值

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0) // 仅在迭代次数大于 0 时才将残差过大的点视为离群点，以排除障碍物
      {
        matA.at<float>(i, 0) = 0; // 将离群点对应的矩阵 A 的值置为 0
        matA.at<float>(i, 1) = 0; // 将离群点对应的矩阵 A 的值置为 0
        matB.at<float>(i, 0) = 0; // 将离群点对应的矩阵 B 的值置为 0
        outlierCount++;
      }
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); // 使用 QR 分解求解线性方程组，得到地形倾角拟合结果

    if (inlierNum == terrainCloudDwzSize - outlierCount) // 如果内点数量没有变化，则提前终止迭代
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0) // 如果内点数量不足或拟合的倾角超过最大允许倾角，则认为地形无效
  {
    terrainValid = false;
  }

  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

void speedHandler(const geometry_msgs::Twist::ConstPtr& speedIn)
{
  // 将 /cmd_vel 解释为期望车体前向速度和偏航角速度。
  vehicleSpeed = speedIn->linear.x;
  vehicleYawRate = speedIn->angular.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
  nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("vehicleX", vehicleX); // 获取车辆在地图坐标系下的 X 坐标
  nhPrivate.getParam("vehicleY", vehicleY); // 获取车辆在地图坐标系下的 Y 坐标
  nhPrivate.getParam("vehicleZ", vehicleZ); // 获取车辆在地图坐标系下的 Z 坐标
  nhPrivate.getParam("terrainZ", terrainZ); // 获取地形在地图坐标系下的 Z 坐标
  nhPrivate.getParam("vehicleYaw", vehicleYaw); // 获取车辆在地图坐标系下的偏航角
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize); // 获取地形体素的大小
  nhPrivate.getParam("groundHeightThre", groundHeightThre); // 获取地面高度阈值
  nhPrivate.getParam("adjustZ", adjustZ); // 是否调整车辆的 Z 轴位置
  nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ); // 获取地形 Z 轴拟合的半径
  nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ); // 获取地形 Z 轴拟合的最小点数
  nhPrivate.getParam("adjustIncl", adjustIncl); // 是否调整车辆的倾角
  nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl); // 获取地形倾角拟合的半径
  nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl); // 获取地形倾角拟合的最小点数
  nhPrivate.getParam("InclFittingThre", InclFittingThre); // 获取倾角拟合的阈值
  nhPrivate.getParam("maxIncl", maxIncl); // 获取车辆的最大倾角

  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, scanHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudHandler);

  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, speedHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("/state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";

  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState cameraState;
  cameraState.model_name = "camera";
  gazebo_msgs::ModelState lidarState;
  lidarState.model_name = "lidar";
  gazebo_msgs::ModelState robotState;
  robotState.model_name = "robot";

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
  pubScanPointer = &pubScan;

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    // 将地形系斜率转为车体系 roll/pitch，再以固定 0.005 s（200 Hz）步长积分平面运动学模型。
    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    ros::Time odomTimeRec = odomTime;
    odomTime = ros::Time::now();
    if (odomTime == odomTimeRec) odomTime += ros::Duration(0.005); // 如果当前里程计时间与上一次相同，则增加 0.005 秒，保证时间递增

    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum; // 更新里程计发送指针，循环使用栈空间
    odomTimeStack[odomSendIDPointer] = odomTime.toSec(); // 将当前里程计时间存入栈中
    vehicleXStack[odomSendIDPointer] = vehicleX; 
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    terrainRollStack[odomSendIDPointer] = terrainRoll;
    terrainPitchStack[odomSendIDPointer] = terrainPitch;

    // /state_estimation 是导航栈的统一位姿接口；角速度 x/y 编码模拟 roll/pitch 变化率供安全逻辑使用。
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll); // 角速度 x 编码模拟 roll 变化率供安全逻辑使用
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch); // 角速度 y 编码模拟 pitch 变化率供安全逻辑使用
    odomData.twist.twist.angular.z = vehicleYawRate; // 角速度 z 直接使用 yaw 变化率
    odomData.twist.twist.linear.x = vehicleSpeed; // 线速度 x 直接使用车辆前进速度
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ); // 线速度 z 编码模拟高度变化率供安全逻辑使用
    pubVehicleOdom.publish(odomData);

    // 保持 TF 与 Odometry 消息一致，供 RViz 和坐标变换使用。
    odomTrans.stamp_ = odomTime;
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    tfBroadcaster.sendTransform(odomTrans);

    // 将 Gazebo 中的可视化/传感器模型移动到刚积分得到的运动学状态。
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    pubModelState.publish(cameraState);

    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState.publish(robotState);

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(terrainRoll, terrainPitch, 0);

    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    pubModelState.publish(lidarState);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
