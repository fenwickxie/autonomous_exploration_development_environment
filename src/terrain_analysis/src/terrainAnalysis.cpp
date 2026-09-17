#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

// 从 terrain_analysis.launch 读取的配置，定义时序地图窗口、点高度范围和障碍解释方式。
double scanVoxelSize = 0.05;      // 扫描点云体素滤波尺寸，用于下采样，单位为米
double decayTime = 2.0;           // 衰减时间，用于动态障碍的时间衰减
double noDecayDis = 4.0;          // 无衰减距离，用于动态障碍的空间衰减
double clearingDis = 8.0;         // 清除距离，用于动态障碍的空间清除
bool clearingCloud = false;       // 是否清除点云，用于动态障碍的空间清除
bool useSorting = true;           // 是否使用排序，排序点云以提高处理效率
double quantileZ = 0.25;          // 高程分位数，用于地面高度估计
bool considerDrop = false;        // 是否考虑落差
bool limitGroundLift = false;     // 是否限制地面抬升，地面抬升指的是地面高度相对于之前估计的提升
double maxGroundLift = 0.15;      // 最大地面抬升
bool clearDyObs = false;          // 是否清除动态障碍
double minDyObsDis = 0.3;         // 动态障碍最小距离，用于判断动态障碍是否接近车辆
double minDyObsAngle = 0;         // 动态障碍最小角度，用于判断动态障碍是否接近车辆的角度限制
double minDyObsRelZ = -0.5;       // 动态障碍最小相对高度，相对于地面的高度
double absDyObsRelZThre = 0.2;    // 动态障碍相对高度阈值，用于判断动态障碍的高度变化是否显著
double minDyObsVFOV = -16.0;      // 动态障碍最小垂直视场，指的是动态障碍在垂直方向上的可见范围下限
double maxDyObsVFOV = 16.0;       // 动态障碍最大垂直视场，指的是动态障碍在垂直方向上的可见范围上限
int minDyObsPointNum = 1;         // 动态障碍最小点数
bool noDataObstacle = false;      // 无数据是否视为障碍
int noDataBlockSkipNum = 0;       // 无数据块跳过数量
int minBlockPointNum = 10;        // 最小块点数
double vehicleHeight = 1.5;       // 车辆高度
int voxelPointUpdateThre = 100;   // 体素点更新阈值
double voxelTimeUpdateThre = 2.0; // 体素时间更新阈值
double minRelZ = -1.5;            // 最小相对高度，指的是相对车辆的高度
double maxRelZ = 0.2;             // 最大相对高度，指的是相对车辆的高度
double disRatioZ = 0.2;           // 高度距离比

// 以车辆为中心的 21 x 21 滚动三维点云格；车辆跨越格边界时复用对应单元。
float terrainVoxelSize = 1.0;     // 每个格子大小，单位为米
int terrainVoxelShiftX = 0;       // 车辆在三维点云格中的 X 方向偏移
int terrainVoxelShiftY = 0;       // 车辆在三维点云格中的 Y 方向偏移
const int terrainVoxelWidth = 21; // 三维点云格的宽度（格子数量）
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2; // 三维点云格的半宽度（格子数量的一半）
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 三维点云格的总格子数量

// 更稠密的二维高程网格，仅用于估计局部地面高度。高程网格指的是在二维平面上划分的网格，每个格子存储对应区域的地面高度信息。
float planarVoxelSize = 0.2;                           // 每个格子大小，单位为米
const int planarVoxelWidth = 51;                       // 二维高程网格的宽度（格子数量）
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2; // 二维高程网格的半宽度（格子数量的一半）
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth; // 二维高程网格的总格子数量

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 原始激光点云
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 裁剪后的激光点云
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 下采样后的激光点云
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 地形点云
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());             // 地形点云的高程信息
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum]; // 三维点云格对应的点云数组

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};    // 每个三维点云格的更新次数
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 每个三维点云格的更新时间
float planarVoxelElev[planarVoxelNum] = {0};         // 每个二维高程网格的高程值
int planarVoxelEdge[planarVoxelNum] = {0};           // 每个二维高程网格的边缘标记
int planarVoxelDyObs[planarVoxelNum] = {0};          // 每个二维高程网格的动态障碍标记
vector<float> planarPointElev[planarVoxelNum];       // 每个二维高程网格的点高程列表

double laserCloudTime = 0;  // 激光点云时间戳
bool newlaserCloud = false; // 是否有新的激光点云到达

double systemInitTime = 0; // 系统初始化时间
bool systemInited = false; // 系统是否初始化完成
int noDataInited = 0;      // 未观测区域初始化状态

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0; // 车辆的滚转角、俯仰角和偏航角
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;          // 车辆在地图坐标系中的位置
float vehicleXRec = 0, vehicleYRec = 0;                  // 记录车辆的初始位置，用于判断是否移动足够距离

float sinVehicleRoll = 0, cosVehicleRoll = 0;   // 车辆滚转角的正弦和余弦
float sinVehiclePitch = 0, cosVehiclePitch = 0; // 车辆俯仰角的正弦和余弦
float sinVehicleYaw = 0, cosVehicleYaw = 0;     // 车辆偏航角的正弦和余弦

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 点云下采样滤波器

/**
 * 处理里程计消息的状态估计回调函数，更新车辆的位姿和三角函数值，更新未观测区域初始化状态。
 * @param odom 里程计消息指针
 */
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  // 缓存位姿和三角函数值，因为主循环内每个点变换都使用最新车辆位姿。
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw); // 从四元数中提取滚转角、俯仰角和偏航角

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0)
  {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1)
  {
    // 车辆移动足够距离后才启用未知区域处理，避免将启动瞬态误判为未观测区域。
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

/**
 * 已配准激光扫描回调函数，裁减保留车辆附近的点云，并更新点云到达状态。
 * @param laserCloud2 激光点云消息指针
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
  // 保留车辆附近 map 坐标系中的点。intensity 暂存观测年龄（激光点云时间戳减去系统初始化时间），发布前会重写为高程。
  laserCloudTime = laserCloud2->header.stamp.toSec();

  if (!systemInited)
  {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++)
  {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY)); // 计算点到车辆的水平距离

    // 保留车辆附近的点，判定条件为点的相对高度在[minRelZ, maxRelZ]范围内且水平距离小于指定阈值（terrainVoxelSize * (terrainVoxelHalfWidth + 1)）
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1))
    {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true; //点云到达状态更新
}

// 手柄回调函数
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  // 5 号按键清理局部地图状态，并重新开始未知区域初始化。
  if (joy->buttons[5] > 0.5)
  {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// 点云清理回调函数
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrainAnalysis");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("considerDrop", considerDrop);
  nhPrivate.getParam("limitGroundLift", limitGroundLift);
  nhPrivate.getParam("maxGroundLift", maxGroundLift);
  nhPrivate.getParam("clearDyObs", clearDyObs);
  nhPrivate.getParam("minDyObsDis", minDyObsDis);
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
  nhPrivate.getParam("absDyObsRelZThre", absDyObsRelZThre);
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
  nhPrivate.getParam("noDataObstacle", noDataObstacle);
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);

  ros::Subscriber subOdometry =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subClearing =
      nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);

  ros::Publisher pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

  for (int i = 0; i < terrainVoxelNum; i++)
  {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (newlaserCloud)
    {
      newlaserCloud = false;

      // 当前实现注意：每帧都会重新初始化所有滚动单元，因此跨帧累积观测会在滚动前丢弃。
      // 这里保持该行为不变，因为修改它会改变地图算法行为。
      for (int i = 0; i < terrainVoxelNum; i++)
      {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>()); // 重新初始化每个滚动单元的点云，丢弃跨帧累积观测。
      }

      // 车辆离开中心格时移动指针所有权而不复制点云；新暴露的外侧行/列会被清空。
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX; // 当前滚动栅格中心的 X 坐标
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY; // 当前滚动栅格中心的 Y 坐标

      // 当车辆离开中心格的左边界时，滚动栅格
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY]; // 保存最右侧列的点云指针，准备滚动
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY]; // 将左侧的点云移动到右侧
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      // 当车辆离开中心格的右边界时，滚动栅格
      while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }
      
      // 当车辆离开中心格的下边界时，滚动栅格
      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // 当车辆离开中心格的上边界时，滚动栅格
      while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // 将当前点插入以车辆为中心的 1 m 单元做时序过滤；此时点 intensity 仍保存初始化后的秒数。
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++)
      {
        point = laserCloudCrop->points[i];

        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth; // 将点的 X 坐标转换为栅格索引
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth; // 将点的 Y 坐标转换为栅格索引

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--; // 如果点在左边界之外，索引减一
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--; // 如果点在下边界之外，索引减一

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) // 如果点在栅格范围内，插入对应的栅格
        {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++; // 更新该栅格的更新次数
        }
      }

      // 对每个栅格进行更新，满足更新条件的栅格进行下采样和时序过滤
      for (int ind = 0; ind < terrainVoxelNum; ind++)
      {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) // 如果栅格更新次数达到阈值，或者时间超过阈值，或者请求清理，则进行更新
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          // 下采样后仅保留最近点，邻近 no-decay 半径内除外；请求清理时也会删除近处点。
          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++)
          {
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud))
            {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      // 提取车辆周围5x5的地形点云
      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 5;
           indX <= terrainVoxelHalfWidth + 5; indX++)
      {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++)
        {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // 每个源点为相邻 3 x 3 平面格投票，在提取障碍前平滑格边界的地面估计。
      for (int i = 0; i < planarVoxelNum; i++)
      {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];

        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth; // 将点的坐标转换为平面栅格索引，并加上半宽度以对齐栅格中心
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth; // 将点的坐标转换为平面栅格索引，并加上半宽度以对齐栅格中心

        if (point.x - vehicleX + planarVoxelSize / 2 < 0) // 如果点在栅格左边界之外，索引减一
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) // 如果点在栅格下边界之外，索引减一
          indY--;

        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) // 如果点的相对高度在最小和最大阈值之间，进行平面格投票
        {
          for (int dX = -1; dX <= 1; dX++)
          {
            for (int dY = -1; dY <= 1; dY++)
            {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) // 如果相邻格在平面栅格范围内，进行投票
              {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z); // 将点的高度加入相邻平面格的高度列表，用于后续平滑处理
              }
            }
          }
        }

        if (clearDyObs)
        {
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis)
            {
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              if (angle1 > minDyObsAngle)
              {
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV || fabs(pointZ4) < absDyObsRelZThre)
                {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            }
            else
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;
            }
          }
        }
      }

      if (clearDyObs)
      {
        // 统计符合传感器垂直视场的点，后续用于排除归因于动态遮挡物的单元。
        for (int i = 0; i < laserCloudCropSize; i++)
        {
          point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle)
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      // 根据配置选择使用低分位数还是最小值来估计地面高度，避免地面上方物体影响地面估计。
      if (useSorting)
      {
        // 使用低分位数而非均值估计地面，避免地面上方物体将均值抬高。
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0)
          {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = int(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&
                limitGroundLift)
            {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            }
            else
            {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      }
      else
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0)
          {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++)
            {
              if (planarPointElev[i][j] < minZ)
              {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1)
            {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      // 遍历地形点云，将符合条件（相对高度在[minRelZ, maxRelZ]范围内，且所在平面格满足观测要求）的点加入地形高度点云，并计算其相对高度。
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                    minDyObsPointNum ||
                !clearDyObs)
            {
              float disZ =
                  point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              // 地形图 intensity 是导航代价：高于地面的高度；考虑坑洼时为绝对高差。
              if (considerDrop)
                disZ = fabs(disZ);
              int planarPointElevSize =
                  planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight &&
                  planarPointElevSize >= minBlockPointNum)
              {
                terrainCloudElev->push_back(point);
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      // 处理未观测平面区域，将其标记为障碍格，并根据配置进行扩张形成安全余量。
      if (noDataObstacle && noDataInited == 2)
      {
        // 将未观测平面区域标为障碍格，再按配置网格步数扩张边界形成安全余量。
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum)
          {
            planarVoxelEdge[i] = 1;
          }
        }

        // 按配置的步数扩张未观测平面区域的边界，形成安全余量。
        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++)
        {
          for (int i = 0; i < planarVoxelNum; i++)
          {
            if (planarVoxelEdge[i] >= 1)
            {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++)
              {
                for (int dY = -1; dY <= 1; dY++)
                {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth)
                  {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i])
                    {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }

        // 将扩张后的未观测平面区域生成地形高度点云，用于表示障碍物。
        for (int i = 0; i < planarVoxelNum; i++)
        {
          if (planarVoxelEdge[i] > noDataBlockSkipNum)
          {
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;

      // 在 map 坐标系发布紧凑的可通行性表示。
      sensor_msgs::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud.publish(terrainCloud2);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
