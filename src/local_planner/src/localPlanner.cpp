#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1 // 是否绘制路径集合

// 几何、感知和行为参数从私有 ROS 命名空间读取，正常调参应修改 launch 文件。
string pathFolder;
double vehicleLength = 0.85;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true; // 是否允许双向行驶,倒车
double laserVoxelSize = 0.05; // 激光点云体素滤波的体素大小
double terrainVoxelSize = 0.2; // 地形点云体素滤波的体素大小
bool useTerrainAnalysis = false; // 是否使用地形分析
bool checkObstacle = true; // 是否检查障碍物
bool checkRotObstacle = false; // 是否检查旋转障碍物
double adjacentRange = 3.5; // 邻近范围
double obstacleHeightThre = 0.2; // 障碍物高度阈值
double groundHeightThre = 0.1; // 地面高度阈值
double costHeightThre = 0.1; // 代价高度阈值
double costScore = 0.02; // 代价评分
bool useCost = false; // 是否使用代价
const int laserCloudStackNum = 1; // 激光点云栈的数量
int laserCloudCount = 0; // 当前激光点云计数
int pointPerPathThre = 2; // 每条路径的点数阈值
double minRelZ = -0.5; // 最小相对高度
double maxRelZ = 0.25; // 最大相对高度
double maxSpeed = 1.0; // 最大速度
double dirWeight = 0.02; // 方向权重，用于计算路径方向与车辆方向的偏差评分
double dirThre = 90.0; // 方向阈值，单位为度，用于判断路径方向与车辆方向的偏差, 超过该阈值则认为路径方向与车辆方向不一致
bool dirToVehicle = false; // 是否将路径方向与车辆方向对齐
double pathScale = 1.0; // 路径缩放系数
double minPathScale = 0.75; // 最小路径缩放系数
double pathScaleStep = 0.25; // 路径缩放步长
bool pathScaleBySpeed = true; // 是否根据速度调整路径缩放
double minPathRange = 1.0; // 最小路径范围
double pathRangeStep = 0.5; // 路径范围步长
bool pathRangeBySpeed = true; // 是否根据速度调整路径范围
bool pathCropByGoal = true; // 是否根据目标裁剪路径
bool noPathReverse = true; // 是否禁止路径倒车
bool autonomyMode = false; // 是否开启自主模式
double autonomySpeed = 1.0; // 自主模式下的速度
double joyToSpeedDelay = 2.0; // 操纵杆到速度的延迟
double joyToCheckObstacleDelay = 5.0; // 操纵杆到检查障碍物的延迟
double goalClearRange = 0.5; // 目标清除范围
double goalX = 0; // 目标X坐标
double goalY = 0; // 目标Y坐标

float joySpeed = 0; // 当前操纵杆速度
float joySpeedRaw = 0; // 原始操纵杆速度
float joyDir = 0; // 当前操纵杆方向

// 离线路径库契约。pathNum 和网格尺寸必须与 path_generator.py 及 paths/ 中的文件一致，
// 不能只修改 launch 参数。
const int pathNum = 343; // 路径数量
const int groupNum = 7; // 路径组数量
float gridVoxelSize = 0.02; // 栅格体素大小
float searchRadius = 0.55; // 搜索半径
float gridVoxelOffsetX = 5.0; // 栅格体素在X方向的偏移
float gridVoxelOffsetY = 4.5; // 栅格体素在Y方向的偏移
const int gridVoxelNumX = 251; // 栅格体素在X方向的数量
const int gridVoxelNumY = 451; // 栅格体素在Y方向的数量
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY; // 栅格体素总数量

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum]; // 激光点云栈
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 规划点云
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>()); // 裁剪后的规划点云
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 边界点云
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>()); // 新增障碍物点云
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum]; // 起始路径点云数组
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum]; // 路径点云数组
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>()); // 空闲路径点云
#endif

int pathList[pathNum] = {0}; // 路径列表
float endDirPathList[pathNum] = {0}; // 路径终点方向列表
int clearPathList[36 * pathNum] = {0}; // 清除路径列表
float pathPenaltyList[36 * pathNum] = {0}; // 路径惩罚列表
float clearPathPerGroupScore[36 * groupNum] = {0}; // 每组清除路径得分
std::vector<int> correspondences[gridVoxelNum]; // 栅格体素对应的点云索引列表

bool newLaserCloud = false; // 是否有新的激光点云
bool newTerrainCloud = false; // 是否有新的地形点云

double odomTime = 0; // 里程计时间戳
double joyTime = 0; // 操纵杆时间戳

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0; // 车辆姿态
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; // 车辆位置

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter; // 激光点云和地形点云下采样滤波器

/**
 * @brief 动态参数配置回调函数
 * @param config 动态参数配置对象
 * @param level 参数级别
 */
void reconfigureCallback(local_planner::LocalPlannerConfig &config, uint32_t)
{
  twoWayDrive = config.twoWayDrive;
  checkObstacle = config.checkObstacle;
  checkRotObstacle = config.checkRotObstacle;
  useCost = config.useCost;
  autonomyMode = config.autonomyMode;
  maxSpeed = config.maxSpeed;
  autonomySpeed = config.autonomySpeed;
  adjacentRange = config.adjacentRange;
  obstacleHeightThre = config.obstacleHeightThre;
  groundHeightThre = config.groundHeightThre;
  costHeightThre = config.costHeightThre;
  costScore = config.costScore;
  pointPerPathThre = config.pointPerPathThre;
  dirWeight = config.dirWeight;
  dirThre = config.dirThre;
  pathScale = config.pathScale;
  minPathScale = config.minPathScale;
  pathScaleStep = config.pathScaleStep;
  pathRangeBySpeed = config.pathRangeBySpeed;
  minPathRange = config.minPathRange;
  pathRangeStep = config.pathRangeStep;
  pathCropByGoal = config.pathCropByGoal;
  goalClearRange = config.goalClearRange;
}

/**
 * @brief 里程计回调函数，用于更新车辆的位姿信息
 * @param odom 里程计消息指针
 */
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 用 sensorOffset 将传感器位姿换算为车辆中心位姿，后者用于碰撞检查和候选路径放置。
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

/**
 * @brief 激光点云回调函数，用于更新车辆周围的点云信息
 * @param laserCloud2 激光点云消息指针
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {
    // 原始点云模式是地形图模式的替代方案：仅保留近处点，并在昂贵的路径碰撞循环前下采样。
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

/**
 * @brief 地形点云回调函数，用于更新车辆周围的地形信息
 * @param terrainCloud2 地形点云消息指针
 */
void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {
    // 地形图 intensity 是相对地面高度，不是雷达反射强度；保留硬障碍，也可将较低高度作为软代价。
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

/**
 * @brief 操纵杆回调函数，用于更新手动行驶的方向和速度
 * @param joy 操纵杆消息指针
 */
void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  // PS3 风格轴提供手动行驶方向和速度；触发轴还选择自主模式及是否启用障碍检查。
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

/**
 * @brief 目标点回调函数，用于更新车辆的目标位置
 * @param goal 目标点消息指针
 */
void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

/**
 * @brief 速度回调函数，用于更新车辆的速度信息
 * @param speed 速度消息指针
 */
void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

/**
 * @brief 边界回调函数，用于更新车辆周围的边界信息
 * @param boundary 边界消息指针
 */
void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  // 将多边形每条边离散成稠密的高 intensity 障碍点；重复点使其达到规划器的碰撞阈值。
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}
/**
 * @brief 外部添加障碍回调函数，用于更新车辆周围的外部添加障碍信息
 * @param addedObstacles2 外部添加障碍点云消息指针
 */
void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  // 外部标注无论原始数值为何，始终视为硬障碍。
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}
/**
 * @brief 障碍检查回调函数，用于更新车辆的障碍检查状态
 * @param checkObs 障碍检查消息指针
 */
void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  double checkObsTime = ros::Time::now().toSec();

  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

/**
 * @brief 双向行驶回调函数，用于更新车辆的双向行驶状态
 * @param twoWayDr 双向行驶消息指针
 */
void twoWayDriveHandler(const std_msgs::Bool::ConstPtr& twoWayDr)
{
  twoWayDrive = twoWayDr->data;
}

/**
 * @brief 读取 PLY 文件头函数，用于获取顶点数量
 * @param filePtr PLY 文件指针
 * @return 顶点数量
 */
int readPlyHeader(FILE *filePtr)
{
  // 规划器 PLY 均为 ASCII 格式；后续按生成器固定行布局解析前，只需读取顶点数。
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

/**
 * @brief 读取起始路径函数
 */
void readStartPaths()
{
  // 起始路径是每个输出组的短代表路径。实际发布的是选中的组，而非 343 条测试曲线之一。
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
/**
 * @brief 读取完整路径函数
 */
void readPaths()
{
  // 完整路径在加载时下采样并供可视化使用；碰撞检查本身使用预计算的对应表。
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

/**
 * @brief 读取路径列表函数
 */
void readPathList()
{
  // 保存候选路径所属分组和末端方向，以供后续评分。
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

/**
 * @brief 读取路径对应关系函数
 */
void readCorrespondences()
{
  // 每行格式为：体素编号、零个或多个被阻塞路径编号、-1 结束标记；该倒排索引是规划器的关键优化。
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("gridVoxelSize", gridVoxelSize);
  nhPrivate.getParam("searchRadius", searchRadius);
  nhPrivate.getParam("gridVoxelOffsetX", gridVoxelOffsetX);
  nhPrivate.getParam("gridVoxelOffsetY", gridVoxelOffsetY);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("noPathReverse", noPathReverse);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);

  dynamic_reconfigure::Server<local_planner::LocalPlannerConfig> server;
  dynamic_reconfigure::Server<local_planner::LocalPlannerConfig>::CallbackType callback = reconfigureCallback;
  server.setCallback(callback);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/terrain_map", 5, terrainCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5, boundaryHandler);

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> ("/added_obstacles", 5, addedObstaclesHandler);

  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool> ("/check_obstacle", 5, checkObstacleHandler);

  ros::Subscriber subTwoWayDrive = nh.subscribe<std_msgs::Bool> ("/two_way_drive", 5, twoWayDriveHandler);

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);
  nav_msgs::Path path;

  #if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> ("/free_paths", 2);
  #endif

  // 如需查看叠加扫描，可在此处恢复对应发布者。

  printf ("\nReading path files.\n");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();

  printf ("\nInitialization complete.\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    // 每次感知更新触发一次完整重规划；启用时使用 terrain_map，否则使用 registered_scan。
    if (newLaserCloud || newTerrainCloud) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      // 将全局坐标系下的点云转换到车辆坐标系下，计算距离用于筛选并裁剪出车辆周围的点云
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        // 为什么转到车辆坐标系下？
        // 因为后续的路径规划和障碍物检测都是在车辆坐标系下进行的，所以需要将全局坐标系下的点云转换到车辆坐标系下。

        // 为什么后续的路径规划和障碍物检测都是在车辆坐标系下进行？
        // 因为**车辆的传感器数据和控制命令**都是基于车辆自身的坐标系，所以在车辆坐标系下进行路径规划和障碍物检测更加直观和高效。
        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      int boundaryCloudSize = boundaryCloud->points.size();
      // 将边界点云转换到车辆坐标系下，并裁剪出车辆周围的点云。
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      // 将新增障碍物点云转换到车辆坐标系下，并裁剪出车辆周围的点云。
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      // 将所有障碍表示到车体系。候选路径也定义在该系中，+x 朝前、+y 朝左。
      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange) pathRange = minPathRange;
      float pathRangeInit = pathRange;
      float relativeGoalDis = adjacentRange;

      if (autonomyMode) {
        // 自主模式中，用 map 坐标系目标替代手柄方向。
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        // 如果车辆不支持双向行驶，则限制手柄方向在 [-90, 90] 范围内。
        if (!twoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0;
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale) pathScale = minPathScale;
      float pathScaleInit = pathScale;
      bool isReverse = false;
      
      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        // 找不到可行路径时逐步缩小候选范围，较短路径更可能穿过车辆近处的拥挤区域。
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float relativeGoalDis2 = relativeGoalDis;
        float joyDir2 = joyDir;
        if (isReverse) {
          relativeGoalDis2 = adjacentRange;
          joyDir2 += 180.0;
          if (joyDir2 > 180.0) joyDir2 -= 360.0;
        }

        float minObsAngCW = -180.0; // 顺时针方向上最小的障碍物角度
        float minObsAngCCW = 180.0; // 逆时针方向上最小的障碍物角度
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

          // 如果障碍物距离在路径规划范围内，并且在目标点附近或不按目标裁剪，同时需要进行障碍物检测，则进行路径阻塞计算。
          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis2 + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir2 - (10.0 * rotDir - 180.0));
              // 如果角度差大于 180 度，则取补角，使角度差在 [0, 180] 范围内。
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              // 如果角度差在允许范围内，则继续进行路径阻塞计算。
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir2) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir2) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              // 如果体素索引在有效范围内，则进行路径阻塞计算。
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                // 一个占据体素会阻塞所有扫掠车体区域覆盖它的候选路径，无需在线逐条做几何查询。
                int blockedPathByVoxelNum = correspondences[ind].size();
                // 遍历所有被该体素阻塞的路径索引，更新路径的清晰度或惩罚值。
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          // 如果障碍物在路径规划范围内，并且在车辆长度或宽度之外，同时高度超过障碍物高度阈值或不使用地形分析，并且需要进行旋转障碍物检测，则计算障碍物角度。计算的障碍物角度用于旋转障碍物的避让策略。
          if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
              (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        // 如果旋转障碍物的最小角度超出范围，则将其限制在合理范围内。
        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        // 遍历所有路径组，计算每条路径的得分，并根据旋转方向和障碍物角度进行加权。
        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum); // 计算当前路径的旋转方向索引
          float angDiff = fabs(joyDir2 - (10.0 * rotDir - 180.0)); // 计算当前路径的角度与操纵杆方向的差值
          // 如果角度差超过 180 度，则取补角，使角度差在 [0, 180] 范围内。
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }

          // 如果角度差在允许范围内，则继续进行路径评分计算。
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir2) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir2) > 90.0 && dirToVehicle)) {
            continue;
          }
          
          // 如果候选路径的碰撞数低于硬阈值，则计算路径的得分。
          if (clearPathList[i] < pointPerPathThre) {
            // 碰撞数低于硬阈值的候选路径可行；较低的地形高度可选地作为软代价降低分数。
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre; // 根据地形高度计算路径的惩罚分数，超过阈值会降低分数。
            if (penaltyScore < costScore) penaltyScore = costScore; // 保证惩罚分数不低于最低分数。

            float dirDiff = fabs(joyDir2 - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0)); // 计算当前路径的方向与操纵杆方向的差值
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW; // 计算旋转方向的权重，根据旋转方向索引的不同，给予不同的权重。
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1); // 对于前半部分的旋转方向，计算其权重。
            else rotDirW = fabs(fabs(rotDir - 27) + 1); // 对于后半部分的旋转方向，计算其权重。
            // 计算路径的最终得分，综合考虑方向差、旋转方向权重和惩罚分数。
            // 此公式的依据为：路径的方向差越小、旋转方向越接近理想值、地形高度越低，路径得分越高。
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore; 
            // 如果路径得分大于 0，则将其累加到对应的路径组得分中。
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        // 遍历所有路径组，选择得分最高且满足旋转角度约束的路径组。
        // 旋转角度约束是指路径组的旋转角度必须在最小顺时针角度和最小逆时针角度之间，或者在双向行驶模式下满足相应的约束。
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          float rotDeg = 10.0 * rotDir;
          if (rotDeg > 180.0) rotDeg -= 360.0;
          // 
          if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        // 如果找到了得分最高且满足旋转角度约束的路径组，则选中该路径组并生成对应的路径。
        // 这里生成的路径是对预先生成的路径进行旋转和平移后的结果，以适应当前的车辆姿态和选中的路径组。
        // 对预先生成的路径进行旋转和平移是从世界坐标系转换到车辆坐标系，以便在车辆坐标系下进行路径跟踪和控制。
        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          // 遍历选中路径组的每个路径点，将其从世界坐标系转换到车辆坐标系，并根据路径范围和相对目标距离进行裁剪。
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            // 如果路径点的距离在路径范围和相对目标距离之内，则保留该路径点，否则裁剪路径。
            if (dis <= pathRange / pathScale && dis <= relativeGoalDis2 / pathScale) {
              // 在规划时刻的车体系发布选中组；z 的正负是紧凑的前进/倒退方向标记。
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              if (isReverse) path.poses[i].pose.position.z = -0.001;
              else path.poses[i].pose.position.z = 0.001;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);

          #if PLOTPATHSET == 1
          freePaths->clear();
          // 遍历所有预先生成的路径，进行旋转和平移，并根据路径范围和相对目标距离进行裁剪，生成并绘制自由路径点云。
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir2 - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir2) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir2) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
              continue;
            }

            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis2 + goalClearRange) / pathScale || !pathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "vehicle";
          pubFreePaths.publish(freePaths2);
          #endif
        }

        // 如果没有选中任何路径组，则尝试通过调整路径缩放和路径范围来寻找可行路径。
        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else if (pathRange >= minPathRange + pathRangeStep || isReverse || !noPathReverse) {
            pathRange -= pathRangeStep;
          } else if (!isReverse && noPathReverse) {
            pathScale = pathScaleInit;
            pathRange = pathRangeInit;
            isReverse = true;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;
      // 如果在调整路径缩放和路径范围后仍未找到可行路径，则发布空路径。
      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2);
        #endif
      }

      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
