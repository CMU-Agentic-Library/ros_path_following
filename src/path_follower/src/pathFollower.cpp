#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
double lookAheadDis = 0.15;
double speed = 0.4;
double accel = 0.5;
double yawRate = 0.6;
double slowDis = 0.4;
double slowAngle = 0.3;
double stopDis = 0.05;
double stopAngle = 0.04;
double goalYawDis = 0.5;
double minRelZ = -0.15;
double maxRelZ = 1.5;
double collisionRadiusFromCenter = 0.3;
double collisionCenterDis = 0.3;
int collisionPointThre = 5;
int collisionPointCount = 0;
bool collisionStop = false;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleYaw = 0;
float vehicleSpeed = 0;

float lookAheadX = 0;
float lookAheadY = 0;

int pathSize = 0;
int pathPointID = 1;
int slowDwnPointID = 0;
int goalYawPointID = 0;
float goalYaw = 0;
bool goalReach = false;

double odomTime = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr path(new pcl::PointCloud<pcl::PointXYZ>());

std_msgs::msg::Empty goalReachMsg;
geometry_msgs::msg::PointStamped lookAheadMsgs;
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubLaserCloud;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) 
{
  if (odomTime <= 0 || pathSize <= 1 || !collisionStop) return;

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  float dis = sqrt(lookAheadX * lookAheadX + lookAheadY * lookAheadY);

  if (dis < 0.001) dis = 0.001;
  float sinLookAhead = lookAheadY / dis;
  float cosLookAhead = lookAheadX / dis;

  dis *= collisionCenterDis / lookAheadDis;
  if (dis > collisionCenterDis) dis = collisionCenterDis;

  laserCloudCrop->clear();
  collisionPointCount = 0;
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    float pointX1 = laserCloud->points[i].x - vehicleX;
    float pointY1 = laserCloud->points[i].y - vehicleY;
    float pointZ1 = laserCloud->points[i].z - vehicleZ;

    float pointX2 = pointX1 * cosLookAhead + pointY1 * sinLookAhead;
    float pointY2 = -pointX1 * sinLookAhead + pointY1 * cosLookAhead;
    float pointZ2 = pointZ1;

    float pointToLookAhead = sqrt((pointX2 - dis) * (pointX2 - dis) + pointY2 * pointY2);

    if (((pointX2 > 0 && pointX2 < dis && fabs(pointY2) < collisionRadiusFromCenter) || 
        (pointX2 >= dis && pointToLookAhead < collisionRadiusFromCenter)) && (pointZ2 > minRelZ && pointZ2 < maxRelZ)) {
      laserCloudCrop->push_back(laserCloud->points[i]);
      collisionPointCount++;
    }
  }

  sensor_msgs::msg::PointCloud2 laserCloud3;
  pcl::toROSMsg(*laserCloudCrop, laserCloud3);
  laserCloud3.header.stamp = laserCloud2->header.stamp;
  laserCloud3.header.frame_id = "map";
  pubLaserCloud->publish(laserCloud3);
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  pathSize = pathIn->poses.size();
  if (pathSize < 2) return;

  pathPointID = 1;
  goalReach = false;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = pathIn->poses[pathSize - 1].pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  goalYaw = yaw;

  path->points.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path->points[i].x = pathIn->poses[i].pose.position.x;
    path->points[i].y = pathIn->poses[i].pose.position.y;
    path->points[i].z = pathIn->poses[i].pose.position.z;
  }

  slowDwnPointID = pathSize - 1;
  float disX = path->points[pathSize - 1].x - path->points[slowDwnPointID].x;
  float disY = path->points[pathSize - 1].y - path->points[slowDwnPointID].y;
  float dis = sqrt(disX * disX + disY * disY);
  while (dis < slowDis && slowDwnPointID > 0) {
    slowDwnPointID--;
    disX = path->points[pathSize - 1].x - path->points[slowDwnPointID].x;
    disY = path->points[pathSize - 1].y - path->points[slowDwnPointID].y;
    dis = sqrt(disX * disX + disY * disY);
  }

  goalYawPointID = pathSize - 1;
  disX = path->points[pathSize - 1].x - path->points[goalYawPointID].x;
  disY = path->points[pathSize - 1].y - path->points[goalYawPointID].y;
  dis = sqrt(disX * disX + disY * disY);
  while (dis < goalYawDis && goalYawPointID > 0) {
    goalYawPointID--;
    disX = path->points[pathSize - 1].x - path->points[goalYawPointID].x;
    disY = path->points[pathSize - 1].y - path->points[goalYawPointID].y;
    dis = sqrt(disX * disX + disY * disY);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("speed", speed);
  nh->declare_parameter<double>("accel", accel);
  nh->declare_parameter<double>("yawRate", yawRate);
  nh->declare_parameter<double>("slowDis", slowDis);
  nh->declare_parameter<double>("slowAngle", slowAngle);
  nh->declare_parameter<double>("stopDis", stopDis);
  nh->declare_parameter<double>("stopAngle", stopAngle);
  nh->declare_parameter<double>("goalYawDis", goalYawDis);
  nh->declare_parameter<double>("minRelZ", minRelZ);
  nh->declare_parameter<double>("maxRelZ", maxRelZ);
  nh->declare_parameter<double>("collisionRadiusFromCenter", collisionRadiusFromCenter);
  nh->declare_parameter<double>("collisionCenterDis", collisionCenterDis);
  nh->declare_parameter<int>("collisionPointThre", collisionPointThre);
  nh->declare_parameter<bool>("collisionStop", collisionStop);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("speed", speed);
  nh->get_parameter("accel", accel);
  nh->get_parameter("yawRate", yawRate);
  nh->get_parameter("slowDis", slowDis);
  nh->get_parameter("slowAngle", slowAngle);
  nh->get_parameter("stopDis", stopDis);
  nh->get_parameter("stopAngle", stopAngle);
  nh->get_parameter("goalYawDis", goalYawDis);
  nh->get_parameter("minRelZ", minRelZ);
  nh->get_parameter("maxRelZ", maxRelZ);
  nh->get_parameter("collisionRadiusFromCenter", collisionRadiusFromCenter);
  nh->get_parameter("collisionCenterDis", collisionCenterDis);
  nh->get_parameter("collisionPointThre", collisionPointThre);
  nh->get_parameter("collisionStop", collisionStop);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  auto pubGoalReach = nh->create_publisher<std_msgs::msg::Empty>("/goal_reach", 5);

  auto pubLookAhead = nh->create_publisher<geometry_msgs::msg::PointStamped>("/look_ahead", 5);

  pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/collision_cloud", 5);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (odomTime > 0 && pathSize > 1) {
      float disX = path->points[pathPointID].x - vehicleX;
      float disY = path->points[pathPointID].y - vehicleY;
      float dis = sqrt(disX * disX + disY * disY);
      while (pathPointID < pathSize - 1) {
        float disNextX = path->points[pathPointID + 1].x - vehicleX;
        float disNextY = path->points[pathPointID + 1].y - vehicleY;
        float disNext = sqrt(disNextX * disNextX + disNextY * disNextY);

        if (dis < lookAheadDis || disNext < dis) {
          disX = disNextX;
          disY = disNextY;
          dis = disNext;
          pathPointID++;
        } else {
          break;
        }
      }

      lookAheadX = disX;
      lookAheadY = disY;

      float disGoal = 1000.0;
      float speedRamped = speed;
      if (pathPointID >= slowDwnPointID) {
        float disGoalX = path->points[pathSize - 1].x - vehicleX;
        float disGoalY = path->points[pathSize - 1].y - vehicleY;
        disGoal = sqrt(disGoalX * disGoalX + disGoalY * disGoalY);
        if (disGoal < slowDis) speedRamped *= disGoal / slowDis;
        if (disGoal < stopDis) speedRamped = 0;
      }

      if (vehicleSpeed < speedRamped) vehicleSpeed += accel / 100.0;
      else if (vehicleSpeed > speedRamped) vehicleSpeed -= accel / 100.0;
      if (collisionPointCount >= collisionPointThre) vehicleSpeed = 0;

      if (dis < 0.001) dis = 0.001;
      float vehicleFwdSpeed = (disX * cos(vehicleYaw) + disY * sin(vehicleYaw)) / dis * vehicleSpeed;
      float vehicleLeftSpeed = (-disX * sin(vehicleYaw) + disY * cos(vehicleYaw)) / dis * vehicleSpeed;

      float pathYaw = goalYaw;
      if (pathPointID < goalYawPointID) {
        pathYaw = atan2(path->points[pathPointID].y - path->points[pathPointID - 1].y, path->points[pathPointID].x - path->points[pathPointID - 1].x);
      }

      float disYaw = pathYaw - vehicleYaw;
      if (disYaw < -PI) disYaw += 2 * PI;
      else if (disYaw > PI) disYaw -= 2 * PI;

      float vehicleYawRate = yawRate / slowAngle * disYaw;
      if (disYaw < -slowAngle) vehicleYawRate = -yawRate;
      else if (disYaw > slowAngle) vehicleYawRate = yawRate;
      if (collisionPointCount >= collisionPointThre) vehicleYawRate = 0;

      if (disGoal < stopDis && fabs(vehicleSpeed) < accel / 100.0 && fabs(disYaw) < stopAngle && !goalReach) {
        pubGoalReach->publish(goalReachMsg);
        goalReach = true;
      }

      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      cmd_vel.header.frame_id = "vehicle";
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.linear.y = 0;
      cmd_vel.twist.angular.z = vehicleYawRate;
      if (fabs(vehicleSpeed) > accel / 100.0) {
        cmd_vel.twist.linear.x = vehicleFwdSpeed;
        cmd_vel.twist.linear.y = vehicleLeftSpeed;
      }
      pubSpeed->publish(cmd_vel);
      
      lookAheadMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      lookAheadMsgs.header.frame_id = "map";
      lookAheadMsgs.point.x = path->points[pathPointID].x;
      lookAheadMsgs.point.y = path->points[pathPointID].y;
      lookAheadMsgs.point.z = 0;
      pubLookAhead->publish(lookAheadMsgs);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}

//Add obstacle stopping, implement 1) or 2). Currently 1) is implemented in laserCloudHandler() callback
/*if (disGoal > stopDis) {
  1) Short distance obstacle checking - Note that lookAheadX and lookAheadY represent the vehicle moving direction, in a rectangle whose long direction is the vehicle moving direction and the width of the rectangle is the width of the vehicle, check if the number of registered lidar points (ground and ceiling removed) exceeds a threshold, if yes set vehicleSpeed = 0.
  2) Long distance obstacle checking - Note that path->points[pathPointID] is the look ahead, take the path points within a certain range before and after the look ahead, check the number of registered lidar points (ground and ceiling removed) whose distances to the closest path points are less than the vehicle width / 2, if the number of registered lidar points exceeds a threshold, set vehicleSpeed = 0.
}*/
