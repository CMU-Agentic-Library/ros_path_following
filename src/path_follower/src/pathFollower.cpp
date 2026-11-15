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
double lookAheadDis = 0.5;
double speed = 0.5;
double accel = 0.5;
double slowDwnDis = 1.0;
double yawRate = 1.0;
double yawRateGain = 1.0;
double goalYawDis = 1.0;
double goalStopDis = 0.05;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleYaw = 0;
float vehicleSpeed = 0;

int pathSize = 0;
int pathPointID = 1;
int slowDwnPointID = 0;
int goalYawPointID = 0;
float goalYaw = 0;
bool goalReach = false;

double odomTime = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr path(new pcl::PointCloud<pcl::PointXYZ>());
std_msgs::msg::Empty goalReachMsg;
geometry_msgs::msg::PointStamped lookAheadMsgs;

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
  while (dis < slowDwnDis && slowDwnPointID > 0) {
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
  nh->declare_parameter<double>("slowDwnDis", slowDwnDis);
  nh->declare_parameter<double>("yawRate", yawRate);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("goalYawDis", goalYawDis);
  nh->declare_parameter<double>("goalStopDis", goalStopDis);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("speed", speed);
  nh->get_parameter("accel", accel);
  nh->get_parameter("slowDwnDis", slowDwnDis);
  nh->get_parameter("yawRate", yawRate);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("goalYawDis", goalYawDis);
  nh->get_parameter("goalStopDis", goalStopDis);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  auto pubGoalReach = nh->create_publisher<std_msgs::msg::Empty>("/goal_reach", 5);

  auto pubLookAhead = nh->create_publisher<geometry_msgs::msg::PointStamped>("/look_ahead", 5);

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

      float disGoal = 1000.0;
      float speedRamped = speed;
      if (pathPointID >= slowDwnPointID) {
        float disGoalX = path->points[pathSize - 1].x - vehicleX;
        float disGoalY = path->points[pathSize - 1].y - vehicleY;
        disGoal = sqrt(disGoalX * disGoalX + disGoalY * disGoalY);
        if (disGoal < slowDwnDis) speedRamped *= disGoal / slowDwnDis;
        if (disGoal < goalStopDis) speedRamped = 0;
      }

      if (vehicleSpeed < speedRamped) vehicleSpeed += accel / 100.0;
      else if (vehicleSpeed > speedRamped) vehicleSpeed -= accel / 100.0;

      float vehicleFwdSpeed = (disX * cos(vehicleYaw) + disY * sin(vehicleYaw)) / dis * vehicleSpeed;
      float vehicleLeftSpeed = (-disX * sin(vehicleYaw) + disY * cos(vehicleYaw)) / dis * vehicleSpeed;

      float pathYaw = goalYaw;
      if (pathPointID < goalYawPointID) {
        pathYaw = atan2(path->points[pathPointID].y - path->points[pathPointID - 1].y, path->points[pathPointID].x - path->points[pathPointID - 1].x);
      }

      float disYaw = pathYaw - vehicleYaw;
      if (disYaw < -PI) disYaw += 2 * PI;
      else if (disYaw > PI) disYaw -= 2 * PI;

      float vehicleYawRate = yawRateGain * disYaw;
      if (vehicleYawRate < -yawRate) vehicleYawRate = -yawRate;
      else if (vehicleYawRate > yawRate) vehicleYawRate = yawRate;

      if (disGoal < goalStopDis && fabs(vehicleSpeed) < accel / 100.0 && fabs(vehicleYawRate) < yawRate / 100.0 && !goalReach) {
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
