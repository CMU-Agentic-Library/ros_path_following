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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

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

string pathFile1;
string pathFile2;
string pathFile3;
double goalYaw1 = 0;
double goalYaw2 = 0;
double goalYaw3 = 0;
double goalDelay = 0;
double initDelay = 0;
bool looping = true;

int pathSize1 = 0;
int pathSize2 = 0;
int pathSize3 = 0;

double odomTime = 0;
double initTime = 0;
double goalTime = 0;
bool goalReach = false;
int state = 0;

nav_msgs::msg::Path path1, path2, path3;

int readPath(string pathFile, nav_msgs::msg::Path &path, float goalYaw)
{
  pathFile.replace(pathFile.find("/install/"), 8, "/src");
  FILE *pathFilePtr = fopen(pathFile.c_str(), "r");

  int val1, val2, val3;
  float x, y, z;
  geometry_msgs::msg::PoseStamped pose;
  while (1) {
    val1 = fscanf(pathFilePtr, "%f", &x);
    val2 = fscanf(pathFilePtr, "%f", &y);
    val3 = fscanf(pathFilePtr, "%f", &z);

    if (val1 != 1 || val2 != 1 || val3 != 1) break;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    path.poses.push_back(pose);
  }

  fclose(pathFilePtr);
  
  int pathSize = path.poses.size();
  if (pathSize > 1) {
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, goalYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::convert(quat_tf, geoQuat);
    path.poses[pathSize - 1].pose.orientation = geoQuat;
  }
  
  return pathSize;
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
}

void goalReachHandler(const std_msgs::msg::Empty::ConstSharedPtr goalReachIn)
{
  goalReach = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("pathSender");

  nh->declare_parameter<std::string>("pathFile1", pathFile1);
  nh->declare_parameter<std::string>("pathFile2", pathFile2);
  nh->declare_parameter<std::string>("pathFile3", pathFile3);
  nh->declare_parameter<double>("goalYaw1", goalYaw1);
  nh->declare_parameter<double>("goalYaw2", goalYaw2);
  nh->declare_parameter<double>("goalYaw3", goalYaw3);
  nh->declare_parameter<double>("goalDelay", goalDelay);
  nh->declare_parameter<double>("initDelay", initDelay);
  nh->declare_parameter<bool>("looping", looping);

  nh->get_parameter("pathFile1", pathFile1);
  nh->get_parameter("pathFile2", pathFile2);
  nh->get_parameter("pathFile3", pathFile3);
  nh->get_parameter("goalYaw1", goalYaw1);
  nh->get_parameter("goalYaw2", goalYaw2);
  nh->get_parameter("goalYaw3", goalYaw3);
  nh->get_parameter("goalDelay", goalDelay);
  nh->get_parameter("initDelay", initDelay);
  nh->get_parameter("looping", looping);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subGoalReach = nh->create_subscription<std_msgs::msg::Empty>("/goal_reach", 5, goalReachHandler);

  pathSize1 = readPath(pathFile1, path1, goalYaw1);
  pathSize2 = readPath(pathFile2, path2, goalYaw2);
  pathSize3 = readPath(pathFile3, path3, goalYaw3);

  RCLCPP_INFO(nh->get_logger(), "Read %d points on path1,  %d points on path2, and %d points on path3.", pathSize1, pathSize2, pathSize3);

  if (pathSize1 < 2 || pathSize2 < 2 || pathSize3 < 2) {
    RCLCPP_INFO(nh->get_logger(), "No sufficient points on paths, exit.");
    return 0;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (state == 0 && odomTime > 0) {
      initTime = odomTime;
      state = 1;
    } else if (state == 1 && odomTime - initTime > initDelay) {
      RCLCPP_INFO(nh->get_logger(), "Sending path1.");
      path1.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      path1.header.frame_id = "map";
      pubPath->publish(path1);
      state = 2;
    } else if (state == 2 && goalReach) {
      goalReach = false;
      goalTime = odomTime;
      state = 3;
    } else if (state == 3 && odomTime - goalTime > goalDelay) {
      RCLCPP_INFO(nh->get_logger(), "Sending path2.");
      path2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      path2.header.frame_id = "map";
      pubPath->publish(path2);
      state = 4;
    } else if (state == 4 && goalReach) {
      goalReach = false;
      goalTime = odomTime;
      state = 5;
    } else if (state == 5 && odomTime - goalTime > goalDelay) {
      RCLCPP_INFO(nh->get_logger(), "Sending path3.");
      path3.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      path3.header.frame_id = "map";
      pubPath->publish(path3);
      state = 6;
    } else if (state == 6 && goalReach) {
      goalReach = false;
      goalTime = odomTime;
      state = 7;
    } else if (state == 7 && odomTime - goalTime > goalDelay) {
      if (looping) {
        RCLCPP_INFO(nh->get_logger(), "Sending path1.");
        path1.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        path1.header.frame_id = "map";
        pubPath->publish(path1);
        state = 2;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
