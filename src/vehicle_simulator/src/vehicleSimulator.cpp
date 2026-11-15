#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

rclcpp::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleFwdSpeed = 0;
float vehicleLeftSpeed = 0;

void speedHandler(const geometry_msgs::msg::TwistStamped::ConstSharedPtr speedIn)
{
  vehicleFwdSpeed = speedIn->twist.linear.x;
  vehicleLeftSpeed = speedIn->twist.linear.y;
  vehicleYawRate = speedIn->twist.angular.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vehicleSimulator");

  nh->declare_parameter<double>("vehicleX", vehicleX);
  nh->declare_parameter<double>("vehicleY", vehicleY);
  nh->declare_parameter<double>("vehicleZ", vehicleZ);
  nh->declare_parameter<double>("vehicleYaw", vehicleYaw);

  nh->get_parameter("vehicleX", vehicleX);
  nh->get_parameter("vehicleY", vehicleY);
  nh->get_parameter("vehicleZ", vehicleZ);
  nh->get_parameter("vehicleYaw", vehicleYaw);

  auto subSpeed = nh->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5, speedHandler);

  auto pubVehicleOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);
  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";

  auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
  tf2::Stamped<tf2::Transform> odomTrans;
  geometry_msgs::msg::TransformStamped transformTfGeom ; 
  odomTrans.frame_id_ = "map";

  RCLCPP_INFO(nh->get_logger(), "Simulation started.");
  
  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status)
  {
    rclcpp::spin_some(nh);

    vehicleYaw += 0.01 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.01 * cos(vehicleYaw) * vehicleFwdSpeed - 0.01 * sin(vehicleYaw) * vehicleLeftSpeed;
    vehicleY += 0.01 * sin(vehicleYaw) * vehicleFwdSpeed + 0.01 * cos(vehicleYaw) * vehicleLeftSpeed;

    odomTime = nh->now();

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::convert(quat_tf, geoQuat);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 0;
    odomData.twist.twist.angular.y = 0;
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleFwdSpeed;
    odomData.twist.twist.linear.y = vehicleLeftSpeed;
    odomData.twist.twist.linear.z = 0;
    pubVehicleOdom->publish(odomData);

    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(vehicleX, vehicleY, vehicleZ));
    transformTfGeom = tf2::toMsg(odomTrans);
    transformTfGeom.child_frame_id = "sensor";
    transformTfGeom.header.stamp = odomTime;
    tfBroadcaster->sendTransform(transformTfGeom);

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
