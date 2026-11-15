The code is tested in Ubuntu 24.04 with ROS2 Jazzy. In a terminal, go to the path_following_ros_ws folder, compile, source the ROS workspace, and launch the software. The vehicle frame should move and follow the path in RVIZ.

  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.sh
  ros2 launch vehicle_simulator vehicle_simulator.launch

Note: The path is given as a set of dense points at 10cm interval. Recommend to keep the interval between 5-10cm. Each path needs to have at least 2 points. The path files are at "src/path_sender/data" folder. Each line is a point (x, y, z) on the path. When the path is send to the path_follower node on the "/path" topic, the goalYaw is stored in the "path.poses[pathSize - 1].pose.orientation" field of the last path point.
