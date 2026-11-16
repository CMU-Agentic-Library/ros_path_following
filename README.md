# ROS Path Following

A simple path following demo for ROS 2. A simulated vehicle follows predefined paths and is visualized in RViz.

Tested on **Ubuntu 24.04** with **ROS 2 Jazzy**.

## Packages

| Package | Description |
| --- | --- |
| `path_sender` | Loads paths from text files and publishes them on the `/path` topic |
| `path_follower` | Follows the received path and generates vehicle commands |
| `vehicle_simulator` | Simulates the vehicle and launches the full system with RViz |
| `h1_bringup` | Real robot: launch files and the `/cmd_vel` → `zeno_h1_motion_control` bridge |
| `arise_slam_mid360`, `arise_slam_mid360_msgs` | Real robot: lidar-inertial SLAM, publishes `/state_estimation` (copied from autonomy_stack_malo) |
| `livox_ros_driver2` | Real robot: Livox message definitions only (`CustomMsg`), needed by arise_slam; the driver runs on the lower computer |
| `src/slam/dependency` | Real robot: Sophus 1.1, Ceres 2.2, GTSAM 4.3 sources for arise_slam (copied from autonomy_stack_malo; not ROS packages, skipped by colcon) |

## Build and Run

In a terminal, go to the `path_following_ros_ws` folder, compile, source the workspace, and launch:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select path_follower path_sender vehicle_simulator
source install/setup.sh
ros2 launch vehicle_simulator vehicle_simulator.launch
```

The vehicle frame should move and follow the path in RViz.

## Path Format

- Path files are in [`src/path_sender/data`](src/path_sender/data).
- Each line is a point `x y z` on the path.
- Paths are given as dense points at a 10 cm interval. Keeping the interval between 5 and 10 cm is recommended.
- Each path must have at least 2 points.
- When a path is sent to the `path_follower` node on the `/path` topic, the goal yaw is stored in `path.poses[pathSize - 1].pose.orientation`, i.e. the orientation of the last path point.

## Real Robot (Zeno H1)

```
lower computer: /livox/lidar, /livox/imu -> livoxInputAdapter -> arise_slam --/state_estimation--> pathFollower
  --/cmd_vel--> cmdVelToMotionControl
  --/zeno/h1/auto/wholebody/cmd--> zeno_h1_motion_control --/zeno/h1/mobilechassis/cmd--> zeno_h1_driver
```

`cmdVelToMotionControl` drives the base through motion_control's AUTO input. AUTO is motion_control's lowest-priority mode, so the joystick, VR, or whole-body teleop can take over at any time.

### Dependencies

- `zeno_h1_msgs`, `zeno_h1_driver` and `zeno_h1_motion_control` on their ROS 2 branches (`master` of the driver and msgs is ROS 1), built in this workspace or an underlay.
- Sophus, Ceres and GTSAM, installed system-wide from [`src/slam/dependency`](src/slam/dependency) (see below).
- The Mid-360 driver runs on the robot's lower computer, so Livox-SDK2 is not needed here.
- If conda is active, add `-DPython3_EXECUTABLE=/usr/bin/python3` to the `--cmake-args`.

### Third-party libraries (once per machine)

arise_slam needs Sophus, Ceres and GTSAM installed system-wide. The sources are in [`src/slam/dependency`](src/slam/dependency), at the versions arise_slam was built against. Each folder has a `CATKIN_IGNORE`, so colcon skips them.

System packages (Ubuntu 24.04 / ROS 2 Jazzy):

```bash
sudo apt install libeigen3-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
  libboost-all-dev libtbb-dev libpcl-dev \
  ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs
```

Then build and install the three libraries, in this order:

```bash
cd src/slam/dependency/Sophus
mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF
make -j6 && sudo make install

cd ../../ceres-solver
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j6 && sudo make install

cd ../../gtsam
mkdir build && cd build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j6 && sudo make install
sudo /sbin/ldconfig
```

Delete the `build` folders afterwards, or leave them: they are git-ignored. GTSAM takes the longest; lower `-j` if the machine runs out of memory.

### Build the workspace

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.sh
```

### Run

One terminal per step, each with the workspace sourced:

| Step | Command | Provides |
| --- | --- | --- |
| 0 | start `zeno_h1_driver`, `zeno_h1_motion_control` and the lidar on the lower computer as usual | chassis, arms, `/livox/lidar`, `/livox/imu` |
| 1 | `ros2 launch h1_bringup slam.launch.py` | `/state_estimation`, TF `map` → `sensor` |
| 2 | `ros2 launch h1_bringup follower.launch.py sensorOffsetX:=<m> sensorOffsetY:=<m>` | `/cmd_vel` |
| 3 | `ros2 launch h1_bringup bridge.launch.py` | AUTO commands to motion_control |
| 4 | `python3 scripts/h1_uturn.py --odom /state_estimation --send`, or `ros2 launch h1_bringup sender.launch.py` | a path on `/follow_path` |

- Start SLAM (step 1) with the robot at the start pose: the map origin is wherever the lidar is when SLAM starts.
- `sensorOffsetX/Y` is the lidar position relative to the chassis center (forward / left).
- `slam.launch.py` starts `livoxInputAdapter`, which subscribes best-effort to `lidar_topic` (default `/livox/lidar`) and `imu_topic` (`/livox/imu`), converts the cloud to `CustomMsg` if needed, and republishes reliably on `/lidar/scan` and `/imu/data` for arise_slam. Set `lidar_format` to match `ros2 topic info /livox/lidar`: `pointcloud2` (default) for `sensor_msgs/msg/PointCloud2`, `custom` for `livox_ros_driver2/msg/CustomMsg`. The PointCloud2 must be the Livox driver's format (`xfer_format` 0, with per-point `timestamp`).
- Arguments: `follower.launch.py` takes `speed` (default 0.2 m/s), `yawRate` (0.75 rad/s) and `path_topic` (`/follow_path`). `bridge.launch.py` takes `maxLinear` (0.2), `maxAngular` (1.0) and `checkOdom` (true). `sender.launch.py` takes `path_topic` and `looping` (false).

### How the bridge behaves

- AUTO messages also carry the torso, arm and gripper targets. The bridge fills them with the measured pose from motion_control's joint_state topics, and holds that pose while AUTO is in control. It does not send AUTO until it has joint states.
- It publishes at 50 Hz while `/cmd_vel` is fresher than 0.25 s and `/state_estimation` is fresher than 0.3 s. Otherwise it sends mode 0 for 0.5 s, which makes motion_control zero the chassis and go IDLE, then stops publishing.
- `ros2 topic pub --once /chassis_estop std_msgs/msg/Bool "{data: true}"` releases AUTO the same way, and stays on until `{data: false}` is sent.
- In IDLE, motion_control sets the arm smoother target to all zeros (`upper_body_ctrl_->Reset(zeros)`), so the arms may return to the zero pose after the bridge releases AUTO. This is motion_control's own IDLE behavior; check it on the robot first.
- If the bridge dies without sending mode 0 (a hard kill), motion_control on `feature/ros2` keeps the last AUTO chassis velocity. Keep the joystick or a physical e-stop within reach.
