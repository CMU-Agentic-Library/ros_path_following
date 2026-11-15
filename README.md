# ROS Path Following

A simple path following demo for ROS 2. A simulated vehicle follows predefined paths and is visualized in RViz.

Tested on **Ubuntu 24.04** with **ROS 2 Jazzy**.

## Packages

| Package | Description |
| --- | --- |
| `path_sender` | Loads paths from text files and publishes them on the `/path` topic |
| `path_follower` | Follows the received path and generates vehicle commands |
| `vehicle_simulator` | Simulates the vehicle and launches the full system with RViz |

## Build and Run

In a terminal, go to the `path_following_ros_ws` folder, compile, source the workspace, and launch:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
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
