# PFW Lunabotics 2026 — Autonomy Stack

ROS 2 (Humble) workspace for Purdue Fort Wayne's NASA Lunabotics 2026 robot.
The stack provides LiDAR-only perception and localization, Nav2-based
navigation, motion-safety control, and an autonomous mission controller for
excavation/traverse/deposition cycles.

> **License:** This source is published for public **viewing and review only**.
> It may not be copied, modified, used, or redistributed without written
> permission. See [LICENSE](LICENSE) for full terms. Third-party components
> under `src/point_lio_ros2/` and `src/unilidar_sdk2/` are governed by their
> own licenses.

## Sensor

Single sensor design: **Unitree 4D LiDAR L2** (point cloud + built-in IMU).
No cameras or depth sensors — perception and localization are geometric.

## Package Layout

```
src/
├── luna_msgs/                # Custom message definitions
├── lunabotics_description/   # URDF, meshes, TF tree
├── lunabotics_gazebo/        # Simulation worlds and arena models
├── lunabotics_perception/    # Hazard pipeline, zone + localization-quality monitors
├── lunabotics_localization/  # Point-LIO + EKF (robot_localization)
├── lunabotics_navigation/    # Nav2 configuration, maps, waypoint navigator
├── lunabotics_control/       # cmd_vel safety relay, teleop, Pico serial bridge
├── lunabotics_autonomy/      # Mission controller, excavation bridge
├── point_lio_ros2/           # Third-party: Point-LIO (LiDAR-inertial odometry)
└── unilidar_sdk2/            # Third-party: Unitree L2 LiDAR SDK + ROS 2 driver
```

`mission_control_vm/` contains a PyQt operator GUI for driving the robot from a
ground-station machine (see its own README).

## Build

Requires ROS 2 Humble on Ubuntu 22.04.

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run (simulation)

```bash
# Full simulation with autonomy
ros2 launch lunabotics_navigation bringup.launch.py use_rviz:=true

# Start an autonomous mission
ros2 service call /mission/start std_srvs/srv/Trigger
```

## Run (hardware)

```bash
# Real robot: Point-LIO localization, no simulator
ros2 launch lunabotics_navigation bringup.launch.py use_point_lio:=true
```

Individual subsystems can be launched via each package's `launch/` directory.

---

Maintained by the PFW Lunabotics Team · `lunabotics@pfw.edu`
