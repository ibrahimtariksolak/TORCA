# ROS2 Autonomous Vehicle Perception

A ROS2 (Foxy/Humble-style) perception and control stack for a simulated autonomous vehicle, combining YOLOv8-based traffic sign detection with a classic computer-vision lane-following controller. Built and tested in Gazebo.

## Features

- **Traffic sign detection** — a YOLOv8 model (`ultralytics`) trained to recognize stop signs, give-way, speed limits (20/30), no entry, and pedestrian crossing signs from the vehicle's camera feed.
- **Lane following** — a Canny/Hough-transform lane detector feeding a PID controller that computes steering and throttle commands.
- **Reactive behavior** — detected signs dynamically adjust vehicle speed or trigger stop/turn maneuvers (e.g. full stop on `Stop_Sign` or `No Entry`, slow-down on `Give Way` / `Pedestrian Crossing`).
- **Simulation environment** — a Gazebo world (highway/asphalt scene) with a URDF/Xacro vehicle model, spawned and driven entirely through ROS2 launch files.
- **Live visualization** — annotated detection frames and lane-tracking debug frames are published as image topics for inspection in RViz/`rqt_image_view`.
- **Standalone camera test node** — a small utility node for testing the trained model against a live webcam feed, independent of the simulation.

## Architecture

```
                 ┌────────────────────┐
   /arac/kamera/ │                    │ /detected_sign
   image_raw ───▶│   sign_detector    │───────────────┐
                 │  (YOLOv8 inference)│                │
                 └────────────────────┘                │
                          │ /sign_detector/annotated    │
                          ▼                             ▼
                 ┌────────────────────┐        ┌────────────────────┐
   /arac/kamera/ │                    │        │                    │
   image_raw ───▶│    serit_takip     │◀───────┤  reacts to signs,  │
   /arac/odom ──▶│ (lane following +  │        │  updates speed /   │
                 │  PID controller)   │        │  maneuver state    │
                 └────────────────────┘        └────────────────────┘
                          │
                          ├──▶ /arac/cmd_vel   (Twist commands to the vehicle)
                          └──▶ /serit_debug    (annotated lane-detection frames)
```

## Tech stack

| Component | Technology |
|---|---|
| Middleware | ROS2 (rclpy) |
| Object detection | YOLOv8 (`ultralytics`), OpenCV |
| Image transport | `cv_bridge`, `sensor_msgs/Image` |
| Control | Custom PID (Kp/Ki/Kd) over Hough-transform lane geometry |
| Simulation | Gazebo (`gzserver`/`gzclient`), URDF/Xacro |
| Build system | `ament_python`, `colcon` |

## Project structure

```
src/perception_pkg/
├── perception_pkg/
│   ├── sign_detector.py      # YOLOv8 traffic sign detection node
│   ├── serit_takip.py        # Lane following + PID control + sign reactions
│   └── weights/best.pt       # Trained YOLOv8 weights
├── kameraSignTest.py         # Standalone webcam test node for the detection model
├── launch/
│   └── otonom_surus.launch.py  # Full simulation + perception + control stack
├── models/                   # Gazebo camera sensor models
├── worlds/                   # Gazebo simulation world(s)
├── urdf/                     # Vehicle URDF/Xacro description
├── test/                     # ament linting/style tests
└── package.xml / setup.py
```

## Getting started

### Prerequisites

- ROS2 (Foxy or newer) with `gazebo_ros`, `cv_bridge`, `rviz2`
- Python 3.10+
- `ultralytics`, `opencv-python`

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pkg
source install/setup.bash
```

### Run the full simulation

```bash
ros2 launch perception_pkg otonom_surus.launch.py
```

This starts Gazebo with the vehicle spawned in the world, brings up the sign detector and lane-following nodes, and opens RViz.

### Test the detection model against a webcam

```bash
ros2 run perception_pkg kameraSignTest
```

## License

No license has been specified yet — all rights reserved by default until one is added. Open an issue or reach out if you'd like to use this code and need licensing terms clarified.
