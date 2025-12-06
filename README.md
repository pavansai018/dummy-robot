# dummy-robot

A 4-wheeled robot simulation built using ROS 2 Jazzy and Gazebo Harmonic.  
Includes PID and LQR controllers for line following and a GUI tool for manual teleoperation.

---

## Features

- Differential-drive robot with clean URDF model  
- Gazebo Harmonic simulation worlds (square and circular tracks)  
- PID and LQR line-following controllers  
- GUI teleoperation  
- Compact and modular ROS 2 package  

---

## Project Structure

```
dummy-robot/
├── dummy_robot/        # Python package
├── nodes/              # Controller + GUI nodes
├── urdf/               # Robot description
├── meshes/             # Geometry files
├── worlds/             # Gazebo worlds
├── launch/             # Launch files
├── package.xml
└── setup.py
```

---

## Installation

```
cd ~/ros2_ws/src
git clone https://github.com/pavansai018/dummy-robot.git
```

Build:

```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Simulation

```
ros2 launch dummy_robot <launch_file>.launch.py
```

Loads:

- Gazebo Harmonic  
- Robot model  
- Selected world (square or circular track)  

---

## Teleoperation

```
ros2 run dummy_robot teleop_buttons
```

Simple GUI for testing robot motion.

---

## Line Following

### PID Controller

```
ros2 run dummy_robot line_follower_pid
```

GUI version:

```
ros2 run dummy_robot line_follower_pid_gui
```

### LQR Controller

```
ros2 run dummy_robot line_follower_lqr
```

Implements optimal state-feedback tracking.

---

## System Overview

```
GUI → /cmd_vel → Gazebo Robot

Line Detector → PID → /
                     → Command Mux → Robot
Line Detector → LQR → \
```

---

## Roadmap

- Camera-based line detection  
- MPC controller  
- RViz visualization  
- Logging and plotting tools  

---

## License

MIT License.

---

## Author

Pavan Sai Eshwar Chandra  
https://github.com/pavansai018
