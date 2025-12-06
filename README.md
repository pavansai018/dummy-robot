# dummy-robot

A 4-wheeled robot simulation built using ROS 2 Jazzy and Gazebo Harmonic.  
Includes PID and LQR controllers for **camera-based line following** and a GUI tool for manual teleoperation.

---

## Features

- Differential-drive robot with clean URDF model  
- Gazebo Harmonic simulation worlds (square and circular tracks)  
- **Camera-based black-line tracking**  
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

This loads:

- Gazebo Harmonic  
- Robot model  
- The selected world (square or circular black line)  

---

## Teleoperation

```
ros2 run dummy_robot teleop_buttons
```

Manual GUI for testing robot motion.

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

Uses camera-based line error + optimal state-feedback control.

---

## Roadmap

- Improve camera thresholding robustness  
- Add support for dynamic lighting changes  
- Add RViz visualization  
- Add logging and plotting tools  

---

## License

MIT License.

---

## Author

Pavan Sai Eshwar Chandra  
https://github.com/pavansai018
