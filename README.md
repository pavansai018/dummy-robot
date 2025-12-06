```markdown
<h1 align="center">dummy-robot</h1>

<p align="center">
  ROS 2 Jazzy · Gazebo Harmonic · PID & LQR Controllers · Line Following Simulation
</p>

<p align="center">
  A compact 4-wheeled robot simulation with modular controllers and GUI teleoperation.
</p>

---

## Features

- Differential-drive 4-wheel robot  
- URDF robot model  
- Gazebo Harmonic simulation (square / circle tracks)  
- PID and LQR line-following controllers  
- GUI teleoperation for manual control  
- Clean ROS 2 package layout  

---

## Project Structure

```
dummy-robot/
├── dummy_robot/        # Python package
├── nodes/              # PID, LQR, GUI nodes
├── urdf/               # Robot description
├── meshes/             # Geometry
├── worlds/             # Gazebo worlds
├── launch/             # Launch files
├── package.xml
└── setup.py
```

---

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/pavansai018/dummy-robot.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Simulation

```bash
ros2 launch dummy_robot <launch_file>.launch.py
```

Loads:
- Gazebo Harmonic  
- Robot model  
- World (square or circular track)

---

## Teleoperation

```bash
ros2 run dummy_robot teleop_buttons
```

Manual GUI driving tool.

---

## Line Following

### PID Controller

```bash
ros2 run dummy_robot line_follower_pid
```

GUI version:

```bash
ros2 run dummy_robot line_follower_pid_gui
```

### LQR Controller

```bash
ros2 run dummy_robot line_follower_lqr
```

State-feedback optimal control using Q/R matrices.

---

## System Overview

```
GUI Teleop     →  /cmd_vel  → Gazebo Robot

Line Detector → PID  → /
                      → Command Mux → Robot
Line Detector → LQR  → \
```

---

## Roadmap

- Add camera-based line detection  
- Add MPC controller  
- Add RViz2 visualization  
- Add logging & plotting utilities  

---

## License

MIT License

---

## Author

Pavan Sai Eshwar Chandra  
https://github.com/pavansai018
```
