```markdown
# dummy-robot

A complete simulation of a 4-wheeled robot built using **ROS 2 Jazzy** and **Gazebo Harmonic**, with **GUI control**, **PID** and **LQR** line-following, and ready-to-run launch files.

![Robot Simulation](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExbDQzbnNyazQ5Z2R4MTFya3ppc2J4a293NDNlcG83d3J2Mnd1M25uayZlcD12MV9naWZzX3NlYXJjaCZjdD1n/e9InzRa6woYxxF17dw/giphy.gif)

---

## 🌟 Features

- 4-wheeled differential drive robot  
- URDF and meshes included  
- GUI-based teleoperation  
- Black line following on square and circle tracks  
- PID controller for line following  
- LQR controller for optimal line following  
- Gazebo Harmonic simulation integration  
- Clean ROS 2 package layout  

---

## 📦 Package Overview

**Package name:** `dummy_robot`

### Console Scripts

| Command                 | Description                    |
|-------------------------|--------------------------------|
| `teleop_buttons`        | GUI based teleoperation        |
| `line_follower_pid`     | Pure PID line follower         |
| `line_follower_pid_gui` | GUI-based PID line follower    |
| `line_follower_lqr`     | LQR line follower with GUI     |

Run any of them with:

```bash
ros2 run dummy_robot <script_name>
```

Examples:

```bash
ros2 run dummy_robot teleop_buttons
ros2 run dummy_robot line_follower_pid
ros2 run dummy_robot line_follower_pid_gui
ros2 run dummy_robot line_follower_lqr
```

---

## 📁 Project Structure

```text
dummy-robot/
├── dummy_robot/
├── launch/
├── meshes/
├── nodes/
├── urdf/
├── worlds/
├── package.xml
├── setup.py
├── setup.cfg
└── LICENSE
```

---

## ⚙️ Requirements

- ROS 2 Jazzy  
- Gazebo Harmonic  
- Python 3.x  
- Colcon-based ROS 2 workspace  

---

## 🚀 Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/pavansai018/dummy-robot.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 🏁 Launching the Simulation

```bash
ros2 launch dummy_robot <launch_file>.launch.py
```

This will:

- Start Gazebo Harmonic  
- Spawn the 4-wheeled dummy robot  
- Load a world (square or circular black path) depending on the launch file  

![Line following](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExZzF2cGxnNHBodDdrcG1pbXh6dzd2N2M2NjZpb2YyZzE5cjJ5aTZyMCZlcD12MV9naWZzX3NlYXJjaCZjdD1n/EK24qVhF2WOobS7zJc/giphy.gif)

---

## 🕹 Teleoperation GUI

```bash
ros2 run dummy_robot teleop_buttons
```

This opens a GUI with directional control buttons:

- Forward  
- Backward  
- Left  
- Right  
- Stop  

Useful for testing robot motion before running controllers.

---

## ⚫ Line Following

The robot follows a **black line** path on the floor of the Gazebo world.

### PID Line Follower

Non-GUI version:

```bash
ros2 run dummy_robot line_follower_pid
```

GUI version:

```bash
ros2 run dummy_robot line_follower_pid_gui
```

PID computes:

- Cross-track error  
- Derivative error  
- Steering and velocity commands  

You can tune:

- Kp  
- Ki  
- Kd  

---

### LQR Line Follower

Run:

```bash
ros2 run dummy_robot line_follower_lqr
```

LQR:

- Uses a state-feedback model  
- Minimizes quadratic cost: J = xᵀQx + uᵀRu  
- Generates optimal control `u = -Kx`  

Tune:

- Q → penalizes state deviation  
- R → penalizes control effort  

---

## 📊 Experiments and Ideas

- Compare PID vs LQR tracking performance  
- Log error vs time, speed, control output  
- Try circle vs square worlds  
- Add noise or friction changes  
- Test higher speed regimes  
- Create plots and include them in this README  

---

## 🗺 Roadmap

- Add RViz2 visualization  
- Add camera-based line detection  
- Add MPC controller  
- Add dynamic obstacles  
- Add automated plotting utilities  

---

## 📜 License

MIT License  
See `LICENSE` file.

---

## 👤 Author

**Pavan Sai Eshwar Chandra**  
GitHub: https://github.com/pavansai018

---

## 📸 Demo Screenshots (Placeholders)

![Robot icon](https://upload.wikimedia.org/wikipedia/commons/thumb/0/0b/Robot-icon.svg/480px-Robot-icon.svg.png)

![Gazebo logo](https://upload.wikimedia.org/wikipedia/commons/thumb/0/03/Gazebo_logo.svg/320px-Gazebo_logo.svg.png)
```
