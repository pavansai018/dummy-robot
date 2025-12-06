🚀 dummy-robot

4-Wheeled Robot Simulation • ROS 2 Jazzy • Gazebo Harmonic • PID & LQR Line Following • GUI Control

A clean, modern simulation framework for experimenting with robot control, optimal feedback (LQR), and classical PID tracking, built on ROS 2 + Gazebo Harmonic.
Includes a plug-and-play GUI teleop, line-following pipelines, and an easily extendable URDF robot.

✨ Key Features
🛞 Robot Platform

4-wheeled differential drive robot

URDF + high-quality meshes

Tunable wheelbase, radius & sensor configs

🎮 GUI Teleoperation

Button-based interface for manual driving

Useful for demos, debugging, or testing controllers

⚫ Line Following

Supports two path shapes:

Square track

Circular track

Control algorithms:

PID Controller (simple & effective)

LQR Controller (optimal for smooth tracking)

🏗️ Built on Modern ROS 2 Tools

ROS 2 Jazzy middleware

Gazebo Harmonic physics simulation

Colcon workspace support

🧩 System Architecture (High-Level)
+------------------------------+
|        User / GUI           |
+---------------+--------------+
                |
                v
+---------------+--------------+
|     Control Layer            |
|  PID Node     |   LQR Node   |
+-------+--------+------+------+
        |               |
        v               v
+-------+---------------+------+
|      Command Mux / Topic     |
+---------------+--------------+
                |
                v
+---------------+--------------+
|    Gazebo Robot Model        |
|  URDF + Joints + Sensors     |
+---------------+--------------+

🗂 Project Structure
dummy-robot/
├── dummy_robot/        # Python package modules
├── nodes/              # PID, LQR, GUI controllers
├── urdf/               # Robot URDF files
├── meshes/             # STL/DAE models
├── worlds/             # Square & circular line worlds
├── launch/             # ROS 2 launch files
├── package.xml
├── setup.py
└── LICENSE

🚀 Installation

cd ~/ros2_ws/src
git clone https://github.com/pavansai018/dummy-robot.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash


▶️ Launch Simulation

Start the robot in Gazebo:

ros2 launch dummy_robot <launch_file>.launch.py


This will automatically:

Load Gazebo Harmonic

Spawn the 4-wheeled robot

Apply correct controllers

Load world (square / circle path)

🎮 Teleoperation GUI
ros2 run dummy_robot teleop_buttons


A clean control window appears with:

Forward

Left

Right

Reverse

Stop

Perfect for manual driving and testing topics.

⚫ Line Following
PID Mode
ros2 run dummy_robot line_follower_pid


PID uses:

Cross-track error

Derivative correction

Tunable velocity

GUI version:

ros2 run dummy_robot line_follower_pid_gui

LQR Mode
ros2 run dummy_robot line_follower_lqr


LQR provides:

Smoother trajectories

Minimal control effort

Higher stability at high speeds

Internally uses:

State vector: [e_y, e_θ, ė_y, ė_θ]

Cost function: xᵀ Q x + uᵀ R u

📸 Demo Placeholders

(Replace with your own screenshots later)

Line Following

Robot Model


📜 License

Licensed under the MIT License.