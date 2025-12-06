dummy-robot

A clean, modular 4-wheeled robot simulation framework built on ROS 2 Jazzy and Gazebo Harmonic, featuring PID and LQR line-following controllers and a GUI teleoperation tool.
Engineered for experimentation, benchmarking, and robotics portfolio demonstration.

📘 Overview

dummy-robot provides:

A differential-drive robot model (URDF + meshes)

Ready-to-use Gazebo Harmonic worlds (square & circular tracks)

GUI teleop for manual control

PID and LQR controllers for black-line following

Clean ROS 2 node architecture

Extendable structure for research and teaching

This repository follows a professional robotics project layout, suitable for real engineering workflows.

🏁 Launching the Simulation

Start the robot in Gazebo:

ros2 launch dummy_robot <launch_file>.launch.py


Each launch file loads:

Gazebo Harmonic

The differential-drive robot

A square / circular line-track world

🎮 GUI Teleoperation

Interactive control interface:

ros2 run dummy_robot teleop_buttons


Used for:

Manual driving

Motion testing

Debugging / calibration

⚫ Line-Following Controllers
PID Controller

Start PID tracking:

ros2 run dummy_robot line_follower_pid


GUI-based PID:

ros2 run dummy_robot line_follower_pid_gui


Features:

Cross-track error calculation

Proportional + derivative stabilization

Tunable velocity

LQR Controller

Start optimal control:

ros2 run dummy_robot line_follower_lqr


Uses:

State vector: lateral offset, heading error, derivatives

Cost function: xᵀ Q x + uᵀ R u

Minimizes tracking error and control effort

Produces smoother, stable paths at higher speeds


📊 Comparing PID vs LQR
Property	PID	LQR
Stability	Moderate	High
Smoothness	Medium	Excellent
Tuning	Manual (Kp, Ki, Kd)	Systematic (Q/R selection)
High-speed performance	Limited	Strong
Optimal control	❌	✅