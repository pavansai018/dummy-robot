#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import ttk

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from PIL import Image as PILImage, ImageTk


class TeleopLineFollower(Node):
    def __init__(self):
        super().__init__('teleop_line_follower')

        # --- ROS pubs/subs ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera',        # adjust if your camera topic is different
            self.image_callback,
            10
        )

        # --- State: teleop + PID + camera ---
        # Teleop velocities
        self.linear_velocity = 2.0   # also used as line-following forward speed
        self.angular_velocity = 2.0  # for manual turning

        # PID gains (line follower)
        self.kp = 0.0050
        self.ki = 0.000091
        self.kd = 0.0050

        # PID state
        self.error_integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # Line following toggle (BooleanVar will be created after Tk root)
        self.line_follow_enabled = None

        # Camera-related
        self.camera_label = None
        self.camera_photo = None  # must hold reference
        self.camera_width = 700
        self.camera_height = 260

        # Current cmd & keyboard state
        self.current_cmd = Twist()
        self.key_pressed = False

        # Build GUI
        self.create_gui()

        self.get_logger().info('Teleop + PID Line Follower GUI started')

    # ----------------------------------------------------------------------
    # GUI setup
    # ----------------------------------------------------------------------
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Dummy Robot Teleop + PID Line Follower")
        self.root.geometry("900x750")
        self.root.resizable(True, True)

        style = ttk.Style()
        style.configure('TButton', font=('Arial', 12), padding=10)

        # ----------------- Velocity Control ----------------- #
        control_frame = ttk.LabelFrame(self.root, text="Velocity Control", padding=10)
        control_frame.place(relx=0.5, rely=0.03, anchor='n', width=820, height=120)

        # Linear Velocity Slider (used by both teleop and line follower)
        ttk.Label(control_frame, text="Linear Velocity (m/s):").grid(row=0, column=0, sticky='w', padx=5)
        self.linear_slider = ttk.Scale(
            control_frame, from_=0.01, to=5.0, orient='horizontal',
            command=self.update_linear_velocity
        )
        self.linear_slider.set(self.linear_velocity)
        self.linear_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.linear_label = ttk.Label(control_frame, text=f"{self.linear_velocity:.2f} m/s")
        self.linear_label.grid(row=0, column=2, padx=5)

        # Angular Velocity Slider (for teleop turning)
        ttk.Label(control_frame, text="Manual Angular Vel (rad/s):").grid(row=1, column=0, sticky='w', padx=5)
        self.angular_slider = ttk.Scale(
            control_frame, from_=0.1, to=4.0, orient='horizontal',
            command=self.update_angular_velocity
        )
        self.angular_slider.set(self.angular_velocity)
        self.angular_slider.grid(row=1, column=1, sticky='ew', padx=5)
        self.angular_label = ttk.Label(control_frame, text=f"{self.angular_velocity:.2f} rad/s")
        self.angular_label.grid(row=1, column=2, padx=5)

        control_frame.columnconfigure(1, weight=1)

        # ----------------- PID Control Sliders ----------------- #
        pid_frame = ttk.LabelFrame(self.root, text="PID Gains (line follower)", padding=10)
        pid_frame.place(relx=0.5, rely=0.20, anchor='n', width=820, height=140)

        # PID enable variable (now root exists)
        self.line_follow_enabled = tk.BooleanVar(self.root, value=False)

        # Kp
        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky='w', padx=5)
        self.kp_slider = ttk.Scale(
            pid_frame, from_=0.0005, to=0.02, orient='horizontal',
            command=self.update_kp
        )
        self.kp_slider.set(self.kp)
        self.kp_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.kp_label = ttk.Label(pid_frame, text=f"{self.kp:.4f}")
        self.kp_label.grid(row=0, column=2, padx=5)

        # Ki
        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=0, sticky='w', padx=5)
        self.ki_slider = ttk.Scale(
            pid_frame, from_=0.0, to=0.001, orient='horizontal',
            command=self.update_ki
        )
        self.ki_slider.set(self.ki)
        self.ki_slider.grid(row=1, column=1, sticky='ew', padx=5)
        self.ki_label = ttk.Label(pid_frame, text=f"{self.ki:.6f}")
        self.ki_label.grid(row=1, column=2, padx=5)

        # Kd
        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0, sticky='w', padx=5)
        self.kd_slider = ttk.Scale(
            pid_frame, from_=0.0, to=0.02, orient='horizontal',
            command=self.update_kd
        )
        self.kd_slider.set(self.kd)
        self.kd_slider.grid(row=2, column=1, sticky='ew', padx=5)
        self.kd_label = ttk.Label(pid_frame, text=f"{self.kd:.4f}")
        self.kd_label.grid(row=2, column=2, padx=5)

        # Line following enable/disable
        self.line_follow_check = ttk.Checkbutton(
            pid_frame,
            text="Enable Line Following",
            variable=self.line_follow_enabled
        )
        self.line_follow_check.grid(row=0, column=3, rowspan=2, padx=15)

        pid_frame.columnconfigure(1, weight=1)
        # self.line_follow_check.set(0) # default off
        # ----------------- Teleop Buttons ----------------- #
        button_frame = ttk.Frame(self.root)
        button_frame.place(relx=0.5, rely=0.40, anchor='n', width=500, height=200)

        # Forward
        self.forward_btn = ttk.Button(button_frame, text="FORWARD", command=self.move_forward)
        self.forward_btn.place(relx=0.5, rely=0.2, anchor='center', width=150, height=60)

        # Stop
        style.configure('Emergency.TButton', background='red', foreground='white')
        self.stop_btn = ttk.Button(button_frame, text="STOP", command=self.stop, style='Emergency.TButton')
        self.stop_btn.place(relx=0.5, rely=0.5, anchor='center', width=150, height=60)

        # Backward
        self.backward_btn = ttk.Button(button_frame, text="BACKWARD", command=self.move_backward)
        self.backward_btn.place(relx=0.5, rely=0.8, anchor='center', width=150, height=60)

        # Left
        self.left_btn = ttk.Button(button_frame, text="LEFT", command=self.turn_left)
        self.left_btn.place(relx=0.2, rely=0.5, anchor='center', width=100, height=60)

        # Right
        self.right_btn = ttk.Button(button_frame, text="RIGHT", command=self.turn_right)
        self.right_btn.place(relx=0.8, rely=0.5, anchor='center', width=100, height=60)

        # ----------------- Camera View ----------------- #
        camera_frame = ttk.LabelFrame(self.root, text="Camera View", padding=5)
        camera_frame.place(relx=0.5, rely=0.63, anchor='n', width=820, height=260)

        self.camera_label = ttk.Label(camera_frame, text="Waiting for image...")
        self.camera_label.pack(expand=True, fill='both')

        # ----------------- Status & Velocity Display ----------------- #
        self.velocity_display = ttk.Label(
            self.root,
            text=f"Current: Linear={self.linear_velocity:.2f} m/s, Angular={self.angular_velocity:.2f} rad/s",
            font=('Arial', 9)
        )
        self.velocity_display.place(relx=0.5, rely=0.93, anchor='center')

        self.status_label = ttk.Label(
            self.root,
            text="Arrow keys + space for teleop. PID sliders for line following.",
            font=('Arial', 10)
        )
        self.status_label.place(relx=0.5, rely=0.97, anchor='center')

        # Keyboard bindings
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        self.root.focus_set()

        # Start periodic ROS spinning
        self.root.after(10, self.ros_spin_once)

    # ----------------------------------------------------------------------
    # ROS <-> Tk integration
    # ----------------------------------------------------------------------
    def ros_spin_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as e:
            self.get_logger().error(f"Error in spin_once: {e}")
        self.root.after(10, self.ros_spin_once)

    # ----------------------------------------------------------------------
    # Camera callback
    # ----------------------------------------------------------------------
    def image_callback(self, msg: Image):
        """ROS2 camera callback: convert and display; also run line-follow PID if enabled."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # --- Line following section (PID) ---
        if self.line_follow_enabled is not None and self.line_follow_enabled.get():
            self.run_line_follower(cv_image)
        elif self.line_follow_enabled is None or not self.line_follow_enabled.get():
            # If line following disabled, stop robot
            self.current_cmd.linear.x = 0.0
            self.current_cmd.angular.z = 0.0
            self.cmd_pub.publish(self.current_cmd)

        # --- GUI display (scaled image) ---
        cv_resized = cv2.resize(cv_image, (self.camera_width, self.camera_height))
        cv_resized = cv2.cvtColor(cv_resized, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_resized)
        self.camera_photo = ImageTk.PhotoImage(image=pil_img)

        if self.camera_label is not None:
            self.camera_label.configure(image=self.camera_photo, text="")

    # ----------------------------------------------------------------------
    # Line follower PID logic (uses current kp, ki, kd, linear_velocity)
    # ----------------------------------------------------------------------
    def run_line_follower(self, cv_image):
        h, w, _ = cv_image.shape

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = 127  # tune if needed
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)

        search_top = int(0.6 * h)
        search_bot = int(0.9 * h)
        roi = mask[search_top:search_bot, :]

        M = cv2.moments(roi)

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = now

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            center_x = w // 2

            error = cx - center_x  # +ve if line is to the right

            # PID terms
            self.error_integral += error * dt
            # anti-windup clamp
            max_int = 10000.0
            self.error_integral = max(min(self.error_integral, max_int), -max_int)

            d_error = (error - self.prev_error) / dt
            self.prev_error = error

            omega = -(float(self.kp_slider.get()) * error + float(self.ki_slider.get()) * self.error_integral + float(self.kd_slider.get()) * d_error)

            # Optional: clamp angular velocity
            max_w = 2.0  # rad/s, tune as needed
            omega = max(-max_w, min(max_w, omega))

            self.current_cmd.linear.x = float(self.linear_slider.get())
            self.current_cmd.angular.z = float(omega)
            self.cmd_pub.publish(self.current_cmd)

        else:
            # Line lost: simple search behaviour
            self.current_cmd.linear.x = 0.0
            self.current_cmd.angular.z = float(self.angular_slider.get()) #0.3
            self.cmd_pub.publish(self.current_cmd)

    # ----------------------------------------------------------------------
    # Slider callbacks
    # ----------------------------------------------------------------------
    def update_linear_velocity(self, value):
        self.linear_velocity = float(value)
        self.linear_label.config(text=f"{self.linear_velocity:.2f} m/s")
        self.update_velocity_display()

    def update_angular_velocity(self, value):
        self.angular_velocity = float(value)
        self.angular_label.config(text=f"{self.angular_velocity:.2f} rad/s")
        self.update_velocity_display()

    def update_kp(self, value):
        self.kp = float(value)
        self.kp_label.config(text=f"{self.kp:.4f}")

    def update_ki(self, value):
        self.ki = float(value)
        self.ki_label.config(text=f"{self.ki:.6f}")

    def update_kd(self, value):
        self.kd = float(value)
        self.kd_label.config(text=f"{self.kd:.4f}")

    def update_velocity_display(self):
        self.velocity_display.config(
            text=f"Current: Linear={self.linear_velocity:.2f} m/s, Angular={self.angular_velocity:.2f} rad/s"
        )

    # ----------------------------------------------------------------------
    # Teleop button actions
    # ----------------------------------------------------------------------
    def move_forward(self):
        self.current_cmd.linear.x = self.linear_velocity
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving FORWARD (manual)")

    def move_backward(self):
        self.current_cmd.linear.x = -self.linear_velocity
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving BACKWARD (manual)")

    def turn_left(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = self.angular_velocity
        self.publish_cmd()
        self.update_status("Turning LEFT (manual)")

    def turn_right(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -self.angular_velocity
        self.publish_cmd()
        self.update_status("Turning RIGHT (manual)")

    def stop(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("STOPPED")

    def publish_cmd(self):
        self.cmd_pub.publish(self.current_cmd)
        self.get_logger().info(
            f'Publishing: linear.x={self.current_cmd.linear.x:.2f}, angular.z={self.current_cmd.angular.z:.2f}'
        )

    def update_status(self, status):
        self.status_label.config(text=f"Status: {status}")

    # ----------------------------------------------------------------------
    # Keyboard handling
    # ----------------------------------------------------------------------
    def key_press(self, event):
        if self.key_pressed:
            return

        self.key_pressed = True
        if event.keysym == 'Up':
            self.move_forward()
        elif event.keysym == 'Down':
            self.move_backward()
        elif event.keysym == 'Left':
            self.turn_left()
        elif event.keysym == 'Right':
            self.turn_right()
        elif event.keysym == 'space':
            self.stop()

    def key_release(self, event):
        self.key_pressed = False
        if event.keysym in ['Up', 'Down', 'Left', 'Right']:
            self.stop()

    # ----------------------------------------------------------------------
    # Mainloop
    # ----------------------------------------------------------------------
    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopLineFollower()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
