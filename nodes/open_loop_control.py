#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from cv_bridge import CvBridge
import cv2
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
from sensor_msgs.msg import Image

class TeleopButtons(Node):
    def __init__(self):
        super().__init__('teleop_buttons')
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocities (adjustable via sliders)
        self.linear_velocity = 1.0
        self.angular_velocity = 1.0
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera',   
            self.image_callback,
            10
        )

        # Camera-related attributes
        self.camera_label = None
        # must keep a reference or Tk will garbage-collect
        self.camera_photo = None  
        self.camera_width = 700
        self.camera_height = 260
        # Create GUI
        self.create_gui()
        
        self.get_logger().info('Teleop Buttons GUI started')

    def create_gui(self):
        """Create the button GUI with sliders"""
        self.root = tk.Tk()
        self.root.title("Dummy Robot Teleoperation")
        self.root.geometry("800x700")
        self.root.resizable(True, True)
        
        # Configure style
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 12), padding=10)
        
        # Velocity Control Frame
        control_frame = ttk.LabelFrame(self.root, text="Velocity Control", padding=10)
        control_frame.place(relx=0.5, rely=0.05, anchor='n', width=500, height=120)
        
        # Linear Velocity Slider
        ttk.Label(control_frame, text="Linear Velocity:").grid(row=0, column=0, sticky='w', padx=5)
        self.linear_slider = ttk.Scale(control_frame, from_=0.1, to=5.0, orient='horizontal', 
                                      command=self.update_linear_velocity)
        self.linear_slider.set(self.linear_velocity)
        self.linear_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.linear_label = ttk.Label(control_frame, text=f"{self.linear_velocity:.1f} m/s")
        self.linear_label.grid(row=0, column=2, padx=5)
        
        # Angular Velocity Slider
        ttk.Label(control_frame, text="Angular Velocity:").grid(row=1, column=0, sticky='w', padx=5)
        self.angular_slider = ttk.Scale(control_frame, from_=0.1, to=4.0, orient='horizontal', 
                                       command=self.update_angular_velocity)
        self.angular_slider.set(self.angular_velocity)
        self.angular_slider.grid(row=1, column=1, sticky='ew', padx=5)
        self.angular_label = ttk.Label(control_frame, text=f"{self.angular_velocity:.1f} rad/s")
        self.angular_label.grid(row=1, column=2, padx=5)
        
        control_frame.columnconfigure(1, weight=1)
        
        # Control Buttons Frame
        button_frame = ttk.Frame(self.root)
        button_frame.place(relx=0.5, rely=0.2, anchor='n', width=500, height=300)
        
        # Create buttons in the specified layout
        # Forward button (top)
        self.forward_btn = ttk.Button(
            button_frame, 
            text="FORWARD", 
            command=self.move_forward
        )
        self.forward_btn.place(relx=0.5, rely=0.2, anchor='center', width=150, height=60)
        
        # Stop button (center)
        self.stop_btn = ttk.Button(
            button_frame, 
            text="STOP", 
            command=self.stop,
            style='Emergency.TButton'
        )
        style.configure('Emergency.TButton', background='red', foreground='white')
        self.stop_btn.place(relx=0.5, rely=0.5, anchor='center', width=150, height=60)
        
        # Backward button (bottom)
        self.backward_btn = ttk.Button(
            button_frame, 
            text="BACKWARD", 
            command=self.move_backward
        )
        self.backward_btn.place(relx=0.5, rely=0.8, anchor='center', width=150, height=60)
        
        # Left button (left of stop)
        self.left_btn = ttk.Button(
            button_frame, 
            text="LEFT", 
            command=self.turn_left
        )
        self.left_btn.place(relx=0.2, rely=0.5, anchor='center', width=100, height=60)
        
        # Right button (right of stop)
        self.right_btn = ttk.Button(
            button_frame, 
            text="RIGHT", 
            command=self.turn_right
        )
        self.right_btn.place(relx=0.8, rely=0.5, anchor='center', width=100, height=60)
        
        # Status label
        self.status_label = ttk.Label(
            self.root, 
            text="Click buttons to control robot", 
            font=('Arial', 10)
        )
        self.status_label.place(relx=0.5, rely=0.95, anchor='center')
        # Camera frame
        camera_frame = ttk.LabelFrame(self.root, text="Camera View", padding=5)
        camera_frame.place(relx=0.5, rely=0.62, anchor='n', width=700, height=260)

        self.camera_label = ttk.Label(camera_frame, text="Waiting for image...")
        self.camera_label.pack(expand=True, fill='both')
        # Current velocities display
        self.velocity_display = ttk.Label(
            self.root,
            text=f"Current: Linear={self.linear_velocity:.1f} m/s, Angular={self.angular_velocity:.1f} rad/s",
            font=('Arial', 9)
        )
        self.velocity_display.place(relx=0.5, rely=0.9, anchor='center')
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        self.root.focus_set()
        
        # Current state
        self.current_cmd = Twist()
        self.key_pressed = False

        # Start periodic ROS spinning so subscription callbacks work
        self.root.after(10, self.ros_spin_once)
    
    def ros_spin_once(self):
        """Pump ROS2 events periodically from Tkinter loop."""
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as e:
            self.get_logger().error(f"Error in spin_once: {e}")
        # schedule next call
        self.root.after(10, self.ros_spin_once)


    def image_callback(self, msg: Image):
        """ROS2 camera callback: convert and display in Tkinter."""
        try:
            # ROS Image -> OpenCV BGR array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w, _ = cv_image.shape

        # Resize for GUI
        cv_image = cv2.resize(cv_image, (self.camera_width, self.camera_height))

        # BGR -> RGB
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convert to PIL image, then to ImageTk
        pil_img = PILImage.fromarray(cv_image)
        self.camera_photo = ImageTk.PhotoImage(image=pil_img)

        # Update Tkinter label
        if self.camera_label is not None:
            self.camera_label.configure(image=self.camera_photo, text="")


    def update_linear_velocity(self, value):
        """Update linear velocity from slider"""
        self.linear_velocity = float(value)
        self.linear_label.config(text=f"{self.linear_velocity:.1f} m/s")
        self.update_velocity_display()

    def update_angular_velocity(self, value):
        """Update angular velocity from slider"""
        self.angular_velocity = float(value)
        self.angular_label.config(text=f"{self.angular_velocity:.1f} rad/s")
        self.update_velocity_display()

    def update_velocity_display(self):
        """Update the current velocities display"""
        self.velocity_display.config(
            text=f"Current: Linear={self.linear_velocity:.1f} m/s, Angular={self.angular_velocity:.1f} rad/s"
        )

    def move_forward(self):
        """Move robot forward"""
        self.current_cmd.linear.x = self.linear_velocity
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving FORWARD")

    def move_backward(self):
        """Move robot backward"""
        self.current_cmd.linear.x = -self.linear_velocity
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving BACKWARD")

    def turn_left(self):
        """Turn robot left (in place)"""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = self.angular_velocity
        self.publish_cmd()
        self.update_status("Turning LEFT")

    def turn_right(self):
        """Turn robot right (in place)"""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -self.angular_velocity
        self.publish_cmd()
        self.update_status("Turning RIGHT")

    def stop(self):
        """Stop robot"""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("STOPPED")

    def publish_cmd(self):
        """Publish the current command to /cmd_vel"""
        self.cmd_pub.publish(self.current_cmd)
        self.get_logger().info(f'Publishing: linear.x={self.current_cmd.linear.x:.2f}, angular.z={self.current_cmd.angular.z:.2f}')

    def update_status(self, status):
        """Update the status label"""
        self.status_label.config(text=f"Status: {status}")

    def key_press(self, event):
        """Handle keyboard press events"""
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
        """Handle keyboard release events"""
        self.key_pressed = False
        if event.keysym in ['Up', 'Down', 'Left', 'Right']:
            self.stop()

    def run(self):
        """Start the GUI main loop"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            # Stop robot when closing
            self.stop()
            self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopButtons()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()