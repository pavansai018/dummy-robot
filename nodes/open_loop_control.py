#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

class TeleopButtons(Node):
    def __init__(self):
        super().__init__('teleop_buttons')
        
        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create GUI
        self.create_gui()
        
        self.get_logger().info('Teleop Buttons GUI started')

    def create_gui(self):
        """Create the button GUI"""
        self.root = tk.Tk()
        self.root.title("Robot Teleoperation")
        self.root.geometry("300x400")
        self.root.resizable(False, False)
        
        # Configure style
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 12), padding=10)
        
        # Create buttons in the specified layout
        # Forward button (top)
        self.forward_btn = ttk.Button(
            self.root, 
            text="FORWARD", 
            command=self.move_forward
        )
        self.forward_btn.place(relx=0.5, rely=0.2, anchor='center', width=100, height=60)
        
        # Stop button (center)
        self.stop_btn = ttk.Button(
            self.root, 
            text="STOP", 
            command=self.stop,
            style='Emergency.TButton'
        )
        style.configure('Emergency.TButton', background='red', foreground='white')
        self.stop_btn.place(relx=0.5, rely=0.5, anchor='center', width=100, height=60)
        
        # Backward button (bottom)
        self.backward_btn = ttk.Button(
            self.root, 
            text="BACKWARD", 
            command=self.move_backward
        )
        self.backward_btn.place(relx=0.5, rely=0.8, anchor='center', width=100, height=60)
        
        # Left button (left of stop)
        self.left_btn = ttk.Button(
            self.root, 
            text="LEFT", 
            command=self.turn_left
        )
        self.left_btn.place(relx=0.2, rely=0.5, anchor='center', width=80, height=60)
        
        # Right button (right of stop)
        self.right_btn = ttk.Button(
            self.root, 
            text="RIGHT", 
            command=self.turn_right
        )
        self.right_btn.place(relx=0.8, rely=0.5, anchor='center', width=80, height=60)
        
        # Status label
        self.status_label = ttk.Label(
            self.root, 
            text="Click buttons to control robot", 
            font=('Arial', 10)
        )
        self.status_label.place(relx=0.5, rely=0.95, anchor='center')
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        self.root.focus_set()
        
        # Current state
        self.current_cmd = Twist()
        self.key_pressed = False

    def move_forward(self):
        """Move robot forward"""
        self.current_cmd.linear.x = 0.5
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving FORWARD")

    def move_backward(self):
        """Move robot backward"""
        self.current_cmd.linear.x = -0.3
        self.current_cmd.angular.z = 0.0
        self.publish_cmd()
        self.update_status("Moving BACKWARD")

    def turn_left(self):
        """Turn robot left (in place)"""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.5
        self.publish_cmd()
        self.update_status("Turning LEFT")

    def turn_right(self):
        """Turn robot right (in place)"""
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -0.5
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