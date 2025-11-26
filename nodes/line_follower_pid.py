#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pdb
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Parameters (so you can tune via launch/CLI)
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('kp', 0.0025)
        self.declare_parameter('forward_speed', 1.00)
        self.declare_parameter('angular_speed', 0.3)
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.bridge = CvBridge()

        # Subscribers & publishers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.twist = Twist()

        self.get_logger().info(f"LineFollowerNode started. Subscribing to {camera_topic}, publishing to {cmd_vel_topic}")

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, w, _ = cv_image.shape

        # 1. Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 2. Threshold for BLACK line
        #    Any pixel below 'thresh' is considered dark -> becomes white in mask (due to THRESH_BINARY_INV)
        thresh = 127  # you may tune this
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)

        # 3. Focus on bottom region of interest (local area near the robot)
        search_top = int(0.6 * h)   # start at 60% height
        search_bot = int(0.9 * h)   # end at 90% height

        roi = mask[search_top:search_bot, :]

        # 4. Compute centroid of the white region (the black line in original image)
        M = cv2.moments(roi)
        # pdb.set_trace()
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            center_x = w // 2

            # Cross-track error in pixel coordinates
            error = cx - center_x  # +ve if line is to the right

            # 5. P controller: angular velocity
            self.twist.linear.x = float(self.forward_speed)
            self.twist.angular.z = float(-self.kp * error)

            self.cmd_pub.publish(self.twist)

            # Optional debug print
            # self.get_logger().info(f"cx: {cx}, error: {error}, ang_z: {self.twist.angular.z:.3f}")
        else:
            # No line found in ROI – fallback behaviour.
            # E.g. stop and slowly rotate to search.
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
            self.cmd_pub.publish(self.twist)
            # self.get_logger().warn("No line detected in ROI; rotating to search.")


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
