#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class MarkerInfoSubscriber(Node):
    def __init__(self):
        super().__init__('marker_info_subscriber')

        # Create a subscriber to receive marker info
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'marker_info',
            self.marker_info_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for Twist commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define the regions in the camera frame
        self.frame_width = 1280
        self.region_left = int(self.frame_width // 3)
        self.region_right = 2 * int(self.frame_width // 3)

        # Define custom rotation velocities for each region
        self.rotation_speed_left = 0.3
        self.rotation_speed_right = -0.3
        self.rotation_speed_center = 0.0

        # Define the area threshold for stopping forward motion
        self.area_threshold = 35000

    def marker_info_callback(self, msg):
        # Check if marker information is empty
        if not msg.data:
            # No marker detected, publish stop command
            twist_cmd = Twist()
            self.cmd_vel_pub.publish(twist_cmd)
            return

        # Convert received data back to numpy array
        marker_info_np = np.array(msg.data).reshape(-1, 12)

        # Unpack the 12 values into separate variables
        for marker_id, tl_x, tr_x, bl_x, br_x, tl_y, tr_y, bl_y, br_y, center_x, center_y, area in marker_info_np:
            # Determine the region where the marker center lies
            if center_x < self.region_left:
                # Marker is in the left region
                rotation_cmd = self.rotation_speed_left
            elif center_x > self.region_right:
                # Marker is in the right region
                rotation_cmd = self.rotation_speed_right
            else:
                # Marker is in the center region
                rotation_cmd = self.rotation_speed_center

            # Calculate area error
            area_error = self.area_threshold - area

            # Check if area error is less than 50 sq. px
            # if area_error < 50:
            #     # Publish stop command
            #     twist_cmd = Twist()
            #     self.cmd_vel_pub.publish(twist_cmd)
            #     self.get_logger().info("Area error below threshold. Stopping.")
            #     return

            # Create Twist message
            twist_cmd = Twist()

            # Set angular velocity in z direction for rotation
            twist_cmd.angular.z = rotation_cmd

            # Set linear velocity in x direction for forward motion
            twist_cmd.linear.x = float(area_error) * 0.00002  # You can adjust the scaling factor as needed

            # Publish Twist command
            self.cmd_vel_pub.publish(twist_cmd)

            # Log the rotational and area errors
            self.get_logger().info(f"Rotational Command: {rotation_cmd}, Area Error: {area_error}")


def main(args=None):
    rclpy.init(args=args)
    marker_info_subscriber = MarkerInfoSubscriber()
    rclpy.spin(marker_info_subscriber)
    marker_info_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
