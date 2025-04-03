#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import numpy as np

class IntegratedNavNode(Node):
    def __init__(self):
        super().__init__('integrated_nav_node')

        # Publisher for goal poses
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Subscriber to /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscriber to marker_info topic
        self.marker_subscription = self.create_subscription(
            Int32MultiArray,
            'marker_info',
            self.marker_info_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Lifecycle client for bt_navigator
        self.lifecycle_client = self.create_client(ChangeState, 'bt_navigator/change_state')
        while not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Lifecycle service not available, waiting again...')

        # Variables for tracking movement
        self.last_pose_time = self.get_clock().now().to_msg().sec
        self.last_position = None
        self.position_timeout = 3.0  # seconds
        self.goal_reached = False

        # Variables for marker detection
        self.marker_detected = False
        self.frame_width = 1280
        self.region_left = int(self.frame_width // 3)
        self.region_right = 2 * int(self.frame_width // 3)
        self.rotation_speed_left = 0.3
        self.rotation_speed_right = -0.3
        self.rotation_speed_center = 0.0
        self.area_threshold = 60000

        # Get initial goal values from the user
        self.ask_for_new_goal(initial=True)

    def timer_callback(self):
        if not self.goal_reached and not self.marker_detected:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = self.x_goal
            goal_msg.pose.position.y = self.y_goal
            goal_msg.pose.orientation.w = self.orientation_w

            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f'Goal pose published: x={self.x_goal}, y={self.y_goal}, orientation_w={self.orientation_w}')

    def odom_callback(self, msg):
        # Update last pose time
        self.last_pose_time = self.get_clock().now().to_msg().sec

        # Extract pose from /odom message
        x_current = msg.pose.pose.position.x
        y_current = msg.pose.pose.position.y
        w_current = msg.pose.pose.orientation.w
        current_position = (x_current, y_current, w_current)

        # Define tolerance for reaching the goal
        tolerance = 1.0

        if (abs(x_current - self.x_goal) < tolerance and
            abs(y_current - self.y_goal) < tolerance and
            abs(w_current - self.orientation_w) < tolerance):
            self.goal_reached = True
            self.get_logger().info('Goal reached. Asking for new goal...')
            self.ask_for_new_goal()

        # Check if the position has changed
        if self.last_position and self.last_position == current_position:
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.last_pose_time > self.position_timeout:
                self.get_logger().info('No movement detected for 3 seconds. Shutting down.')
                self.destroy_node()
                rclpy.shutdown()
        else:
            # Update the last position
            self.last_position = current_position

    def marker_info_callback(self, msg):
        # Check if marker information is empty
        if not msg.data:
            self.marker_detected = False
            return

        self.marker_detected = True
        # Deactivate the navigation
        self.deactivate_navigation()

        # Convert received data back to numpy array
        marker_info_np = np.array(msg.data).reshape(-1, 12)

        # Process marker data
        for marker_id, tl_x, tr_x, bl_x, br_x, tl_y, tr_y, bl_y, br_y, center_x, center_y, area in marker_info_np:
            # Determine the region where the marker center lies
            if center_x < self.region_left:
                rotation_cmd = self.rotation_speed_left
            elif center_x > self.region_right:
                rotation_cmd = self.rotation_speed_right
            else:
                rotation_cmd = self.rotation_speed_center

            area_error = self.area_threshold - area

            twist_cmd = Twist()
            twist_cmd.angular.z = rotation_cmd
            twist_cmd.linear.x = float(area_error) * 0.00002  # Scaling factor

            if area_error < 50:
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_cmd)
                self.get_logger().info("Area error below threshold. Stopping.")
                return

            self.cmd_vel_pub.publish(twist_cmd)
            self.get_logger().info(f"Rotational Command: {rotation_cmd}, Area Error: {area_error}")

    def deactivate_navigation(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.lifecycle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Navigation deactivated successfully.')
        else:
            self.get_logger().error('Failed to deactivate navigation.')

    def ask_for_new_goal(self, initial=False):
        try:
            if initial:
                print("Enter the initial goal coordinates:")
            else:
                print("Enter the new goal coordinates:")

            self.x_goal = float(input("Enter the x coordinate of the goal: "))
            self.y_goal = float(input("Enter the y coordinate of the goal: "))
            self.orientation_w = float(input("Enter the orientation (w) of the goal: "))
            self.goal_reached = False
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numerical values.")
            self.ask_for_new_goal(initial)

def main(args=None):
    rclpy.init(args=args)
    integrated_nav_node = IntegratedNavNode()
    rclpy.spin(integrated_nav_node)
    integrated_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
