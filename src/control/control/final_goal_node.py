#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
import math

class GoalHandler(Node):
    def __init__(self):
        super().__init__('goal_handler')

        # Initialize BasicNavigator
        self.navigator = BasicNavigator()

        # Subscriptions
        self.goal_sub = self.create_subscription(PoseStamped, '/final_goal', self.goal_callback, 10)
        self.user_interrupt_sub = self.create_subscription(PoseStamped, '/user_interrupt', self.user_interrupt_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.feedback_pub = self.create_publisher(Float32, '/feedback', 10)
        self.user_interrupt_pub = self.create_publisher(PoseStamped, '/user_interrupt', 10)

        self.current_position = None
        self.last_user_goal = None

        self.get_logger().info("GoalHandler node has been started.")

    def goal_callback(self, msg):
        self.get_logger().info(f"Received goal: {msg.pose.position.x}, {msg.pose.position.y}")
        self.send_goal_to_nav2(msg)

    def user_interrupt_callback(self, msg):
        self.get_logger().info(f"Received user interrupt goal: {msg.pose.position.x}, {msg.pose.position.y}")
        self.last_user_goal = msg

    def odom_callback(self, msg):
        # Update current position from odometry data
        self.current_position = msg.pose.pose

    def send_goal_to_nav2(self, goal_msg):
        try:
            # Send the goal using the BasicNavigator API
            self.get_logger().info("Sending goal to Nav2 stack...")
            self.navigator.goToPose(goal_msg)

            # Wait for the result
            while not self.navigator.isTaskComplete():
                # Get feedback
                feedback = self.navigator.getFeedback()
                if feedback:
                    distance_remaining = feedback.distance_remaining
                    self.get_logger().info(f"Distance remaining: {distance_remaining:.2f} meters")

                    # Publish feedback
                    feedback_msg = Float32()
                    feedback_msg.data = distance_remaining
                    self.feedback_pub.publish(feedback_msg)

            # Goal completed; check the result
            result = self.navigator.getResult()
            self.get_logger().info(f"Goal result: {result}")

            # Perform the post-goal distance check
            self.check_distance_to_last_goal()

        except Exception as e:
            self.get_logger().error(f"Failed to send goal or publish feedback: {e}")

    def calculate_distance(self, pos1, pos2):
        """ Calculate the Euclidean distance between two positions """
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.sqrt(dx**2 + dy**2)

    def check_distance_to_last_goal(self):
        """ Check the distance to the last user goal after reaching the final goal """
        if self.current_position and self.last_user_goal:
            distance = self.calculate_distance(self.current_position.position, self.last_user_goal.pose.position)
            self.get_logger().info(f"Distance to last user goal: {distance:.2f} meters")

            if distance > 3.0:
                self.get_logger().info("Distance is greater than 3 meters, resending last user goal...")
                self.user_interrupt_pub.publish(self.last_user_goal)
            else:
                self.get_logger().info("Distance is less than 3 meters, resetting stored user goal.")
                self.last_user_goal = None
        else:
            self.get_logger().warn("Current position or last user goal is not available.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
