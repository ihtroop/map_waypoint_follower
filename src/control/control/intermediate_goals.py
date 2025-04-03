#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Float32  # Import for publishing float data
import math

class NavigationCommander(Node):
    def __init__(self):
        super().__init__('navigation_commander')
        
        # Initialize the BasicNavigator
        self.navigator = BasicNavigator()

        # Create a subscription to the /map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )     
        # Create a subscription to the /user_interrupt topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/user_interrupt',
            self.goal_callback,
            10
        )

        # Create a subscription to the /odom topic to track the robot's current location
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create a publisher for the /feedback_dist topic
        self.feedback_pub = self.create_publisher(Float32, '/feedback_dist', 10)

        # Variables to store map parameters
        self.origin = None
        self.height = None
        self.width = None
        self.resolution = None

        # Variables to track the robot's position
        self.robot_x = None
        self.robot_y = None

        # Flag to track if we are using an intermediate goal
        self.using_intermediate_goal = False

    def calculate_corners(self):
        # Calculate the coordinates of the four corners based on the origin, height, width, and resolution
        corners = {
            'bottom_left_x': self.origin.x,
            'bottom_left_y': self.origin.y,
            'bottom_right_x': self.origin.x + self.width * self.resolution,
            'bottom_right_y': self.origin.y,
            'top_left_x': self.origin.x,
            'top_left_y': self.origin.y + self.height * self.resolution,
            'top_right_x': self.origin.x + self.width * self.resolution,
            'top_right_y': self.origin.y + self.height * self.resolution       
        }
        self.get_logger().info("$$$$####----Map parameters updated----####$$$$")

        return corners

    def map_callback(self, msg):
        # Extract the map parameters
        self.origin = msg.info.origin.position
        self.height = msg.info.height
        self.width = msg.info.width
        self.resolution = msg.info.resolution

        # Calculate the 4 corners of the global cost map
        corners = self.calculate_corners()

        # Print the corners to the terminal
        self.get_logger().info(f"Bottom-right corner: ({corners['bottom_right_x']}, {corners['bottom_right_y']})")
        self.get_logger().info(f"Bottom-left corner: ({corners['bottom_left_x']}, {corners['bottom_left_y']})")
        self.get_logger().info(f"Top-right corner: ({corners['top_right_x']}, {corners['top_right_y']})")
        self.get_logger().info(f"Top-left corner: ({corners['top_left_x']}, {corners['top_left_y']})")

    def odom_callback(self, msg):
        # Update the robot's current position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        if not all([self.origin, self.height, self.width, self.resolution]):
            self.get_logger().info("Map parameters are not yet received.")
            return

        if self.robot_x is None or self.robot_y is None:
            self.get_logger().info("Robot position not yet received.")
            return

        self.get_logger().info(f"Received new goal: {msg}")

        # Set the goal pose for the navigator
        user_goal_pose = msg

        while True:
            # Check if the goal lies inside the global cost map
            corners = self.calculate_corners()
            goal_x = user_goal_pose.pose.position.x
            goal_y = user_goal_pose.pose.position.y

            # Apply a small buffer to account for possible rounding errors
            buffer = 0.05  # 1 cm buffer
            if (corners['bottom_left_x'] - buffer <= goal_x <= corners['bottom_right_x'] + buffer and
                corners['bottom_right_y'] - buffer <= goal_y <= corners['top_right_y'] + buffer):
                self.get_logger().info("------------yes------------")
                # If the goal is inside the map, send it directly to the Nav2 stack
                self.navigator.goToPose(user_goal_pose)
                self.using_intermediate_goal = False
                
            else:
                self.get_logger().info("------------no-------------")
                # Calculate an intermediate goal that lies within the map with 0.5m padding
                intermediate_goal = PoseStamped()
                # intermediate_goal.pose.position.x = None
                # intermediate_goal.pose.position.y = None
                intermediate_goal = self.calculate_intermediate_goal(goal_x, goal_y, corners, user_goal_pose)
                self.get_logger().info(f"Intermediate goal: ({intermediate_goal.pose.position.x}, {intermediate_goal.pose.position.y})")
                # Send the intermediate goal to the Nav2 stack
                self.navigator.goToPose(intermediate_goal)
                self.using_intermediate_goal = True

            # Wait for the robot to reach the goal or fail
            while not self.navigator.isTaskComplete():
                # Feedback about the current status of the goal
                feedback = self.navigator.getFeedback()
                if feedback:
                    distance_remaining = feedback.distance_remaining
                    self.get_logger().info(f"Distance remaining: {distance_remaining:.2f} meters")
                    
                    # Publish the distance remaining to the /feedback_dist topic
                    distance_msg = Float32()
                    distance_msg.data = distance_remaining
                    self.feedback_pub.publish(distance_msg)

                    # If we are using an intermediate goal and the robot is within 0.5m of the goal, recalculate
                    if self.using_intermediate_goal and distance_remaining < 0.5:
                        self.get_logger().info("Intermediate goal reached. Recalculating goal...")

                        # Update the map information
                        corners = self.calculate_corners()

                        # Check again if the user-defined goal lies within the updated map
                        if (corners['bottom_left_x'] - buffer <= goal_x <= corners['bottom_right_x'] + buffer and
                            corners['bottom_right_y'] - buffer <= goal_y <= corners['top_right_y'] + buffer):
                            self.get_logger().info("User-defined goal is now within the map. Navigating to user-defined goal.")
                            self.navigator.goToPose(user_goal_pose)
                            self.using_intermediate_goal = False
                        else:
                            self.get_logger().info("User-defined goal is still outside the map. Calculating a new intermediate goal.")
                            # Calculate a new intermediate goal
                            intermediate_goal = PoseStamped()
                            # intermediate_goal.pose.position.x = None
                            # intermediate_goal.pose.position.y = None
                            intermediate_goal = self.calculate_intermediate_goal(goal_x, goal_y, corners, user_goal_pose)
                            self.get_logger().info(f"New intermediate goal: ({intermediate_goal.pose.position.x}, {intermediate_goal.pose.position.y})")
                            self.navigator.goToPose(intermediate_goal)
                    
                    # If we are close enough to the user-defined goal, stop
                    if not self.using_intermediate_goal and distance_remaining < 0.5:
                        self.get_logger().info("User-defined goal reached. Exiting.")
                        return

    def calculate_intermediate_goal(self, goal_x, goal_y, corners, original_goal_pose):
        # Calculate the direction vector from the robot's position to the goal
        dir_x = goal_x - self.robot_x
        dir_y = goal_y - self.robot_y

        # Normalize the direction vector
        distance = math.sqrt(dir_x**2 + dir_y**2)
        dir_x /= distance
        dir_y /= distance

        # Find the intersection of the line with the map boundaries, with 0.5m padding
        new_goal_x = goal_x
        new_goal_y = goal_y

        if goal_x < corners['bottom_left_x']:
            new_goal_x = corners['bottom_left_x'] + 0.5
        elif goal_x > corners['bottom_right_x']:
            new_goal_x = corners['bottom_right_x'] - 0.5

        if goal_y < corners['bottom_right_y']:
            new_goal_y = corners['bottom_right_y'] + 0.5
        elif goal_y > corners['top_right_y']:
            new_goal_y = corners['top_right_y'] - 0.5

        # Adjust the new goal to lie on the line from the robot to the original goal
        new_goal_x = self.robot_x + dir_x * (new_goal_x - self.robot_x)
        new_goal_y = self.robot_y + dir_y * (new_goal_y - self.robot_y)

        # Create a new PoseStamped message for the intermediate goal
        intermediate_goal = PoseStamped()
        intermediate_goal.header.frame_id = "map"
        intermediate_goal.header.stamp = self.get_clock().now().to_msg()
        intermediate_goal.pose.position.x = new_goal_x
        intermediate_goal.pose.position.y = new_goal_y
        intermediate_goal.pose.orientation = original_goal_pose.pose.orientation

        return intermediate_goal

def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
