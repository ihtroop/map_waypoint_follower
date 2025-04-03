#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Float32
import math
import threading  # For multithreading
import gc  # For garbage collection

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/user_interrupt', self.goal_callback, 10)
        self.feedback_sub = self.create_subscription(Float32, '/feedback', self.feedback_callback, 10)

        # Publishers
        self.feedback_pub = self.create_publisher(String, '/status', 10)
        self.final_goal_pub = self.create_publisher(PoseStamped, '/final_goal', 10)

        # Map parameters
        self.origin = None
        self.height = None
        self.width = None
        self.resolution = None

        # Position and goal tracking
        self.robot_x = None
        self.robot_y = None
        self.user_goal = None
        self.using_intermediate_goal = False

        # Last published goal
        self.last_given_goal = None

        # Multithreading Lock
        self.lock = threading.Lock()

        # Start threads for map and odom updates
        self.map_thread = threading.Thread(target=self.map_thread_function)
        self.odom_thread = threading.Thread(target=self.odom_thread_function)
        self.map_thread.start()
        self.odom_thread.start()

    def map_thread_function(self):
        rclpy.spin(self, executor=rclpy.executors.SingleThreadedExecutor())
    
    def odom_thread_function(self):
        rclpy.spin(self, executor=rclpy.executors.SingleThreadedExecutor())
    
    def map_callback(self, msg):
        # Thread-safe update of map parameters
        with self.lock:
            self.origin = msg.info.origin.position
            self.height = msg.info.height
            self.width = msg.info.width
            self.resolution = msg.info.resolution

        corners = self.calculate_corners()
        # self.get_logger().info(f"Map corners updated: {corners}")
        self.get_logger().info("Map")

    def calculate_corners(self):
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
        return corners

    def odom_callback(self, msg):
        # Thread-safe update of robot position
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
        # self.get_logger().info(f"Robot position updated: ({self.robot_x}, {self.robot_y})")

    def goal_callback(self, msg):
        # Lock the thread to ensure consistent state across map and odom updates
        with self.lock:
            if self.robot_x is None or self.robot_y is None:
                self.get_logger().info("Waiting for odometry information...")
                return

            self.user_goal = msg
            goal_x = self.user_goal.pose.position.x
            goal_y = self.user_goal.pose.position.y

            # Calculate distance between current robot position and user-defined goal
            distance_to_goal = self.calculate_distance(self.robot_x, self.robot_y, goal_x, goal_y)
            self.get_logger().info(f"Received goal at: ({goal_x}, {goal_y}), Distance to goal: {distance_to_goal}")

            if distance_to_goal < 1.0:
                # Goal is within 1 meter
                self.publish_feedback("Success")
                return
            elif distance_to_goal > 2.0:
                # Goal is far away, proceed with nested if-else
                corners = self.calculate_corners()
                if self.is_goal_within_map(goal_x, goal_y, corners):
                    # Goal is inside the map, publish to /final_goal
                    self.get_logger().info("---------------------------GOAL-IN-MAP-------------------------------")
                    self.final_goal_pub.publish(self.user_goal)
                    self.using_intermediate_goal = False
                    self.last_given_goal = self.user_goal
                else:
                    # Goal is outside the map, calculate an intermediate goal
                    self.get_logger().info("----------$$$$$$$$$---------GOAL-NOT-IN-MAP--------------$$$$$$$$$$------")
                    intermediate_goal = self.calculate_intermediate_goal(goal_x, goal_y, corners)
                    self.get_logger().info(f"+++++++++++++++++++++Inter_goal: ({intermediate_goal.pose.position.x}, {intermediate_goal.pose.position.y} ++++++++)")
                    self.final_goal_pub.publish(intermediate_goal)
                    self.using_intermediate_goal = True
                    self.last_given_goal = intermediate_goal

    def is_goal_within_map(self, goal_x, goal_y, corners):
        buffer = 0.05  # Small buffer to account for rounding errors
        return (corners['bottom_left_x'] - buffer <= goal_x <= corners['bottom_right_x'] + buffer and
                corners['bottom_right_y'] - buffer <= goal_y <= corners['top_right_y'] + buffer)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def calculate_intermediate_goal(self, goal_x, goal_y, corners):
        # # Ensure the latest map data is used
        # self.get_logger().info("Updating map before calculating intermediate goal.")
        # with self.lock:
        #     corners = self.calculate_corners()

        # Calculate direction vector from robot to goal
        dir_x = goal_x - self.robot_x
        dir_y = goal_y - self.robot_y
        distance = math.sqrt(dir_x ** 2 + dir_y ** 2)
        dir_x /= distance
        dir_y /= distance

        # Adjust goal_x and goal_y based on map boundaries
        new_goal_x = goal_x
        new_goal_y = goal_y

        # Check and adjust for map boundaries in the X direction
        if goal_x < corners['bottom_left_x']:
            new_goal_x = corners['bottom_left_x'] + 0.5  # Add a small buffer
        elif goal_x > corners['bottom_right_x']:
            new_goal_x = corners['bottom_right_x'] - 0.5  # Subtract a small buffer

        # Check and adjust for map boundaries in the Y direction
        if goal_y < corners['bottom_right_y']:
            new_goal_y = corners['bottom_right_y'] + 0.5  # Add a small buffer
        elif goal_y > corners['top_right_y']:
            new_goal_y = corners['top_right_y'] - 0.5  # Subtract a small buffer

        # Generate intermediate goal using the adjusted coordinates
        intermediate_goal = PoseStamped()
        intermediate_goal.header.frame_id = "map"
        intermediate_goal.header.stamp = self.get_clock().now().to_msg()
        intermediate_goal.pose.position.x = new_goal_x
        intermediate_goal.pose.position.y = new_goal_y
        intermediate_goal.pose.orientation = self.user_goal.pose.orientation

        self.get_logger().info(f"Calculated intermediate goal: ({new_goal_x}, {new_goal_y})")
        return intermediate_goal


    def feedback_callback(self, msg):
        distance_remaining = msg.data
        self.get_logger().info(f"DIST REM. = {distance_remaining}")

        if distance_remaining < 1.0:
            if self.robot_x is None or self.robot_y is None:
                self.get_logger().info("Waiting for updated odometry...")
                return

            # Check distance between robot and last given goal
            if self.last_given_goal:
                last_goal_x = self.last_given_goal.pose.position.x
                last_goal_y = self.last_given_goal.pose.position.y
                distance_to_last_goal = self.calculate_distance(self.robot_x, self.robot_y, last_goal_x, last_goal_y)

                if distance_to_last_goal < 1.0 and not self.using_intermediate_goal:
                    # Check if user-defined goal is inside the map
                    corners = self.calculate_corners()
                    if self.is_goal_within_map(last_goal_x, last_goal_y, corners):
                        # Publish user-defined goal if it's within the map
                        self.get_logger().info("XXXXXXX-------GOAL-IN-MAP--------XXXXXXXX")
                        self.final_goal_pub.publish(self.user_goal)
                    else:
                        # Calculate a new intermediate goal if the user-defined goal is not valid
                        self.get_logger().info("XXXXXXX-------GOAL-NOT-IN-MAP--------XXXXXXXX")
                        intermediate_goal = self.calculate_intermediate_goal(last_goal_x, last_goal_y, corners)
                        self.get_logger().info(f"XXXXXXXXX+++++++Inter_Goal({intermediate_goal.pose.position.x}, {intermediate_goal.pose.position.y})+++++++++XXXXXXXX")
                        self.final_goal_pub.publish(intermediate_goal)
                else:
                    self.get_logger().info("The distance to the last goal is more than 1m or the latest goal is the intermediate goal.")

    def publish_feedback(self, feedback_msg):
        msg = String()
        msg.data = feedback_msg
        self.feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
