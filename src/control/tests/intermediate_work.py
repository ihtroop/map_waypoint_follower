# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import OccupancyGrid
# from nav2_simple_commander.robot_navigator import BasicNavigator

# class NavigationCommander(Node):
#     def __init__(self):
#         super().__init__('navigation_commander')
        
#         # Initialize the BasicNavigator
#         self.navigator = BasicNavigator()
        
#         # Create a subscription to the /user_interrupt topic
#         self.goal_sub = self.create_subscription(
#             PoseStamped,
#             '/user_interrupt',
#             self.goal_callback,
#             10
#         )

#         # Create a subscription to the /map topic
#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             '/map',
#             self.map_callback,
#             10
#         )

#         # Variables to store map parameters
#         self.origin = None
#         self.height = None
#         self.width = None
#         self.resolution = None

#     def map_callback(self, msg):
#         # Extract the map parameters
#         self.origin = msg.info.origin.position
#         self.height = msg.info.height
#         self.width = msg.info.width
#         self.resolution = msg.info.resolution

#         # Calculate the 4 corners of the global cost map
#         corners = self.calculate_corners()

#         # Print the corners to the terminal
#         self.get_logger().info(f"Bottom-right corner: ({corners['bottom_right_x']}, {corners['bottom_right_y']})")
#         self.get_logger().info(f"Bottom-left corner: ({corners['bottom_left_x']}, {corners['bottom_left_y']})")
#         self.get_logger().info(f"Top-right corner: ({corners['top_right_x']}, {corners['top_right_y']})")
#         self.get_logger().info(f"Top-left corner: ({corners['top_left_x']}, {corners['top_left_y']})")

#     def calculate_corners(self):
#         # Calculate the coordinates of the four corners based on the origin, height, width, and resolution
#         corners = {
#             'bottom_left_x': self.origin.x,
#             'bottom_left_y': self.origin.y,
#             'bottom_right_x': self.origin.x + self.width * self.resolution,
#             'bottom_right_y': self.origin.y,
#             'top_left_x': self.origin.x,
#             'top_left_y': self.origin.y + self.height * self.resolution,
#             'top_right_x': self.origin.x + self.width * self.resolution,
#             'top_right_y': self.origin.y + self.height * self.resolution       
#         }
#         return corners

#     def goal_callback(self, msg):
#         if not all([self.origin, self.height, self.width, self.resolution]):
#             self.get_logger().info("Map parameters are not yet received.")
#             return

#         self.get_logger().info(f"Received new goal: {msg}")

#         # Set the goal pose for the navigator
#         goal_pose = msg

#         # Check if the goal lies inside the global cost map
#         corners = self.calculate_corners()
#         goal_x = goal_pose.pose.position.x
#         goal_y = goal_pose.pose.position.y

#         # Apply a small buffer to account for possible rounding errors
#         buffer = 0.01  # 1 cm buffer

#         if (corners['bottom_left_x'] - buffer <= goal_x <= corners['bottom_right_x'] + buffer and
#             corners['bottom_right_y'] - buffer <= goal_y <= corners['top_right_y'] + buffer):
#             self.get_logger().info("------------------yes------------------")
#         else:
#             self.get_logger().info("------------------no-------------------")

#         # Use the navigator to send the goal to the Nav2 stack
#         self.navigator.goToPose(goal_pose)

#         # Wait for the robot to reach the goal or fail
#         while not self.navigator.isTaskComplete():
#             # Feedback about the current status of the goal
#             feedback = self.navigator.getFeedback()
#             if feedback:
#                 distance_remaining = feedback.distance_remaining
#                 self.get_logger().info(f"Distance remaining: {distance_remaining:.2f} meters")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationCommander()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

