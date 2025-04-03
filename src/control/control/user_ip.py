# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import String
# import math
# import time

# class UserInputGoalPublisher(Node):
#     def __init__(self):
#         super().__init__('user_input_goal_publisher')
        
#         # Create publisher for user-defined goal
#         self.user_goal_pub = self.create_publisher(PoseStamped, '/user_interrupt', 10)
        
#         # Create subscription for user feedback
#         self.user_feedback_sub = self.create_subscription(String, '/user_feedback', self.user_feedback_callback, 10)
        
#         # Prompt for the initial goal when the node starts
#         self.get_logger().info("Node started. Prompting for initial goal...")
#         self.ask_for_new_goal()

#     def ask_for_new_goal(self):
#         x, y, yaw = self.get_user_input()
#         if x is not None and y is not None and yaw is not None:
#             self.publish_goal(x, y, yaw)

#     def get_user_input(self):
#         try:
#             x = float(input("Enter the X coordinate: "))
#             y = float(input("Enter the Y coordinate: "))
#             yaw = float(input("Enter the yaw (degrees): "))
#             return x, y, yaw
#         except ValueError:
#             self.get_logger().error("Invalid input. Please enter numeric values.")
#             return None, None, None

#     def euler_to_quaternion(self, roll, pitch, yaw):
#         # Convert Euler angles to quaternion
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)
        
#         q = [0, 0, 0, 0]
#         q[0] = cr * cp * cy + sr * sp * sy
#         q[1] = sr * cp * cy - cr * sp * sy
#         q[2] = cr * sp * cy + sr * cp * sy
#         q[3] = cr * cp * sy - sr * sp * cy
        
#         return q

#     def publish_goal(self, x, y, yaw):
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = "map"
#         goal_pose.pose.position.x = x
#         goal_pose.pose.position.y = y
#         goal_pose.pose.position.z = 0.0
        
#         quaternion = self.euler_to_quaternion(0, 0, math.radians(yaw))
#         goal_pose.pose.orientation.x = quaternion[0]
#         goal_pose.pose.orientation.y = quaternion[1]
#         goal_pose.pose.orientation.z = quaternion[2]
#         goal_pose.pose.orientation.w = quaternion[3]
        
#         self.user_goal_pub.publish(goal_pose)
#         self.get_logger().info(f"Published user-defined goal: {goal_pose}")

#     def user_feedback_callback(self, msg):
#         if msg.data == "Success":
#             self.get_logger().info("Received 'Success'. Prompting for new goal...")
#             self.ask_for_new_goal()

# def main(args=None):
#     rclpy.init(args=args)
#     node = UserInputGoalPublisher()
    
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



















import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
import math
import time

class UserInputGoalPublisher(Node):
    def __init__(self):
        super().__init__('user_input_goal_publisher')
        
        # Create publisher for user-defined goal
        self.user_goal_pub = self.create_publisher(PoseStamped, '/user_interrupt', 10)
        
        # Create subscriber for the status topic
        self.status_sub = self.create_subscription(String, '/status', self.status_callback, 10)

        # Create subscriber for the feedback distance topic
        self.feedback_sub = self.create_subscription(Float32, '/feedback_dist', self.feedback_callback, 10)
        
        # Create subscriber for the marker status topic
        self.marker_status_sub = self.create_subscription(String, '/marker_status', self.marker_status_callback, 10)

        # Variables to keep track of the status and feedback
        self.status = None
        self.new_goal_ready = True
        self.last_distance = None
        self.last_feedback_time = time.time()
        self.static_time_threshold = 5  # 5 seconds threshold for no movement

    def get_user_input(self):
        try:
            x = float(input("Enter the X coordinate: "))
            y = float(input("Enter the Y coordinate: "))
            yaw = float(input("Enter the yaw please: "))
            return x, y, yaw
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
            return None, None, None

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy
        
        return q

    def publish_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        quaternion = self.euler_to_quaternion(0, 0, math.radians(yaw))
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        
        self.user_goal_pub.publish(goal_pose)
        self.get_logger().info(f"Published user-defined goal: {goal_pose}")
        self.new_goal_ready = False
        self.last_distance = None  # Reset distance tracking

    def status_callback(self, msg):
        self.status = msg.data
        self.get_logger().info(f"Received status: {self.status}")
        if self.status in ['reached', 'aborted', 'Success']:
            self.new_goal_ready = True

    def feedback_callback(self, msg):
        current_time = time.time()
        current_distance = msg.data

        if self.last_distance is None:
            self.last_distance = current_distance
            self.last_feedback_time = current_time
            return

        # Check if the distance is less than 0.5m or hasn't changed for 5 seconds
        if current_distance < 0.5:
            self.get_logger().info("Distance below 0.5m. Asking for new goal.")
            self.new_goal_ready = True

        if abs(current_distance - self.last_distance) < 1e-3:  # No significant change in distance
            if (current_time - self.last_feedback_time) > self.static_time_threshold:
                self.get_logger().info("No movement detected for 5 seconds. Asking for new goal.")
                self.new_goal_ready = True
        else:
            self.last_feedback_time = current_time

        self.last_distance = current_distance

    def marker_status_callback(self, msg):
        if msg.data == "1":
            self.get_logger().info("Marker status is 1. Asking for new goal.")
            self.new_goal_ready = True

def main(args=None):
    rclpy.init(args=args)
    node = UserInputGoalPublisher()
    
    while rclpy.ok():
        # Only get user input and publish a new goal if the previous goal has been handled
        if node.new_goal_ready:
            x, y, yaw = node.get_user_input()
            if x is not None and y is not None and yaw is not None:
                node.publish_goal(x, y, yaw)
        
        # Spin once to process feedback, status, and marker status callbacks
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
