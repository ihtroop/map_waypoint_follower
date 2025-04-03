import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        
        # Create subscribers for the topics
        # self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.user_interrupt_sub = self.create_subscription(PoseStamped, '/user_interrupt', self.user_interrupt_callback, 10)
        self.given_goal_sub = self.create_subscription(PoseStamped, '/given_goal', self.given_goal_callback, 10)
        self.aruco_interrupt_sub = self.create_subscription(String, '/aruco_interrupt', self.aruco_interrupt_callback, 10)
        
        # Create publisher for the /status topic
        self.status_pub = self.create_publisher(String, '/status', 10)
        
        # Variables to store the relevant data
        self.current_pose = None
        self.user_goal = None
        self.given_goal = None
        self.stop_published = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose  # Update the robot's current pose

    def user_interrupt_callback(self, msg):
        self.user_goal = msg.pose  # Update the user-defined goal

    def given_goal_callback(self, msg):
        self.given_goal = msg.pose  # Update the given goal

    def aruco_interrupt_callback(self, msg):
        if msg.data == "reached":
            self.publish_status("reached")

    def cmd_vel_callback(self, msg):
        # Check if the robot is stopped
        if (msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and
            msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            
            if not self.stop_published and self.current_pose and self.user_goal:
                distance = self.calculate_distance(self.current_pose.position, self.user_goal.position)
                
                if distance < 1.0:
                    self.publish_status("reached")
                else:
                    self.publish_status("aborted")
                
                self.stop_published = True
        else:
            self.stop_published = False

    def calculate_distance(self, pos1, pos2):
        """Calculate the Euclidean distance between two points."""
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def publish_status(self, status):
        """Helper function to publish the status."""
        status_message = String()
        status_message.data = status
        self.status_pub.publish(status_message)
        self.get_logger().info(f"Published status: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
