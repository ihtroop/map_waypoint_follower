import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SingleMessagePublisher(Node):
    def __init__(self):
        super().__init__('single_message_publisher')
        
        # Create publisher for the string message
        self.single_message_pub = self.create_publisher(String, '/marker_status', 10)
        
        # Publish the message once
        self.publish_once()

    def publish_once(self):
        message = String()
        message.data = "1"
        self.single_message_pub.publish(message)
        self.get_logger().info(f"Published message: {message.data}")
        
        # Destroy the node after publishing to ensure it only publishes once
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SingleMessagePublisher()
    rclpy.spin(node)  # This will keep the node alive until it is explicitly destroyed
    rclpy.shutdown()

if __name__ == '__main__':
    main()
