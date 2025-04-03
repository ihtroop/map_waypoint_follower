import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class NavigationControlServer(Node):
    def __init__(self):
        super().__init__('navigation_control_server')

        self.is_paused = False
        
        # Subscription to marker_status topic
        self.marker_status_subscription = self.create_subscription(
            String,
            '/marker_status',
            self.marker_status_callback,
            10)

        # Subscription to user_interrupt topic
        self.user_interrupt_subscription = self.create_subscription(
            PoseStamped,
            '/user_interrupt',
            self.user_interrupt_callback,
            10)
        
        self.is_paused = False

        # Client for lifecycle service to change state of bt_navigator
        self.lifecycle_client = self.create_client(ChangeState, 'bt_navigator/change_state')
        while not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info("Navigation Control Server is up and running...")

        # Internal state flag to track if the Nav2 stack is paused        

    def marker_status_callback(self, msg):
        # if msg.data == "1" and not self.is_paused:
            self.get_logger().info('Received marker status 1, deactivating navigation...')
            self.deactivate_navigation()

    def deactivate_navigation(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.lifecycle_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.is_paused = True  # Set the flag to indicate the Nav2 stack is paused
            self.get_logger().info('Navigation deactivated successfully.----{}'.format(self.is_paused))
        else:
            self.get_logger().error('Failed to deactivate navigation.')

    def user_interrupt_callback(self, msg):
        # if self.is_paused==True:
        # Reset the internal state when a new goal is received
        self.is_paused = False 
        self.get_logger().info('Received new goal, ready to pause Nav2 again-----{}'.format(self.is_paused))
            # Reset the pause flag

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationControlServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
