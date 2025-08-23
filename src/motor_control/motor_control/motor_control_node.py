import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Motor Control Node Started')

        # Initialize motor control here (e.g., via serial or CAN)    

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        # Convert linear/angular velocity to motor commands for tracked wheels
        self.get_logger().info(f'Motor command received - Linear: {linear}, Angular: {angular}')
        # TODO: Send commands to motor drivers here

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
