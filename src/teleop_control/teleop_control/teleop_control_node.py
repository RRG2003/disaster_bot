import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop Node Started')
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        msg = Twist()
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    msg.linear.x = 0.5
                elif key == 's':
                    msg.linear.x = -0.5
                elif key == 'a':
                    msg.angular.z = 0.5
                elif key == 'd':
                    msg.angular.z = -0.5
                elif key == ' ':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                elif key == '\x03':  # Ctrl-C
                    break
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
