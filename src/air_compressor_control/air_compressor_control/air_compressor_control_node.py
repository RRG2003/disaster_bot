import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class AirCompressorNode(Node):
    def __init__(self):
        super().__init__('air_compressor_node')
        self.subscription = self.create_subscription(
            Bool,
            '/vine/inflate',
            self.inflate_callback,
            10)
        self.get_logger().info('Air Compressor Node Started')

        # Initialize GPIO or serial interface to compressor control

    def inflate_callback(self, msg):
        if msg.data:
            self.get_logger().info('Inflating vine')
            # TODO: send command to start compressor and open valves
        else:
            self.get_logger().info('Deflating vine')
            # TODO: send command to stop compressor and close valves

def main(args=None):
    rclpy.init(args=args)
    node = AirCompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
