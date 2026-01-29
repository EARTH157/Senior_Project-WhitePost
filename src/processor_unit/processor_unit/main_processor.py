#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Main_Processor(Node):
    def __init__(self):
        super().__init__('Main_processor_node')

    def send_angle(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.target_pub.publish(msg)
        self.get_logger().info(f"📤 Sent Command: Move to {angle:.2f} deg")

def main(args=None):
    rclpy.init(args=args)
    node = Main_Processor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()