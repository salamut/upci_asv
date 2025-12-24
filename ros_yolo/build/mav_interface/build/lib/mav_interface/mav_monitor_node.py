import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State

class MavMonitor(Node):
    def __init__(self):
        super().__init__('mav_monitor')
        self.create_subscription(State, '/mavros/state', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Mode: {msg.mode} | Armed: {msg.armed}")


def main(args=None):
    rclpy.init(args=args)
    node = MavMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
