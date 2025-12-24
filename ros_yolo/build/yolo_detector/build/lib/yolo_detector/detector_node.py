import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class DummyYolo(Node):
    def __init__(self):
        super().__init__('dummy_yolo')

        self.pub = self.create_publisher(String, '/yolo/detections', 10)
        self.timer = self.create_timer(0.5, self.publish_dummy)

    def publish_dummy(self):
        # Simulasi buoy kadang muncul, kadang tidak
        if random.random() < 0.5:
            msg = [{
                "class": "buoy",
                "xc": random.randint(200, 440),
                "yc": 200,
                "w": 80,
                "h": 80
            }]
        else:
            msg = []

        ros_msg = String()
        ros_msg.data = json.dumps(msg)
        self.pub.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
