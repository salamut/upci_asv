import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from time import sleep
from cv_bridge import CvBridge
import pyvirtualcam

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.get_logger().info("Object Detection Node has been started.")
        self.camera = "/dev/video3"  # Default camera device
        self.model_name = "yolov8n.pt"  # Default model path
        self.detection_threshold = 0.8  # Default detection threshold
        # self.timer = self.create_timer(0.05, self.timer_callback)
        self.image_publisher = self.create_publisher(
            Image,
            "/asv/object_detection/image_up",
            qos_profile=qos_profile,
        )
        self.bridge = CvBridge()
        self.virtual_cam = pyvirtualcam.Camera(width=640, height=480, fps=30, device="/dev/video4")

    def start(self):
        self.get_logger().info(f"Starting object detection on camera: {self.camera}")
        self.get_logger().info(
            f"Using model: {self.model_name} with threshold: {self.detection_threshold}"
        )
        # Here you would add the code to initialize your object detection model
        # and start processing video frames from the camera.
        model_path = os.path.join(
            get_package_share_directory("object_detection"), "models", self.model_name
        )
        model = YOLO(model_path)

        cap = cv2.VideoCapture(self.camera)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open camera: {self.camera}")
            return
        while True:
            ret, frame = cap.read()
            frame = cv2.resize(frame, (640, 480))
            if not ret:
                self.get_logger().error("Failed to grab frame")
                break
            results = model(frame)
            annotated_frame = results[0].plot()
            boxes = results[0].boxes
            valid_boxes = [b for b in boxes if b.conf.item() > self.detection_threshold]
            if len(valid_boxes) > 0:
                annotated_frame = mark_image(annotated_frame)
            rgb_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            self.virtual_cam.send(rgb_frame)
            self.virtual_cam.sleep_until_next_frame()
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_publisher.publish(ros_image)
            sleep(0.04)
def mark_image(image):
    cv2.rectangle(image, (0, 0), (10, 10), (0, 255, 0), thickness=-1)
    return image

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    node.start()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
