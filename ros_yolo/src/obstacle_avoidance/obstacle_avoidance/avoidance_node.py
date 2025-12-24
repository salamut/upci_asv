#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
import json
import time


class SmoothAvoidance(Node):
    def __init__(self):
        super().__init__("smooth_avoidance")

        # Subscribers
        self.create_subscription(String, "/yolo/detections", self.detection_cb, 10)
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)

        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/mavros/set_mode", 10)

        # Service client
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Internal state
        self.current_state = None
        self.buoy_detected = False
        self.last_buoy_time = 0

        # Control phase timers
        self.last_phase_change = time.time()
        self.phase = "AUTO"  # AUTO → GUIDED → AUTO ...

        # Timer loop
        self.timer = self.create_timer(0.05, self.main_loop)

    # --------------------------
    # CALLBACKS
    # --------------------------

    def state_cb(self, msg):
        self.current_state = msg

    def detection_cb(self, msg):
        try:
            detections = json.loads(msg.data)
        except:
            return

        for det in detections:
            if det.get("class") == "buoy":
                self.buoy_detected = True
                self.last_buoy_time = time.time()
                self.buoy_x = det["xc"]
                return

        # jika tidak ada buoy
        self.buoy_detected = False

    # --------------------------
    # MODE SWITCH HELPER
    # --------------------------

    def set_mode(self, mode):
        # Service call
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_client.call_async(req)
            self.get_logger().info(f"[SERVICE] SetMode → {mode}")

        # Topic fallback
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"[TOPIC] SetMode → {mode}")

    # --------------------------
    # MAIN LOOP
    # --------------------------

    def main_loop(self):
        now = time.time()

        # --------------------------
        # PHASE 1 → AUTO (10s)
        # --------------------------
        if self.phase == "AUTO":
            if self.current_state and self.current_state.mode != "AUTO":
                self.set_mode("AUTO")

            # stop any velocity (very important!)
            self.publish_zero_velocity()

            # setelah 10 detik → pindah ke GUIDED jika ada buoy
            if now - self.last_phase_change > 10:
                if (now - self.last_buoy_time) < 10:   # buoy detected recently
                    self.phase = "GUIDED"
                    self.last_phase_change = now
                    self.get_logger().info("Switching → GUIDED (avoidance active)")
                else:
                    # tidak ada buoy → tetap auto reset timer
                    self.last_phase_change = now

        # --------------------------
        # PHASE 2 → GUIDED (2s)
        # --------------------------
        elif self.phase == "GUIDED":

            # Pastikan mode GUIDED aktif
            if self.current_state and self.current_state.mode != "GUIDED":
                self.set_mode("GUIDED")

            # Smooth avoidance movement
            self.smooth_avoidance_motion()

            # Setelah 2 detik → kembali ke AUTO
            if now - self.last_phase_change > 2:
                self.phase = "AUTO"
                self.last_phase_change = now
                self.get_logger().info("Switching → AUTO (resume mission)")

    # --------------------------
    # SMOOTH AVOIDANCE MOTION
    # --------------------------

    def smooth_avoidance_motion(self):

        frame_center = 640 / 2
        err = self.buoy_x - frame_center

        turn = max(min(err / 300.0, 1.0), -1.0)   # smooth scale
        speed = 0.3

        cmd = TwistStamped()
        cmd.twist.linear.x = speed
        cmd.twist.angular.z = -turn * 0.6  # smooth turning
        self.vel_pub.publish(cmd)

    # --------------------------
    # STOP MOVEMENT WHEN AUTO
    # --------------------------
    def publish_zero_velocity(self):
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.vel_pub.publish(cmd)


def main(args=None):
        rclpy.init(args=args)
        node = SmoothAvoidance()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()
