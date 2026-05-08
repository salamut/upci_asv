import math
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

from mavros_msgs.msg import State

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

from flask import Flask, render_template
from flask_socketio import SocketIO

# =====================================================
# FLASK
# =====================================================

app = Flask(__name__)

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading'
)

@app.route('/')
def index():
    return render_template('index.html')

# =====================================================
# GLOBAL DATA
# =====================================================

latest_data = {
    "latitude": 0.0,
    "longitude": 0.0,
    "mode": "UNKNOWN",
    "armed": False,
    "battery": 0.0,
    "speed": 0.0,
    "heading": 0.0
}

# =====================================================
# ROS2 NODE
# =====================================================

class MavrosMonitor(Node):

    def __init__(self):

        super().__init__('mavros_monitor')

        # ==========================================
        # QOS
        # ==========================================

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==========================================
        # GPS
        # ==========================================

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile
        )

        # ==========================================
        # STATE
        # ==========================================

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        # ==========================================
        # BATTERY
        # ==========================================

        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile
        )

        # ==========================================
        # VELOCITY
        # ==========================================

        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos_profile
        )

        # ==========================================
        # IMU
        # ==========================================

        self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )

        self.get_logger().info("MAVROS Monitor Started")

    # =====================================================
    # GPS CALLBACK
    # =====================================================

    def gps_callback(self, msg):

        latest_data["latitude"] = msg.latitude
        latest_data["longitude"] = msg.longitude

        socketio.emit('gps', latest_data)

        print("GPS SENT:", latest_data)

    # =====================================================
    # STATE CALLBACK
    # =====================================================

    def state_callback(self, msg):

        latest_data["mode"] = msg.mode
        latest_data["armed"] = msg.armed

        socketio.emit('state', latest_data)

    # =====================================================
    # BATTERY CALLBACK
    # =====================================================

    def battery_callback(self, msg):

        if msg.percentage >= 0:

            latest_data["battery"] = round(
                msg.percentage * 100,
                2
            )

        socketio.emit('battery', latest_data)

    # =====================================================
    # VELOCITY CALLBACK
    # =====================================================

    def velocity_callback(self, msg):

        vx = msg.twist.linear.x
        vy = msg.twist.linear.y

        speed = math.sqrt(vx**2 + vy**2)

        latest_data["speed"] = round(speed, 2)

        socketio.emit('speed', latest_data)

    # =====================================================
    # IMU CALLBACK
    # =====================================================

    def imu_callback(self, msg):

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)

        yaw = math.atan2(siny_cosp, cosy_cosp)

        yaw_deg = math.degrees(yaw)

        if yaw_deg < 0:
            yaw_deg += 360

        latest_data["heading"] = round(yaw_deg, 2)

        socketio.emit('heading', latest_data)

# =====================================================
# ROS THREAD
# =====================================================

def ros_thread():

    rclpy.init()

    node = MavrosMonitor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

# =====================================================
# MAIN
# =====================================================

if __name__ == '__main__':

    # ROS2 Thread
    threading.Thread(
        target=ros_thread,
        daemon=True
    ).start()

    # Flask Server
    socketio.run(
        app,
        host='0.0.0.0',
        port=5000,
        debug=False,
        allow_unsafe_werkzeug=True
    )