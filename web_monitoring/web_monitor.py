import math
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Float64

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from flask import Flask, render_template
from flask_socketio import SocketIO

# =====================================================
# FLASK
# =====================================================
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

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
    "speed": 0.0,
    "heading": 0.0,   # heading KOMPAS (0 = Utara, searah jarum jam)
}

# =====================================================
# ROS2 NODE
# =====================================================
class MavrosMonitor(Node):

    def __init__(self):
        super().__init__('mavros_monitor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self.gps_callback, qos)
        self.create_subscription(State, '/mavros/state',
                                 self.state_callback, qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self.velocity_callback, qos)
        # SUMBER HEADING YANG BENAR: heading kompas langsung dari MAVROS,
        # bukan dihitung dari yaw IMU (yang konvensinya ENU/ambigu).
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 self.heading_callback, qos)

        self.get_logger().info("MAVROS Monitor Started")

    def gps_callback(self, msg):
        latest_data["latitude"] = msg.latitude
        latest_data["longitude"] = msg.longitude
        socketio.emit('gps', latest_data)

    def state_callback(self, msg):
        latest_data["mode"] = msg.mode
        latest_data["armed"] = msg.armed
        socketio.emit('state', latest_data)

    def velocity_callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        latest_data["speed"] = round(math.sqrt(vx*vx + vy*vy), 2)
        socketio.emit('speed', latest_data)

    def heading_callback(self, msg):
        # msg.data sudah dalam derajat kompas (0 = Utara). Tidak perlu konversi.
        latest_data["heading"] = round(msg.data, 2)
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
    threading.Thread(target=ros_thread, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000,
                 debug=False, allow_unsafe_werkzeug=True)