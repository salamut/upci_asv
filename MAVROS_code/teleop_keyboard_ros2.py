#!/usr/bin/env python3
"""
Teleop keyboard untuk ArduRover SITL via MAVROS (ROS2 / rclpy).
Menggerakkan kapal dengan RC override di mode MANUAL — uji awal plumbing.

Tombol:
   w / arrow-up     : gas maju (tekan beberapa kali untuk lebih cepat)
   s / arrow-down    : gas mundur
   a / arrow-left    : belok kiri (tahan; otomatis lurus saat dilepas)
   d / arrow-right   : belok kanan
   spasi             : berhenti (gas & kemudi netral)
   q                 : keluar (lepas override, disarm)

Pakai:
   source /opt/ros/humble/setup.bash
   python3 teleop_keyboard_ros2.py
   (jalankan di terminal sungguhan agar keyboard terbaca)
"""

import sys, select, termios, tty, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode, CommandBool

# kanal ArduRover bawaan: ch1 = kemudi (steering), ch3 = gas (throttle)
CH_STEER, CH_THR = 0, 2
NEUTRAL = 1500
THR_STEP, THR_MAX, THR_MIN = 60, 1800, 1200
STEER_OFFSET = 180          # kiri=1320, kanan=1680
STEER_TIMEOUT = 0.4         # detik; kemudi auto-lurus jika tak ada a/d
HZ = 20.0

ARROWS = {"\x1b[A": "w", "\x1b[B": "s", "\x1b[C": "d", "\x1b[D": "a"}


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.state = State()
        self.thr = NEUTRAL
        self.steer = NEUTRAL
        self.last_steer = 0.0
        self.ready = False
        self.tick = 0

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m),
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm  = self.create_client(CommandBool, "/mavros/cmd/arming")
        for cli, n in [(self.cli_mode, "set_mode"), (self.cli_arm, "arming")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f"Service {n} belum siap")

        # terminal mode raw (baca tombol tanpa Enter)
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info("Teleop aktif. w/a/s/d atau panah; spasi=stop; q=keluar")
        self.timer = self.create_timer(1.0 / HZ, self.loop)

    def read_keys(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":                      # mungkin tombol panah
                seq = ch
                for _ in range(2):
                    if select.select([sys.stdin], [], [], 0)[0]:
                        seq += sys.stdin.read(1)
                keys.append(ARROWS.get(seq, ""))
            else:
                keys.append(ch.lower())
        return keys

    def ensure_manual_armed(self):
        if self.tick % 10 != 0:
            return
        if self.state.mode != "MANUAL":
            req = SetMode.Request(); req.custom_mode = "MANUAL"
            self.cli_mode.call_async(req)
        elif not self.state.armed:
            req = CommandBool.Request(); req.value = True
            self.cli_arm.call_async(req)
        else:
            self.ready = True
            self.get_logger().info("MANUAL + armed. Silakan kemudikan.")

    def publish_rc(self):
        msg = OverrideRCIn()
        ch = [0] * 18                              # 0 = lepas (pakai RC)
        ch[CH_STEER] = self.steer
        ch[CH_THR]   = self.thr
        msg.channels = ch
        self.pub.publish(msg)

    def loop(self):
        self.tick += 1
        if not self.ready:
            self.ensure_manual_armed()

        for k in self.read_keys():
            if k == "w":
                self.thr = min(self.thr + THR_STEP, THR_MAX)
            elif k == "s":
                self.thr = max(self.thr - THR_STEP, THR_MIN)
            elif k == "a":
                self.steer = NEUTRAL - STEER_OFFSET; self.last_steer = time.time()
            elif k == "d":
                self.steer = NEUTRAL + STEER_OFFSET; self.last_steer = time.time()
            elif k == " ":
                self.thr = NEUTRAL; self.steer = NEUTRAL
            elif k == "q":
                raise KeyboardInterrupt

        # kemudi auto-lurus jika tak ada a/d
        if time.time() - self.last_steer > STEER_TIMEOUT:
            self.steer = NEUTRAL

        self.publish_rc()
        self.get_logger().info(
            f"gas={self.thr}  kemudi={self.steer}  "
            f"(mode={self.state.mode} armed={self.state.armed})",
            throttle_duration_sec=0.5)

    def shutdown(self):
        # lepas override + netral, lalu kembalikan terminal
        try:
            msg = OverrideRCIn(); msg.channels = [0] * 18
            self.pub.publish(msg)
            req = CommandBool.Request(); req.value = False
            self.cli_arm.call_async(req)
        except Exception:
            pass
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()