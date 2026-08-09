#!/usr/bin/env python3
"""
Kontrol servo di MAIN OUT 8 Pixhawk via MAVROS (ROS2 / rclpy).
MAIN OUT 8 disetel passthrough dari RC channel 7, jadi servo digerakkan dengan
RC override pada ch7. Rentang 0-180 derajat dipetakan linear ke PWM 1000-2000 us.

Parameter ArduPilot (fungsi "mapped", butuh Rover >= 4.6):
   SERVO8_FUNCTION = 146       # RCIN7 mapped: input ch7 DISKALAKAN ke MIN..MAX
   SERVO8_MIN = 500 ; SERVO8_MAX = 2500 ; SERVO8_TRIM = 1500
   SERVO8_REVERSED = 0         # 1 untuk membalik arah (dihormati oleh 146)
   RC7_MIN = 1000 ; RC7_MAX = 2000 ; RC7_OPTION = 0
   MAV_GCS_SYSID = 1           # <4.6 namanya SYSID_MYGCS; harus = sysid MAVROS

Kenapa 146, bukan 57: fungsi 57 (RCIN7) adalah passthrough MENTAH — PWM dari
receiver diteruskan apa adanya dan SERVO8_MIN/MAX/REVERSED tidak berlaku
(terukur di Pixhawk 30 Jul 2026: ch7=800 -> main8=800, ch7=2200 -> main8=2200).
Karena receiver hanya sanggup ~1000-2000 us sementara servo butuh 500-2500 us
untuk 180 deg, stik penuh cuma memberi ~90 deg. Fungsi 146 menormalkan input
lewat RC7_MIN/MAX/TRIM lalu memetakannya ke SERVO8_MIN..SERVO8_MAX
(SRV_Channel_aux.cpp:61-70, tipe ANGLE), jadi 1000-2000 us dari receiver ATAU
dari override di bawah menjadi 500-2500 us di MAIN OUT 8 -> 180 deg penuh,
baik lewat transmitter maupun lewat node ini.

Konsekuensinya: kirimlah 1000-2000 us dari sini (default di bawah) dan biarkan
FCU yang menskalakan. Rentang fisik diatur dari SERVO8_MIN/MAX, bukan dari
--pwm-min/--pwm-max. Kalau SERVO8_FUNCTION dikembalikan ke 57, barulah span
lebar dipakai di sini: --pwm-min 500 --pwm-max 2500.

Pakai:
   source /opt/ros/humble/setup.bash
   python3 servo8_rc7_ros2.py                  # tahan di 90 deg, tunggu perintah topic
   python3 servo8_rc7_ros2.py --angle 180      # langsung ke 180 deg
   python3 servo8_rc7_ros2.py --sweep          # sapu 0<->180 terus (uji mekanik)
   python3 servo8_rc7_ros2.py --monitor        # amati stik ch7 vs MAIN OUT 8,
                                               # tanpa override (untuk kalibrasi)

   # ubah sudut saat node jalan:
   ros2 topic pub -1 /servo8/angle std_msgs/Float64 "{data: 45.0}"

Catatan: override harus dikirim terus-menerus, ArduPilot melepasnya setelah
RC_OVERRIDE_TIME (default 3 s) tanpa pesan baru. Servo passthrough umumnya
bergerak walau disarm; kalau diam, cek safety switch (BRD_SAFETY_DEFLT).
"""

import argparse, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut, State

CH_SERVO = 6                 # index 0-based -> RC channel 7
OUT_SERVO = 7                # index 0-based -> MAIN OUT 8 (untuk baca /mavros/rc/out)
ANGLE_MAX = 180.0            # deg
PWM_MIN, PWM_MAX = 1000, 2000  # us untuk 0 dan 180 deg; dengan SERVO8_FUNCTION=146
                               # FCU menskalakannya ke SERVO8_MIN..MAX (500-2500)
SWEEP_RATE = 60.0            # deg/s saat mode --sweep
HZ = 20.0


def clamp_angle(deg):
    return max(0.0, min(ANGLE_MAX, deg))


class Servo8(Node):
    def __init__(self, angle, sweep, pwm_min, pwm_max, monitor=False):
        super().__init__("servo8_rc7")
        self.pwm_min, self.pwm_max = pwm_min, pwm_max
        self.target = clamp_angle(angle)
        self.sweep = sweep
        self.monitor = monitor
        self.sweep_dir = 1.0
        self.state = State()
        self.rc_out = None
        self.rc_in = None
        self.t_prev = time.time()

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m),
                                 qos_profile_sensor_data)
        self.create_subscription(RCOut, "/mavros/rc/out", self.on_rc_out,
                                 qos_profile_sensor_data)
        self.create_subscription(RCIn, "/mavros/rc/in", self.on_rc_in,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, "/servo8/angle", self.on_angle, 10)
        self.pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        if monitor:
            mode = "MONITOR (tanpa override, ch7 tetap milik transmitter)"
        else:
            mode = "sapu 0<->180" if sweep else f"tahan {self.target:.0f} deg"
        self.get_logger().info(
            f"Servo MAIN OUT 8 via RC ch7 ({self.pwm_min}-{self.pwm_max} us), {mode}")
        self.timer = self.create_timer(1.0 / HZ, self.loop)

    def angle_to_pwm(self, deg):
        span = self.pwm_max - self.pwm_min
        return int(round(self.pwm_min + span * clamp_angle(deg) / ANGLE_MAX))

    def on_angle(self, msg):
        if not 0.0 <= msg.data <= ANGLE_MAX:
            self.get_logger().warn(
                f"Sudut {msg.data:.1f} di luar 0-{ANGLE_MAX:.0f} deg, dipotong")
        self.target = clamp_angle(msg.data)
        self.sweep = False                      # perintah manual menghentikan sapuan

    def on_rc_out(self, msg):
        if len(msg.channels) > OUT_SERVO:
            self.rc_out = msg.channels[OUT_SERVO]

    def on_rc_in(self, msg):
        if len(msg.channels) > CH_SERVO:
            self.rc_in = msg.channels[CH_SERVO]

    def publish_rc(self, pwm):
        msg = OverrideRCIn()
        # kanal lain CHAN_NOCHANGE supaya kemudi/gas tidak ikut terganggu
        ch = [OverrideRCIn.CHAN_NOCHANGE] * 18
        ch[CH_SERVO] = pwm
        msg.channels = ch
        self.pub.publish(msg)

    def loop(self):
        now = time.time()
        dt, self.t_prev = now - self.t_prev, now

        if self.monitor:
            # hanya mengamati: berguna untuk kalibrasi RC7_MIN/MAX pakai stik asli
            rin = f"{self.rc_in} us" if self.rc_in is not None else "n/a"
            rout = f"{self.rc_out} us" if self.rc_out is not None else "n/a"
            self.get_logger().info(f"stik ch7={rin}  ->  main8={rout}",
                                   throttle_duration_sec=0.25)
            return

        if self.sweep:
            self.target += self.sweep_dir * SWEEP_RATE * dt
            if self.target >= ANGLE_MAX:
                self.target, self.sweep_dir = ANGLE_MAX, -1.0
            elif self.target <= 0.0:
                self.target, self.sweep_dir = 0.0, 1.0

        pwm = self.angle_to_pwm(self.target)
        self.publish_rc(pwm)

        out = f"{self.rc_out} us" if self.rc_out is not None else "n/a"
        self.get_logger().info(
            f"sudut={self.target:6.1f} deg  ch7={pwm} us  main8={out}  "
            f"(mode={self.state.mode} armed={self.state.armed})",
            throttle_duration_sec=0.5)

    def shutdown(self):
        # lepas override ch7 -> servo kembali ikut RC transmitter
        try:
            msg = OverrideRCIn()
            ch = [OverrideRCIn.CHAN_NOCHANGE] * 18
            ch[CH_SERVO] = OverrideRCIn.CHAN_RELEASE
            msg.channels = ch
            for _ in range(5):
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.02)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Servo MAIN OUT 8 via RC ch7 (MAVROS)")
    ap.add_argument("--angle", type=float, default=90.0, help="sudut awal, 0-180 deg")
    ap.add_argument("--sweep", action="store_true", help="sapu 0<->180 terus-menerus")
    ap.add_argument("--pwm-min", type=int, default=PWM_MIN, help="us untuk 0 deg")
    ap.add_argument("--pwm-max", type=int, default=PWM_MAX, help="us untuk 180 deg")
    ap.add_argument("--monitor", action="store_true",
                    help="hanya amati stik ch7 vs MAIN OUT 8, tanpa override")
    cli, ros_args = ap.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Servo8(cli.angle, cli.sweep, cli.pwm_min, cli.pwm_max, cli.monitor)
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
