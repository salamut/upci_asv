#!/usr/bin/env python3
"""
Node guidance LOS SADAR-GERBANG untuk ArduRover via MAVROS (ROS2 / rclpy).

Mengikuti acuan (reference_with_gates.plan) di mode GUIDED, dan saat kapal
melewati tiap waypoint GERBANG, ia mencatat XTE & heading error pada titik
terdekat lalu menilai LULUS/GAGAL terhadap kriteria tugas akhir:
    XTE <= 0.30 m  dan  heading error <= 10 derajat.

ROS2 Humble. Untuk SITL maupun lapangan.

Pakai:
   source /opt/ros/humble/setup.bash
   python3 los_gate_guidance_ros2.py reference_with_gates.plan
"""

import sys, math, csv, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

# ---------- knob ----------
LOOKAHEAD       = 1.5      # Delta (m): diperketat agar menempel rapat & sudut tak terpotong
ACCEPT_RADIUS   = 0.25     # m: kecil = kapal mendekati tiap gerbang dulu sebelum belok
                           #     (membatasi error di gerbang ~ radius ini)
CRUISE_SPEED    = 0.3      # m/s: sangat pelan untuk uji aman (hindari tabrakan keras).
                           #       naikkan bertahap (0.4, 0.5, 0.6) bila sudah yakin.
LOOP_HZ         = 10.0
USE_ILOS        = False    # True untuk kompensasi arus
ILOS_GAIN       = 0.05
ANCHOR_AT_START = False   # False = koordinat ABSOLUT: kursus tetap di lokasi
                          #         aslinya, jalankan ulang selalu mengacu ke sana.
                          #         (pastikan SITL di-spawn di lokasi kursus)
GATE_IDX        = [1, 2, 5, 7, 8, 9]   # gerbang masuk/keluar tiap sisi (indeks setelah lead-in):
                                       # Barat WP1(masuk) WP2(keluar), Selatan WP5 WP7, Timur WP8 WP9
                                       # WP0=lead-in, WP10=lead-out (bukan gerbang)
XTE_LIMIT       = 0.30     # m  (kriteria)
HDG_LIMIT       = 10.0     # deg (kriteria)
CSV_PATH        = "/tmp/los_gate_log.csv"

EARTH_R = 6378137.0

def parse_plan(path):
    d = json.load(open(path))
    return [(it["params"][4], it["params"][5])
            for it in d["mission"]["items"] if it.get("command") in (16, 82)]

def ll_to_enu(lat, lon, lat0, lon0):
    return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * EARTH_R)

def wrap_pi(a):  return math.atan2(math.sin(a), math.cos(a))
def wrap180(d):  return (d + 180.0) % 360.0 - 180.0


class LOSGate(Node):
    def __init__(self, plan_path):
        super().__init__("los_gate_guidance")
        self.wp_ll = parse_plan(plan_path)
        l0, o0 = self.wp_ll[0]
        self.off = [ll_to_enu(la, lo, l0, o0) for la, lo in self.wp_ll]

        self.state = State(); self.hdg = 0.0
        self.x = self.y = 0.0
        self.origin = None; self.have_fix = False
        self.idx = 0; self.y_int = 0.0
        self.ready = False; self.tick = 0; self.finished = False
        self.phase = "TO_START"   # menuju WP awal dulu, baru "FOLLOW"
        self.gate_min = {}      # gate_wp -> (jarak_min, xte, herr)
        self.gate_done = {}     # hasil per gerbang

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), qos_profile_sensor_data)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 lambda m: setattr(self, "hdg", m.data), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self.cb_gps, qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm  = self.create_client(CommandBool, "/mavros/cmd/arming")
        for cli, n in [(self.cli_mode, "set_mode"), (self.cli_arm, "arming")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f"Service {n} belum siap")

        self.csv = open(CSV_PATH, "w", newline=""); self.w = csv.writer(self.csv)
        self.w.writerow(["t", "wp_idx", "xte_m", "along_m", "chi_d_deg",
                         "hdg_deg", "hdg_err_deg", "di_gerbang"])
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / LOOP_HZ, self.loop)

    def cb_gps(self, msg):
        if self.origin is None:
            self.origin = ((msg.latitude, msg.longitude) if ANCHOR_AT_START
                           else (self.wp_ll[0][0], self.wp_ll[0][1]))
        self.x, self.y = ll_to_enu(msg.latitude, msg.longitude, *self.origin)
        self.have_fix = True

    def step(self):
        xk, yk = self.off[self.idx]; xk1, yk1 = self.off[self.idx + 1]
        alpha = math.atan2(yk1 - yk, xk1 - xk)
        dx, dy = self.x - xk, self.y - yk
        along =  dx * math.cos(alpha) + dy * math.sin(alpha)
        cross = -dx * math.sin(alpha) + dy * math.cos(alpha)
        if USE_ILOS:
            self.y_int += (LOOKAHEAD * cross /
                           (LOOKAHEAD**2 + (cross + ILOS_GAIN*self.y_int)**2)) / LOOP_HZ
            chi_d = alpha + math.atan2(-(cross + ILOS_GAIN*self.y_int), LOOKAHEAD)
        else:
            chi_d = alpha + math.atan2(-cross, LOOKAHEAD)
        return wrap_pi(chi_d), cross, along

    def at_waypoint(self, along):
        xk1, yk1 = self.off[self.idx + 1]
        seg = math.hypot(self.off[self.idx+1][0]-self.off[self.idx][0],
                         self.off[self.idx+1][1]-self.off[self.idx][1])
        return math.hypot(self.x - xk1, self.y - yk1) < ACCEPT_RADIUS or along >= seg

    def send_velocity(self, chi_d):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                        PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        sp.velocity.x = CRUISE_SPEED * math.cos(chi_d)
        sp.velocity.y = CRUISE_SPEED * math.sin(chi_d)
        self.sp_pub.publish(sp)

    def stop(self):
        # kecepatan NOL (bukan arah 0) -> kapal berhenti
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                        PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        sp.velocity.x = 0.0; sp.velocity.y = 0.0; sp.velocity.z = 0.0
        self.sp_pub.publish(sp)

    def ensure_guided_armed(self):
        self.stop()
        if self.tick % 10 != 0: return
        if self.state.mode != "GUIDED":
            r = SetMode.Request(); r.custom_mode = "GUIDED"; self.cli_mode.call_async(r)
        elif not self.state.armed:
            r = CommandBool.Request(); r.value = True; self.cli_arm.call_async(r)
        else:
            self.ready = True; self.get_logger().info("GUIDED + armed. Mulai mengikuti acuan.")

    def loop(self):
        self.tick += 1
        if not self.have_fix:
            self.get_logger().info("Menunggu fix GPS...", throttle_duration_sec=1.0); return
        if not self.ready:
            self.ensure_guided_armed(); return

        # fase MENUJU WP AWAL: dari posisi mana pun, kembali ke WP0 dulu
        if self.phase == "TO_START":
            d0 = math.hypot(self.x - self.off[0][0], self.y - self.off[0][1])
            if d0 < ACCEPT_RADIUS:
                self.phase = "FOLLOW"; self.idx = 0
                self.get_logger().info("Sampai di WP awal. Mulai mengikuti acuan.")
                return
            chi = math.atan2(self.off[0][1] - self.y, self.off[0][0] - self.x)
            self.send_velocity(chi)
            self.get_logger().info(f"Menuju WP awal, jarak {d0:.1f} m",
                                   throttle_duration_sec=1.0)
            return

        if self.idx >= len(self.off) - 1:
            if not self.finished:
                r = SetMode.Request(); r.custom_mode = "HOLD"; self.cli_mode.call_async(r)
                self.finished = True
                self.get_logger().info("Misi selesai. Kapal BERHENTI (mode HOLD).")
                self.summary()
            self.stop()      # kecepatan nol sebagai cadangan
            return

        chi_d, cross, along = self.step()
        des = (90.0 - math.degrees(chi_d)) % 360.0
        herr = wrap180(des - self.hdg)

        # lacak titik terdekat ke gerbang yang sedang dituju
        target = self.idx + 1
        if target in GATE_IDX:
            d = math.hypot(self.x - self.off[target][0], self.y - self.off[target][1])
            if target not in self.gate_min or d < self.gate_min[target][0]:
                self.gate_min[target] = (d, cross, herr)

        if self.at_waypoint(along):
            if target in GATE_IDX and target in self.gate_min:
                _, xte, he = self.gate_min[target]
                ok = abs(xte) <= XTE_LIMIT and abs(he) <= HDG_LIMIT
                self.gate_done[target] = (xte, he, ok)
                self.get_logger().info(
                    f">>> GERBANG WP{target}: XTE={xte:+.2f} m, hdg_err={he:+.0f} deg "
                    f"-> {'LULUS' if ok else 'GAGAL'}")
            self.idx += 1; self.y_int = 0.0
            self.get_logger().info(f"Pindah ke WP{self.idx}")
            return

        self.send_velocity(chi_d)
        at_gate = 1 if target in GATE_IDX else 0
        self.w.writerow([f"{time.time()-self.t0:.2f}", self.idx, f"{cross:.3f}",
                         f"{along:.2f}", f"{math.degrees(chi_d):.1f}",
                         f"{self.hdg:.1f}", f"{herr:.1f}", at_gate])
        self.get_logger().info(f"WP{self.idx} XTE={cross:+.2f} m hdg_err={herr:+.0f} deg",
                               throttle_duration_sec=1.0)

    def summary(self):
        self.get_logger().info("===== RINGKASAN GERBANG =====")
        passed = 0
        for g in GATE_IDX:
            if g in self.gate_done:
                xte, he, ok = self.gate_done[g]
                passed += ok
                self.get_logger().info(f"WP{g}: XTE={xte:+.2f} hdg_err={he:+.0f} "
                                       f"{'LULUS' if ok else 'GAGAL'}")
        self.get_logger().info(f"Lulus {passed}/{len(GATE_IDX)} gerbang "
                               f"(kriteria XTE<={XTE_LIMIT} m, hdg<={HDG_LIMIT} deg)")

def main():
    plan = sys.argv[1] if len(sys.argv) > 1 else "reference_with_gates.plan"
    rclpy.init()
    node = LOSGate(plan)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv.flush(); node.csv.close()
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()