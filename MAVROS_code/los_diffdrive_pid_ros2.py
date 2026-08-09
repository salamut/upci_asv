#!/usr/bin/env python3
"""
Guidance LOS + KENDALI PID DIFFERENTIAL DRIVE untuk ArduRover/boat (MAVROS / ROS2).

Arsitektur:
  - GUIDANCE (tidak berubah): LOS menghitung arah yang diinginkan chi_d dari
    cross-track error dan lookahead.
  - CONTROL (ini yang baru): PID atas ERROR SUDUT (theta) menghasilkan perintah
    belok, lalu dicampur secara DIFERENSIAL ke motor kiri/kanan.

Output dikirim sebagai steering(RC1) + throttle(RC3) via RC override (mode MANUAL).
ArduPilot dalam konfigurasi skid-steer akan mencampurnya menjadi motor kiri/kanan:
    SERVO1_FUNCTION = 73 (ThrottleLeft), SERVO3_FUNCTION = 74 (ThrottleRight)
(throttle dipatok pada base agar uji aman; hanya steering yang dikendalikan PID).

Umpan balik sudut bisa pakai HEADING (kompas) atau COG (arah gerak nyata dari GPS),
dipilih lewat USE_COG. Saat kecepatan < MIN_SPEED_COG, otomatis kembali ke heading.

Tetap: baca .plan, fase menuju WP awal, pindah waypoint, catat XTE & heading error
di tiap gerbang + LULUS/GAGAL, berhenti aman di akhir.

Pakai:  python3 los_diffdrive_pid_ros2.py reference_with_gates.plan
"""

import sys, math, csv, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

# ---------- gas / motor (PWM) ----------
THROTTLE_PWM   = 1590      # dorongan maju dasar (base). Naikkan hati-hati bila perlu.
THROTTLE_STOP  = 1500      # netral / berhenti

# ---------- kemudi (PWM) ----------
STEER_NEUTRAL  = 1500
STEER_MAX      = 300       # batas simpang kemudi (diperkecil agar belok lebih lembut)
STEER_SIGN     = 1         # BALIK ke -1 bila kapal berbelok ke arah yang salah

# ---------- PID atas error sudut (deg) -> satuan PWM ----------
KP             = 4.0       # PWM per derajat error (DITURUNKAN dari 8: belok lebih lembut)
KI             = 0.2       # KECIL: SITL tak ber-arus, KI besar -> overshoot. Naikkan di lapangan bila ada arus.
KD             = 4.0       # dinaikkan: peredam laju-yaw lebih kuat melawan ayunan
I_LIMIT        = STEER_MAX # batas suku integral (anti-windup) dalam satuan PWM
DERIV_LP       = 0.3       # koefisien filter low-pass turunan (0..1; kecil = lebih halus)

# ---------- pelembut (anti belok mendadak) ----------
TURN_RATE_MAX  = 120.0      # deg/s: batas laju sapuan HEADING TUJUAN (pelembut setpoint)
STEER_SLEW     = 800.0     # PWM/s: dilonggarkan -- kelembutan kini dari gain, bukan rate-limit

# ---------- umpan balik arah ----------
USE_COG        = False     # True = pakai COG (arah gerak GPS); False = heading kompas
MIN_SPEED_COG  = 0.3       # m/s; di bawah ini COG tidak andal -> pakai heading

# ---------- guidance ----------
LOOKAHEAD      = 2.5
ACCEPT_RADIUS  = 0.65
FINAL_RADIUS   = 0.30      # radius capai titik AKHIR (lead-out): dihampiri langsung, bukan dilewati
FINAL_OVERSHOOT = 1.0      # bila terlanjur lewat sejauh ini (m), berhenti homing (pengaman)
LOOP_HZ        = 10.0
GATE_IDX = [1, 2, 4, 6, 8, 9]  # +2 WP perantara disisipkan antara WP7 dan WP8 (sudut kanan-bawah)
XTE_LIMIT      = 0.30
HDG_LIMIT      = 10.0
CSV_PATH       = "/tmp/los_diffdrive_log.csv"

EARTH_R = 6378137.0

def parse_plan(path):
    d = json.load(open(path))
    return [(it["params"][4], it["params"][5])
            for it in d["mission"]["items"] if it.get("command") in (16, 82)]

def ll_to_enu(lat, lon, lat0, lon0):
    return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * EARTH_R)

def wrap_pi(a): return math.atan2(math.sin(a), math.cos(a))
def wrap180(d): return (d + 180.0) % 360.0 - 180.0
def clamp(v, lo, hi): return max(lo, min(hi, v))


class DiffDrivePID(Node):
    def __init__(self, plan_path):
        super().__init__("los_diffdrive_pid")
        self.wp_ll = parse_plan(plan_path)
        l0, o0 = self.wp_ll[0]
        self.off = [ll_to_enu(la, lo, l0, o0) for la, lo in self.wp_ll]

        self.state = State(); self.hdg = 0.0
        self.vx = self.vy = 0.0                 # kecepatan ENU (untuk COG)
        self.x = self.y = 0.0; self.origin = None; self.have_fix = False
        self.idx = 0; self.ready = False; self.tick = 0
        self.phase = "TO_START"; self.finished = False
        self.gate_min = {}; self.gate_done = {}; self.start_min = float("inf")
        # state PID (turunan dihitung pada PENGUKURAN heading, bukan error)
        self.integ = 0.0; self.prev_meas = None; self.prev_t = None; self.deriv_filt = 0.0
        # state pelembut: heading tujuan ter-rate-limit & kemudi terakhir
        self.des_filt = None; self.steer_cmd = float(STEER_NEUTRAL)

        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "state", m), qos_profile_sensor_data)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 lambda m: setattr(self, "hdg", m.data), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self.cb_gps, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self.cb_vel, qos_profile_sensor_data)
        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm  = self.create_client(CommandBool, "/mavros/cmd/arming")
        for cli, n in [(self.cli_mode, "set_mode"), (self.cli_arm, "arming")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f"Service {n} belum siap")

        self.csv = open(CSV_PATH, "w", newline=""); self.w = csv.writer(self.csv)
        self.w.writerow(["t","wp_idx","xte_m","theta_err_deg","steer_pwm",
                         "thr_kiri","thr_kanan","arah_dipakai","di_gerbang"])
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / LOOP_HZ, self.loop)

    def cb_gps(self, msg):
        if self.origin is None:
            self.origin = (self.wp_ll[0][0], self.wp_ll[0][1])   # koordinat absolut
        self.x, self.y = ll_to_enu(msg.latitude, msg.longitude, *self.origin)
        self.have_fix = True

    def cb_vel(self, msg):
        self.vx = msg.twist.linear.x; self.vy = msg.twist.linear.y

    # ---- arah aktual yang dipakai: heading atau COG ----
    def current_dir(self):
        spd = math.hypot(self.vx, self.vy)
        if USE_COG and spd >= MIN_SPEED_COG:
            cog_enu = math.atan2(self.vy, self.vx)            # rad, dari Timur
            return (90.0 - math.degrees(cog_enu)) % 360.0, "COG"
        return self.hdg, "HDG"

    # ---- LOS guidance ----
    def los_course(self):
        xk, yk = self.off[self.idx]; xk1, yk1 = self.off[self.idx + 1]
        alpha = math.atan2(yk1 - yk, xk1 - xk)
        dx, dy = self.x - xk, self.y - yk
        along =  dx * math.cos(alpha) + dy * math.sin(alpha)
        cross = -dx * math.sin(alpha) + dy * math.cos(alpha)
        chi_d = wrap_pi(alpha + math.atan2(-cross, LOOKAHEAD))
        seg = math.hypot(xk1 - xk, yk1 - yk)
        return chi_d, cross, along, seg

    # ---- PID atas error sudut -> steering PWM + campuran diferensial ----
    def pid_to_rc(self, des_compass):
        cur, src = self.current_dir()
        now = time.time()
        dt = max(now - self.prev_t, 1e-3) if self.prev_t is not None else (1.0 / LOOP_HZ)

        # 1) PELEMBUT SETPOINT: batasi laju sapuan heading tujuan (deg/s).
        #    Loncatan chi_d saat transisi/koreksi besar diubah jadi sapuan bertahap,
        #    sehingga error tidak meloncat -> suku P tidak menyentak.
        if self.des_filt is None:
            self.des_filt = des_compass
        else:
            step = clamp(wrap180(des_compass - self.des_filt),
                         -TURN_RATE_MAX * dt, TURN_RATE_MAX * dt)
            self.des_filt = (self.des_filt + step) % 360.0
        err = wrap180(self.des_filt - cur)

        # 2) Turunan pada PENGUKURAN (heading), bukan error -> kebal loncatan setpoint.
        if self.prev_t is None or self.prev_meas is None:
            deriv = 0.0                     # tick pertama: paksa turunan nol
        else:
            dpsi = wrap180(cur - self.prev_meas)        # perubahan heading (di-wrap)
            raw = -dpsi / dt
            self.deriv_filt = (1 - DERIV_LP) * self.deriv_filt + DERIV_LP * raw  # low-pass
            deriv = self.deriv_filt

        u_unsat = KP * err + KI * self.integ + KD * deriv
        # anti-windup: integrasi hanya bila keluaran belum jenuh
        if self.prev_t is not None and abs(u_unsat) < STEER_MAX:
            self.integ = clamp(self.integ + err * dt, -I_LIMIT / max(KI, 1e-6),
                               I_LIMIT / max(KI, 1e-6))
        u = KP * err + KI * self.integ + KD * deriv
        self.prev_meas = cur; self.prev_t = now

        steer_raw = clamp(STEER_NEUTRAL + STEER_SIGN * u, STEER_NEUTRAL - STEER_MAX,
                          STEER_NEUTRAL + STEER_MAX)
        # 3) PELEMBUT KELUARAN: batasi laju perubahan kemudi (slew-rate, PWM/detik).
        dmax = STEER_SLEW * dt
        steer = self.steer_cmd + clamp(steer_raw - self.steer_cmd, -dmax, dmax)
        self.steer_cmd = steer

        # campuran diferensial (untuk pencatatan; ArduPilot yang mencampur sebenarnya)
        turn = (steer - STEER_NEUTRAL) / STEER_MAX            # -1..1
        MAX_DIFF = 150 
        diff = turn * MAX_DIFF
        kiri  = int(clamp(THROTTLE_PWM - diff, 1300, 1700))
        kanan = int(clamp(THROTTLE_PWM + diff, 1300, 1700))
        return int(steer), err, src, kiri, kanan

    def send_rc(self, steer, thr):
        m = OverrideRCIn()
        ch = [0] * 18
        ch[0] = int(steer)   # RC1 = steering
        ch[2] = int(thr)     # RC3 = throttle (base)
        m.channels = ch
        self.rc_pub.publish(m)

    def reset_pid(self):
        # Hanya integral yang direset agar bias segmen lama tak terbawa.
        # State turunan & pelembut (des_filt, steer_cmd) DIBIARKAN kontinu supaya
        # transisi waypoint tetap mulus -- tidak ada loncatan kemudi.
        self.integ = 0.0

    def ensure_manual_armed(self):
        self.send_rc(STEER_NEUTRAL, THROTTLE_STOP)
        if self.tick % 10 != 0: return
        if self.state.mode != "MANUAL":
            r = SetMode.Request(); r.custom_mode = "MANUAL"; self.cli_mode.call_async(r)
        elif not self.state.armed:
            r = CommandBool.Request(); r.value = True; self.cli_arm.call_async(r)
        else:
            self.ready = True
            self.get_logger().info("MANUAL + armed. PID differential drive aktif.")

    def loop(self):
        self.tick += 1
        if not self.have_fix:
            self.get_logger().info("Menunggu fix GPS...", throttle_duration_sec=1.0); return
        if not self.ready:
            self.ensure_manual_armed(); return

        # fase menuju WP awal (lead-in): hampiri WP0, lalu mulai begitu kapal MELEWATINYA
        if self.phase == "TO_START":
            d0 = math.hypot(self.x - self.off[0][0], self.y - self.off[0][1])
            # proyeksi posisi pada arah segmen pertama (WP0->WP1); >=0 berarti sudah melewati WP0
            sdir = math.atan2(self.off[1][1] - self.off[0][1], self.off[1][0] - self.off[0][0])
            s0 = (self.x - self.off[0][0]) * math.cos(sdir) + (self.y - self.off[0][1]) * math.sin(sdir)
            if d0 < ACCEPT_RADIUS or s0 >= 0.0:
                self.phase = "FOLLOW"; self.idx = 0; self.reset_pid()
                self.get_logger().info(f"Melewati WP awal (jarak {d0:.2f} m). Mulai mengikuti acuan.")
                return
            bearing = math.atan2(self.off[0][1] - self.y, self.off[0][0] - self.x)
            des = (90.0 - math.degrees(bearing)) % 360.0
            steer, _, _, _, _ = self.pid_to_rc(des)
            self.send_rc(steer, THROTTLE_PWM)
            self.get_logger().info(f"Menuju WP awal, jarak {d0:.1f} m", throttle_duration_sec=1.0)
            return

        if self.idx >= len(self.off) - 1:
            if not self.finished:
                r = SetMode.Request(); r.custom_mode = "HOLD"; self.cli_mode.call_async(r)
                self.finished = True
                self.get_logger().info("Misi selesai. Kapal BERHENTI (HOLD).")
                self.summary()
            self.send_rc(STEER_NEUTRAL, THROTTLE_STOP)
            return

        chi_d, cross, along, seg = self.los_course()
        target = self.idx + 1
        is_final = (target == len(self.off) - 1)   # segmen lead-out menuju titik AKHIR

        # LOS untuk SEMUA segmen (termasuk lead-out). Tidak ada lagi "homing" ke titik
        # tetap -> kapal tidak berputar-putar mengejar satu titik di air nyata.
        des = (90.0 - math.degrees(chi_d)) % 360.0
        steer, terr, src, kiri, kanan = self.pid_to_rc(des)

        # heading error untuk kriteria gerbang (selalu pakai heading lambung)
        herr = wrap180(des - self.hdg)
        if target in GATE_IDX:
            d = math.hypot(self.x - self.off[target][0], self.y - self.off[target][1])
            if target not in self.gate_min or d < self.gate_min[target][0]:
                self.gate_min[target] = (d, cross, herr)

        dist_t = math.hypot(self.x - self.off[target][0], self.y - self.off[target][1])
        radius = FINAL_RADIUS if is_final else ACCEPT_RADIUS
        # 'along >= seg' = kapal sudah MELEWATI garis tegak lurus di waypoint. Syarat ini
        # selalu tercapai oleh kapal yang bergerak, sehingga mencegah putar-putar (orbit).
        switch = dist_t < radius or along >= seg

        if switch:
            if target in GATE_IDX and target in self.gate_min:
                _, xte, he = self.gate_min[target]
                ok = abs(xte) <= XTE_LIMIT and abs(he) <= HDG_LIMIT
                self.gate_done[target] = (xte, he, ok)
                self.get_logger().info(
                    f">>> GERBANG WP{target}: XTE={xte:+.2f} m, hdg_err={he:+.0f} deg "
                    f"-> {'LULUS' if ok else 'GAGAL'}")
            if is_final:
                self.get_logger().info(f"Mencapai titik akhir WP{target} (jarak {dist_t:.2f} m).")
            self.idx += 1; self.reset_pid()
            self.get_logger().info(f"Pindah ke WP{self.idx}")
            return

        self.send_rc(steer, THROTTLE_PWM)
        self.w.writerow([f"{time.time()-self.t0:.2f}", self.idx, f"{cross:.3f}",
                         f"{terr:.1f}", steer, kiri, kanan, src,
                         1 if target in GATE_IDX else 0])
        self.get_logger().info(
            f"WP{self.idx} XTE={cross:+.2f} m err={terr:+.0f} steer={steer} "
            f"L/R={kiri}/{kanan} [{src}]", throttle_duration_sec=1.0)

    def summary(self):
        self.get_logger().info("===== RINGKASAN GERBANG =====")
        passed = 0
        for g in GATE_IDX:
            if g in self.gate_done:
                xte, he, ok = self.gate_done[g]; passed += ok
                self.get_logger().info(f"WP{g}: XTE={xte:+.2f} hdg_err={he:+.0f} "
                                       f"{'LULUS' if ok else 'GAGAL'}")
        self.get_logger().info(f"Lulus {passed}/{len(GATE_IDX)} gerbang "
                               f"(XTE<={XTE_LIMIT} m, hdg<={HDG_LIMIT} deg)")

def main():
    plan = sys.argv[1] if len(sys.argv) > 1 else "reference_with_gates.plan"
    rclpy.init(); node = DiffDrivePID(plan)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.send_rc(STEER_NEUTRAL, THROTTLE_STOP)
        node.csv.flush(); node.csv.close()
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()