# ASV UPCI — Autonomous Surface Vehicle

Kode, simulasi, dan alat bantu untuk **kapal permukaan otonom (ASV) katamaran**
berbasis **ArduPilot (ArduRover / frame boat) + MAVROS + ROS 2 Humble**.

Repositori ini berisi seluruh rantai kerja: mulai dari node navigasi
*Line-of-Sight* (LOS) + PID differential-drive, simulasi fisika Gazebo yang
menggantikan kapal sungguhan tanpa mengubah satu baris pun kode navigasi,
deteksi objek YOLO + penghindaran rintangan, sampai dashboard telemetri web dan
skrip pengolahan data GPS RTK.

---

## Daftar Isi

- [Arsitektur Sistem](#arsitektur-sistem)
- [Struktur Repositori](#struktur-repositori)
- [Prasyarat](#prasyarat)
- [Instalasi](#instalasi)
- [Cara Menjalankan](#cara-menjalankan)
  - [1. Simulasi Gazebo (rekomendasi)](#1-simulasi-gazebo-rekomendasi)
  - [2. Node navigasi LOS + PID](#2-node-navigasi-los--pid)
  - [3. Dashboard web monitoring](#3-dashboard-web-monitoring)
  - [4. Workspace ROS 2 (YOLO + avoidance)](#4-workspace-ros-2-yolo--avoidance)
  - [5. Kontrol manual & uji aktuator](#5-kontrol-manual--uji-aktuator)
- [Konfigurasi Penting ArduPilot](#konfigurasi-penting-ardupilot)
- [Pengolahan Data GPS](#pengolahan-data-gps)
- [Hasil Verifikasi](#hasil-verifikasi)
- [Troubleshooting](#troubleshooting)

---

## Arsitektur Sistem

Kapal dikendalikan lewat **RC override** (RC1 = kemudi, RC3 = gas) pada mode
`MANUAL`. ArduPilot dengan konfigurasi *skid-steer* mencampurnya menjadi motor
kiri/kanan (`SERVO1_FUNCTION=73` ThrottleLeft, `SERVO3_FUNCTION=74`
ThrottleRight). Node navigasi karena itu identik antara simulasi dan lapangan —
yang berganti hanya sumber fisikanya.

```
   ┌──────────────────────┐        ┌──────────────────────┐
   │  los_diffdrive_pid   │        │  yolo_detector +     │
   │  (LOS guidance +     │        │  obstacle_avoidance  │
   │   PID diff-drive)    │        │  (ros_yolo/)         │
   └──────────┬───────────┘        └──────────┬───────────┘
              │ OverrideRCIn                  │ setpoint_velocity
              └───────────────┬───────────────┘
                              ▼
                     ┌──────────────────┐       ┌────────────────────┐
                     │      MAVROS      │◄─────►│  web_monitoring    │
                     │ (udp://:14552@)  │       │ (Flask + SocketIO) │
                     └────────┬─────────┘       └────────────────────┘
                              │ MAVLink
                     ┌────────▼─────────┐
                     │  mavlink-routerd │──UDP 14550──► QGroundControl
                     └────────┬─────────┘
                              │ TCP 5760
              ┌───────────────▼─────────────────┐
              │       ArduPilot (SITL)          │ ← atau Pixhawk sungguhan
              └───────────────┬─────────────────┘
                              │ backend JSON (UDP 9002)
              ┌───────────────▼─────────────────┐
              │ Gazebo Fortress + AsvJsonBridge │
              │ (fisika kapal katamaran)        │
              └─────────────────────────────────┘
```

---

## Struktur Repositori

| Direktori | Isi |
|---|---|
| **`MAVROS_code/`** | Node navigasi utama. `los_diffdrive_pid_ros2.py` (LOS + PID diff-drive, produksi), `los_gate_guidance_ros2.py` (LOS mode GUIDED, sadar-gerbang), `teleop_keyboard_ros2.py` (teleop keyboard), `servo8_rc7_ros2.py` (kontrol servo MAIN OUT 8), misi `.plan`, dan [`PENJELASAN_los_diffdrive_pid.md`](MAVROS_code/PENJELASAN_los_diffdrive_pid.md) — dokumentasi teknis lengkap berikut turunan rumusnya. |
| **`gazebo_sim/`** | Simulasi fisika Ignition Fortress: plugin `AsvJsonBridge` (jembatan protokol JSON SITL ↔ Gazebo), model SDF katamaran, world berisi 12 buoy gerbang, `run_sim.sh`. Lihat [`gazebo_sim/README.md`](gazebo_sim/README.md). |
| **`ros_yolo/`** | Workspace colcon ROS 2: `yolo_detector`, `obstacle_avoidance`, `mav_interface`, `waypoint_speed_control`, `asv_launch`. |
| **`ros/`** | Workspace ROS 2 generasi awal: `keyboard_control`, `object_detection` (YOLOv8), `web_monitoring`, `webrtc_publisher`. |
| **`web_monitoring/`** | Dashboard telemetri Flask + Socket.IO (posisi, mode, armed, kecepatan, heading dari topic MAVROS). |
| **`sitl/`, `sitl-arm/`** | Binary ArduRover SITL untuk x86-64 dan arm64 beserta parameternya. |
| **`tools/`** | `mavlink-routerd` (x86 & arm64) untuk merutekan MAVLink ke QGC + MAVROS sekaligus. |
| **`mavlink/`** | Pustaka MAVLink + pymavlink. |
| **`example/`** | Contoh mandiri: pub/sub ROS, pesan MAVLink, streaming video realtime, WebRTC/Cloudflare, monitoring dasar. |
| **`data gps/`, `*.txt`, `*.html`** | Data mentah GPS RTK, hasil parsing, dan peta trajektori. |

---

## Prasyarat

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop`)
- `ros-humble-mavros` + `ros-humble-mavros-extras` (+ dataset GeographicLib)
- Ignition Gazebo **Fortress** (`libignition-gazebo6`) — hanya untuk simulasi
- ArduPilot SITL (`sim_vehicle.py` + `build/sitl/bin/ardurover`)
- Python 3.10, `pymavlink`, `flask`, `flask-socketio`, `ultralytics` (YOLO)

## Instalasi

```bash
git clone https://github.com/salamut/upci_asv.git
cd upci_asv
./setup.sh          # ROS 2 Humble + MAVROS + GeographicLib + venv Python
```

`setup.sh` memasang dependensi sistem, ROS 2, MAVROS, lalu membuat virtualenv
`venv/` berisi numpy/scipy/pyserial/opencv/matplotlib.

> **Catatan shell:** jika memakai **zsh**, gunakan `setup.zsh` — bukan
> `setup.bash` — di setiap perintah `source`.

---

## Cara Menjalankan

### 1. Simulasi Gazebo (rekomendasi)

Menjalankan Gazebo + ArduPilot SITL + mavlink-router + MAVROS sekaligus:

```bash
cd gazebo_sim
./run_sim.sh                 # GUI Gazebo
HEADLESS=1 ./run_sim.sh      # tanpa GUI (lebih ringan)
```

Tunggu ±30 detik (EKF3 align + GPS origin), lalu verifikasi:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/global_position/global --once      # lat/lon ≈ -6.9729 / 107.6303
ros2 topic echo /mavros/global_position/compass_hdg --once # ≈ 187° saat spawn
```

Peta port: UDP 9002 (SITL ↔ Gazebo), TCP 5760 (SITL serial0), TCP 5765 (server
router untuk QGC), UDP 14550 (QGroundControl), UDP 14552 (MAVROS).

Uji fisika tanpa SITL:

```bash
python3 gazebo_sim/tools/test_bridge.py 1590 1590 20   # pwm kiri, pwm kanan, durasi (s)
```

### 2. Node navigasi LOS + PID

Berjalan **identik** di simulasi maupun di kapal sungguhan:

```bash
cd MAVROS_code
source /opt/ros/humble/setup.bash
python3 los_diffdrive_pid_ros2.py reference_with_gates.plan
```

Node akan: membaca misi `.plan`, menuju waypoint awal, mengikuti lintasan dengan
LOS + PID atas error sudut, menilai tiap **gerbang** terhadap kriteria
`XTE ≤ 0,30 m` dan `heading error ≤ 10°`, lalu berhenti aman di titik akhir.
Log CSV ditulis ke `/tmp/los_diffdrive_log.csv`.

Parameter tuning utama ada di bagian atas berkas:

| Parameter | Arti | Nilai |
|---|---|---|
| `THROTTLE_PWM` | gas dasar (PWM) | 1590 |
| `KP` / `KI` / `KD` | gain PID atas error sudut (PWM per derajat) | 4.0 / 0.2 / 4.0 |
| `STEER_MAX` | batas simpang kemudi | 300 |
| `LOOKAHEAD` | jarak lookahead LOS (m) | 2.5 |
| `ACCEPT_RADIUS` | radius capai waypoint (m) | 0.65 |
| `USE_COG` | umpan balik pakai COG (GPS) alih-alih kompas | `False` |
| `STEER_SIGN` | balik ke `-1` bila kapal berbelok ke arah salah | 1 |

Alternatif berbasis mode `GUIDED` (tanpa RC override):

```bash
python3 los_gate_guidance_ros2.py reference_with_gates.plan
```

### 3. Dashboard web monitoring

```bash
source /opt/ros/humble/setup.bash
python3 web_monitoring/web_monitor.py
# buka http://localhost:5000
```

Menampilkan lintang/bujur, mode, status armed, kecepatan, dan heading kompas
secara realtime lewat Socket.IO. Berjalan tanpa perubahan baik di simulasi
maupun di lapangan karena hanya membaca topic MAVROS.

### 4. Workspace ROS 2 (YOLO + avoidance)

```bash
cd ros_yolo
colcon build
source install/setup.bash          # setup.zsh bila memakai zsh
ros2 launch asv_launch asv_system.launch.py
ros2 launch asv_launch asv_system.launch.py mission_master:=false   # tanpa mission master
```

Node yang dijalankan:

| Node | Paket | Fungsi |
|---|---|---|
| `yolo_detector` | `yolo_detector` | Publish deteksi buoy sebagai JSON ke `/yolo/detections` |
| `avoidance` | `obstacle_avoidance` | Peralihan halus AUTO ↔ GUIDED + perintah kecepatan saat buoy terdeteksi |
| `mav_monitor` | `mav_interface` | Cetak mode & status armed dari `/mavros/state` |
| `mission_master` | `mav_interface` | Orkestrasi misi |
| `waypoint_speed_node` | `waypoint_speed_control` | Ubah kecepatan jelajah per waypoint via `MAV_CMD_DO_CHANGE_SPEED` |

### 5. Kontrol manual & uji aktuator

```bash
python3 MAVROS_code/teleop_keyboard_ros2.py   # w/s/a/d, spasi = stop, q = keluar
python3 MAVROS_code/servo8_rc7_ros2.py        # servo 0-180° di MAIN OUT 8 lewat RC ch7
```

---

## Konfigurasi Penting ArduPilot

| Parameter | Nilai | Alasan |
|---|---|---|
| `MAV_GCS_SYSID` | `1` | **Wajib.** Tanpa ini RC override dari MAVROS diabaikan dan kapal diam. (Nama lama: `SYSID_MYGCS`, ArduPilot < 4.6) |
| `SERVO1_FUNCTION` | `73` | ThrottleLeft (skid-steer) |
| `SERVO3_FUNCTION` | `74` | ThrottleRight (skid-steer) |
| `SERVO8_FUNCTION` | `146` | RCIN7 *mapped* — menskalakan 1000–2000 µs ke `SERVO8_MIN..MAX` (500–2500 µs) sehingga servo mencapai 180° penuh. Fungsi `57` adalah passthrough mentah dan mengabaikan MIN/MAX/REVERSED. |
| `ARMING_CHECK` | `0` | **Hanya untuk simulasi.** |

Parameter simulasi lengkap: [`gazebo_sim/config/gazebo-boat.parm`](gazebo_sim/config/gazebo-boat.parm).

---

## Pengolahan Data GPS

```bash
python3 gps_parser.py    # datartk.txt      -> gps_data.csv
python3 gpsdata.py       # gngga_only.txt   -> gps_data.csv
python3 panda.py         # gps_data.csv     -> gps_map.html (peta Folium)
```

Data mentah RTK (`gpsrtkdatabuatTA.txt`, `datartk.txt`, `gngga_only.txt`), hasil
tangkapan lapangan (`data gps/`), dan peta trajektori (`gps_map.html`,
`gps_avg_lap_autopilot.html`) disertakan.

---

## Hasil Verifikasi

Uji end-to-end di simulasi Gazebo (28 Juli 2026):

- Kapal mengapung stabil — draft 3,4 cm, roll/pitch ±0°.
- Kecepatan jelajah **0,72 m/s** pada PWM 1590 (target 0,7–1,0 m/s).
- Laju belok diferensial > 55°/s, tanpa perilaku berputar-putar di tikungan.
- `los_diffdrive_pid_ros2.py` **tanpa modifikasi** melewati **seluruh 6 pasang
  buoy**; XTE maksimum 0,34 m terhadap setengah lebar gerbang 0,75 m.
- Terhadap kriteria ketat (XTE ≤ 0,30 m **dan** heading error ≤ 10°):
  **5 dari 6 gerbang LULUS**. Satu-satunya kegagalan di WP4, gerbang tepat
  setelah tikungan tertajam lintasan.

---

## Troubleshooting

**Kapal diam meski node mengirim RC override.**
`MAV_GCS_SYSID` belum di-set ke system-id MAVROS (biasanya `1`).

**Kemudi berosilasi, heading error belasan derajat.**
Laju stream telemetri terlalu rendah. mavlink-router hanya merutekan dan tidak
meminta stream (berbeda dengan MAVProxy yang diam-diam meminta 4 Hz), sehingga
ArduPilot hanya mengirim 1 Hz dan node membaca heading basi. Solusinya sudah
otomatis di `run_sim.sh`:

```bash
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 10, on_off: true}"
```

**Kapal berbelok ke arah yang salah.** Balik `STEER_SIGN` menjadi `-1`.

**Fisika Gazebo "meledak" / ODE crash.** Koefisien *added mass*
(`xDotU` … `nDotR`) harus tetap `0` — implementasinya di plugin Hydrodynamics
Fortress tidak stabil. Jangan diisi kecuali pindah ke Gazebo Garden/Harmonic.

**Kapal terbalik di air.** Plugin `Buoyancy` bawaan Fortress salah menghitung
momen pemulih pitch untuk kapal permukaan; `AsvJsonBridge` karena itu menghitung
hidrostatika 8 titik sendiri (4 sudut per lambung).

**`colcon build --symlink-install` gagal.** Konflik setuptools ≥ 82 dengan
colcon; bangun tanpa `--symlink-install`.

---

## Catatan

- Artefak runtime (`logs/`, `*.BIN`, `*.tlog`, `venv/`, `build/`, `install/`)
  sengaja tidak dilacak git — lihat [`.gitignore`](.gitignore).
- Paket-paket di `ros/` menyertakan berkas `LICENSE` (Apache-2.0)
  masing-masing.
