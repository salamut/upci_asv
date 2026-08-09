# Simulasi Gazebo ASV (Ignition Fortress + ArduPilot SITL + MAVROS)

Simulasi fisika untuk ASV katamaran yang **tidak mengubah satu baris pun** kode
navigasi (`MAVROS_code/los_diffdrive_pid_ros2.py`). Gazebo hanya menggantikan
fisika kapal; ArduPilot SITL, MAVROS, dan node LOS+PID berjalan persis seperti
biasa (RC override RC1/RC3, mode MANUAL, skid-steer SERVO1=73 / SERVO3=74).

```
 node LOS+PID ──RC override──> MAVROS ──UDP 14552──┐
                        QGroundControl ──UDP 14550─┤ (atau TCP 5765)
                                                   ▼
                                        mavlink-routerd ──TCP 5760── ArduPilot SITL
                                        (tools/x86)                      │  backend JSON
                                                         paket servo UDP 9002 │ ▲ state JSON
                                                                         ▼  │
                                                      Gazebo Fortress (plugin AsvJsonBridge)
```

## Mengapa bukan plugin `ardupilot_gazebo` resmi?

Plugin resmi butuh Gazebo Garden/Harmonic; mesin ini memakai **Ignition
Fortress 6.16** (pasangan resmi ROS 2 Humble) dan saat pembuatan tidak ada
akses internet untuk meng-clone/instal. Sebagai gantinya, plugin kecil
`plugin/AsvJsonBridge.cc` mengimplementasikan protokol backend JSON SITL yang
sama (dokumentasi: `ardupilot/libraries/SITL/examples/JSON/readme.md`):

- menerima paket servo biner SITL di **UDP 9002** → memetakan `pwm[0]`
  (SERVO1, kiri) dan `pwm[2]` (SERVO3, kanan) menjadi gaya dorong (N) yang
  dipublish ke plugin *Thruster* bawaan Gazebo;
- membalas satu frame JSON per langkah fisika (500 Hz, lockstep): posisi/
  kecepatan NED, attitude, gyro & `accel_body` FRD — dari sinilah SITL
  mensintesis GPS, kompas, baro, dan IMU-nya sendiri (GPS SITL default nyaris
  tanpa noise ≈ RTK, jadi **tidak perlu sensor navsat/magnetometer Gazebo**);
- menghitung **hidrostatika 8 titik** (4 sudut per lambung): plugin *Buoyancy*
  bawaan Fortress tidak stabil untuk kapal permukaan (kapal terbalik karena
  momen pemulih pitch salah) sehingga gaya apung + redaman heave dihitung
  sendiri — kokoh dan bisa dituning;
- opsional: **gaya arus air konstan** (lihat *Tuning* di bawah) untuk uji ILOS.

## Isi direktori

```
gazebo_sim/
├── plugin/            AsvJsonBridge.cc + CMakeLists (build/ berisi .so hasil kompilasi)
├── models/asv_boat/   SDF katamaran 0.75x0.45 m, 2 thruster, hidrodinamika
├── worlds/asv_course.sdf  air + 12 buoy (merah kiri/hijau kanan) + origin geodetik WP0
├── config/gazebo-boat.parm  parameter ArduPilot utk simulasi
├── run_sim.sh         peluncur (a) Gazebo (b) SITL (c) MAVROS
└── tools/             test_bridge.py (uji tanpa SITL), get_pose.py
```

## Dependensi

Sudah terpasang di mesin ini (tidak ada instalasi baru):
Ubuntu 22.04, ROS 2 Humble, `ros-humble-mavros`, Ignition Fortress
(`libignition-gazebo6`), ArduPilot di `~/simulation/ardupilot` (binary
`build/sitl/bin/ardurover` + `sim_vehicle.py`), pymavlink, dan
`mavlink-routerd` di `../tools/x86/` (override lokasi: env `MAVLINK_ROUTERD`).

Build ulang plugin (hanya bila `AsvJsonBridge.cc` diubah):

```bash
cd gazebo_sim/plugin/build && cmake .. && make
```

## Cara menjalankan

```bash
cd ~/asv_upci/gazebo_sim
./run_sim.sh                 # GUI Gazebo; HEADLESS=1 ./run_sim.sh utk server saja
```

Tunggu ±30 s (EKF3 align + GPS origin), verifikasi:

```bash
source /opt/ros/humble/setup.bash          # setup.zsh bila shell zsh!
ros2 topic echo /mavros/global_position/global --once   # lat/lon ≈ -6.9729/107.6303
ros2 topic echo /mavros/global_position/compass_hdg --once  # ≈187 saat spawn
```

Lalu jalankan node navigasi **tanpa modifikasi** di terminal lain:

```bash
cd ~/asv_upci/MAVROS_code
source /opt/ros/humble/setup.bash
python3 los_diffdrive_pid_ros2.py reference_with_gates.plan
```

Log CSV tetap ditulis ke `/tmp/los_diffdrive_log.csv` seperti biasa.
Dashboard `web_monitoring/web_monitor.py` juga jalan tanpa perubahan (ia hanya
membaca topic MAVROS).

Uji fisika tanpa SITL (mengirim paket servo palsu langsung ke jembatan):

```bash
python3 tools/test_bridge.py 1590 1590 20   # pwm kiri, pwm kanan, durasi s
```

## Peta port (ubah di run_sim.sh)

| Port | Fungsi |
|---|---|
| UDP 9002 | SITL → jembatan Gazebo (paket servo; JSON dibalas ke port asal) |
| TCP 5760 | SITL serial0 ← `mavlink-routerd -p` (klien TCP) |
| TCP 5765 | server TCP router (`-t`) — QGC: Comm Link **TCP** 127.0.0.1:5765 |
| UDP 14550 | endpoint router (`-e`) → **QGroundControl** (autoconnect UDP bawaan QGC — cukup buka QGC, tanpa konfigurasi) |
| UDP 14552 | endpoint router (`-e`) → MAVROS (`fcu_url:=udp://:14552@`) |

MAVLink dirutekan `tools/x86/mavlink-routerd` (bukan MAVProxy):

```bash
./mavlink-routerd -p 127.0.0.1:5760 -t 5765 -e 127.0.0.1:14550 -e 127.0.0.1:14552
```

SITL dijalankan `sim_vehicle.py ... --no-mavproxy`. QGC memakai 14550 (bukan
MAVROS) karena autoconnect bawaan QGC hanya mendengarkan port itu — link
kustom dari file .ini terbukti tidak ikut autoconnect.

**Penting (laju telemetri):** dulu MAVProxy diam-diam meminta stream 4 Hz.
mavlink-router murni merutekan, tidak meminta apa-apa → ArduPilot hanya
mengirim 1 Hz dan node LOS membaca heading basi (kemudi berosilasi, hdg_err
belasan derajat). `run_sim.sh` karena itu memanggil
`/mavros/set_stream_rate {stream_id: 0, message_rate: 10, on_off: true}`
setelah MAVROS hidup (param `SR0_*` tidak mempan di ArduPilot 4.8-dev).

## Parameter fisika yang boleh dituning

Semua di `models/asv_boat/model.sdf` (kode navigasi JANGAN diubah):

| Parameter | Lokasi | Efek | Nilai kini |
|---|---|---|---|
| `<mass>` | `base_link/inertial` | massa kapal (4–6 kg) | 5.0 kg |
| `<max_thrust>` | plugin AsvJsonBridge | N per motor pada PWM penuh (±400 µs). Menentukan kecepatan jelajah & otoritas belok: PWM 1590 = 22.5 % → 1.575 N/motor | 7.0 N |
| `xUabsU` | plugin Hydrodynamics | drag kuadratik maju. Kecepatan tunak: `v = √(2·T/|xUabsU|)`. Kini: √(2·1.575/6.0) ≈ **0.72 m/s** pada PWM 1590 | −6.0 |
| `nRabsR` | plugin Hydrodynamics | redaman yaw — besarkan bila kapal berputar-putar/oversteer; terlalu kecil → heading berosilasi di tikungan (teruji: −0.45 → hdg_err 19°) | −0.55 |
| `yVabsV` | plugin Hydrodynamics | drag menyamping (sway) — "cengkeraman" lateral di tikungan | −60 |
| `kPabsP`, `mQabsQ` | plugin Hydrodynamics | redaman roll/pitch | −5 / −8 |
| `<heave_damping_per_point>` | plugin AsvJsonBridge | redaman naik-turun (8 titik) | 15 N·s/m |
| `<current_force_x/y>` | plugin AsvJsonBridge | **arus air konstan** (N, kerangka dunia ENU; x=Timur y=Utara). Default 0. Contoh: `<current_force_y>1.5</current_force_y>` = arus dorong ke utara — untuk menguji ILOS | 0 |

Catatan: koefisien *added mass* (`xDotU` … `nDotR`) sengaja **0** — implementasi
added mass plugin Hydrodynamics Fortress tidak stabil (fisika meledak → crash
ODE). Jangan diisi kecuali pindah ke Gazebo Garden/Harmonic.

Parameter ArduPilot di `config/gazebo-boat.parm`. Yang penting:
`MAV_GCS_SYSID=1` (alias lama `SYSID_MYGCS`) — **tanpa ini RC override dari
MAVROS diabaikan** dan kapal diam; `ARMING_CHECK=0` hanya untuk simulasi.

## Hasil verifikasi (28 Jul 2026)

- Mengapung stabil: draft 3.4 cm, roll/pitch ±0°, heading spawn 187°.
- `global_position/global` = −6.97296…/107.63038…; `compass_hdg` mengikuti putaran kapal.
- Jelajah PWM 1590: **0.72 m/s** (sasaran 0.7–1.0); belok diferensial >55°/s,
  tidak ada perilaku berputar-putar di tikungan.
- `los_diffdrive_pid_ros2.py reference_with_gates.plan` **tanpa modifikasi**,
  headless end-to-end: kapal melewati SEMUA 6 pasang buoy (XTE maks 0.34 m,
  setengah lebar gerbang 0.75 m), CSV tertulis normal, berhenti HOLD di akhir.
- Skor kriteria ketat node (XTE≤0.3 m & hdg≤10°): **5/6 gerbang LULUS**;
  satu-satunya yang gagal WP4 (XTE −0.34 m) — gerbang tepat setelah tikungan
  tertajam lintasan. Riwayat tuning: 3/6 → 4/6 → 5/6; kombinasi `nRabsR=−0.45`
  memperbaiki XTE tapi bikin heading berosilasi (19°), jadi dikembalikan.
