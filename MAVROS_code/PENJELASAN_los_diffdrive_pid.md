# Penjelasan `los_diffdrive_pid_ros2.py`

Dokumentasi teknis node ROS 2 untuk **guidance LOS + kendali PID differential drive**
pada ArduRover/boat via MAVROS.

Berkas sumber: [`los_diffdrive_pid_ros2.py`](los_diffdrive_pid_ros2.py)

---

## Daftar Isi

1. [Gambaran Besar](#1-gambaran-besar)
2. [Persiapan Data: `.plan` dan Konversi Koordinat](#2-persiapan-data-plan-dan-konversi-koordinat)
3. [Guidance: Line-of-Sight (LOS)](#3-guidance-line-of-sight-los)
4. [Konversi Sudut ENU ↔ Kompas](#4-konversi-sudut-enu--kompas)
5. [Umpan Balik Arah: HDG vs COG](#5-umpan-balik-arah-hdg-vs-cog)
6. [Kendali PID](#6-kendali-pid)
7. [Antarmuka Aktuator (RC Override)](#7-antarmuka-aktuator-rc-override)
8. [Mesin Status `loop()`](#8-mesin-status-loop)
9. [Penilaian Gerbang & Logging](#9-penilaian-gerbang--logging)
10. [Siklus Hidup Program](#10-siklus-hidup-program)
11. [Ringkasan Semua Rumus](#11-ringkasan-semua-rumus)
12. [Tabel Parameter Tuning](#12-tabel-parameter-tuning)
13. [Catatan & Temuan](#13-catatan--temuan)

---

## 1. Gambaran Besar

Program ini adalah **node ROS 2 tunggal** yang membuat kapal (ArduRover mode boat /
skid-steer) mengikuti jalur waypoint dari file `.plan` QGroundControl, sambil menilai
apakah kapal berhasil melewati beberapa "gerbang" dengan presisi tertentu.

Rantai kendalinya klasik **guidance → control → actuator**:

```
GPS (lat,lon) ─┐
               ├─► ENU (x,y) ─► LOS guidance ─► chi_d (arah tujuan)
waypoint .plan ┘                                    │
                                                    ▼
compass / COG ─────────────────────► error sudut ─► PID ─► steering PWM
                                                    │
                                                    ▼
                                    RC override (RC1=steer, RC3=throttle)
                                                    │
                                                    ▼
                                     ArduPilot skid-steer mixer → motor kiri/kanan
```

**Poin penting:** throttle **dikunci konstan** di `THROTTLE_PWM = 1590`. Yang dikendalikan
PID **hanya arah (steering)**. Ini disengaja supaya uji coba aman dan hasil tuning tidak
tercampur variabel kecepatan.

Pencampuran ke motor kiri/kanan **tidak dilakukan oleh script ini** — ArduPilot yang
melakukannya, lewat konfigurasi:

| Parameter | Nilai | Arti |
|---|---|---|
| `SERVO1_FUNCTION` | 73 | ThrottleLeft |
| `SERVO3_FUNCTION` | 74 | ThrottleRight |

Script hanya mengirim "steering" + "throttle" seperti stik RC biasa.

### Antarmuka ROS

| Arah | Topik / Service | Tipe | Guna |
|---|---|---|---|
| sub | `/mavros/state` | `mavros_msgs/State` | cek mode & armed |
| sub | `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | heading kompas (deg) |
| sub | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | posisi lat/lon |
| sub | `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | kecepatan ENU (untuk COG) |
| pub | `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | perintah steering + throttle |
| srv | `/mavros/set_mode` | `mavros_msgs/SetMode` | MANUAL saat mulai, HOLD saat selesai |
| srv | `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | arming |

Semua subscription memakai `qos_profile_sensor_data` (best-effort), sesuai sifat data
telemetri yang boleh hilang sesekali.

---

## 2. Persiapan Data: `.plan` dan Konversi Koordinat

### 2.1 Parsing misi — `parse_plan()` (baris 72–75)

```python
d = json.load(open(path))
return [(it["params"][4], it["params"][5])
        for it in d["mission"]["items"] if it.get("command") in (16, 82)]
```

File `.plan` adalah JSON QGroundControl. Diambil hanya item dengan MAVLink command:

- **16** = `MAV_CMD_NAV_WAYPOINT`
- **82** = `MAV_CMD_NAV_SPLINE_WAYPOINT`

Pada standar MAVLink, `params[4]` = latitude dan `params[5]` = longitude.
Hasilnya list `[(lat, lon), ...]` berurutan.

### 2.2 Geodetik → bidang datar lokal — `ll_to_enu()` (baris 77–79)

Perhitungan cross-track error butuh koordinat kartesian (meter), bukan derajat.
Dipakai **proyeksi equirectangular** dengan titik nol di WP0:

```
x_E = (lon - lon0) * (pi/180) * R * cos(lat0)
y_N = (lat - lat0) * (pi/180) * R
```

dengan:

- `lat`/`lon` posisi, indeks `0` = titik acuan (WP pertama)
- `R = 6378137 m` (jari-jari ekuator WGS-84)
- `cos(lat0)` = faktor penyusutan garis bujur karena meridian merapat ke arah kutub

> **Konvensi frame: `x` = Timur (East), `y` = Utara (North).**
> Ini frame ENU, sama dengan konvensi ROS/MAVROS. Ingat ini — sumber kebingungan sudut
> paling sering ada di sini.

Akurasi proyeksi ini sangat baik untuk area lomba (< beberapa ratus meter); error orde
milimeter. Karena `origin` di-set ke `wp_ll[0]` (baris 127), posisi kapal dan semua
waypoint berada di frame yang sama persis.

### 2.3 Helper sudut (baris 81–83)

```python
wrap_pi(a)  = atan2(sin a, cos a)      # bungkus ke (-pi, pi]  — radian
wrap180(d)  = (d + 180) % 360 - 180    # bungkus ke (-180, 180] — derajat
clamp(v,lo,hi)                          # potong ke rentang
```

`wrap180` **wajib** ada. Tanpa itu, error dari heading 350° ke 10° akan terbaca 340°
(putar hampir satu lingkaran) padahal seharusnya cuma −20°.

---

## 3. Guidance: Line-of-Sight (LOS)

Fungsi `los_course()` (baris 143–151). Inti navigasinya: dari posisi kapal sekarang,
hitung **arah yang harus dituju** agar kapal kembali ke garis lurus antar dua waypoint.

### 3.1 Sudut jalur

```
alpha = atan2(y[k+1] - y[k], x[k+1] - x[k])
```

Sudut segmen WP*k* → WP*k+1*, dalam radian, diukur **CCW dari sumbu Timur**
(konvensi matematis ENU).

### 3.2 Rotasi ke frame jalur

Vektor posisi relatif terhadap WP*k*: `(dx, dy) = (x - x[k], y - y[k])`, diputar `-alpha`:

```
| s |   |  cos(alpha)   sin(alpha) | | dx |
|   | = |                          | |    |
| e |   | -sin(alpha)   cos(alpha) | | dy |
```

Dalam kode:

```python
along =  dx*cos(alpha) + dy*sin(alpha)     # s
cross = -dx*sin(alpha) + dy*cos(alpha)     # e
```

- **`along` (s)** = jarak sepanjang jalur dari WP*k*. Nilai 0 = tepat di WP*k*;
  nilai = panjang segmen berarti tepat di WP*k+1*.
- **`cross` (e)** = **cross-track error (XTE)**, jarak tegak lurus dari garis.
  Positif = kapal di sebelah **kiri** jalur (karena rotasi CCW).

### 3.3 Hukum LOS lookahead

```
chi_d = alpha + arctan( -e / DELTA )        DELTA = LOOKAHEAD = 2.5 m
```

Kode: `chi_d = wrap_pi(alpha + math.atan2(-cross, LOOKAHEAD))`

**Intuisi geometris:** bayangkan titik semu di jalur sejauh `DELTA` di depan proyeksi
kapal. LOS menyuruh kapal menuju titik itu.

| Kondisi | `arctan(-e/DELTA)` | Perilaku |
|---|---|---|
| `e = 0` (di jalur) | 0° | jalan sejajar jalur, `chi_d = alpha` |
| `e = +DELTA` (2.5 m di kiri) | −45° | miring 45° ke kanan, memotong balik |
| `e >> DELTA` | → −90° | tegak lurus menyerbu jalur |

Sifat penting: sudut koreksi **selalu terbatas di ±90°**, jadi kapal tidak pernah diminta
berbalik arah — inilah kenapa LOS stabil dan tidak menghasilkan orbit.

**`DELTA` adalah tuning knob utama guidance:**

- `DELTA` kecil → agresif, cepat kembali ke jalur, tapi rawan osilasi zig-zag
- `DELTA` besar → lembut, konvergen perlahan, XTE steady-state lebih lama hilang

Nilai 2.5 m cocok untuk kapal kecil dengan `XTE_LIMIT` 0.30 m.

---

## 4. Konversi Sudut ENU ↔ Kompas

Ada dua konvensi sudut yang tercampur dan harus dijembatani:

| Konvensi | Nol | Arah putar | Dipakai oleh |
|---|---|---|---|
| **ENU (matematis)** | Timur | CCW (berlawanan jarum jam) | `alpha`, `chi_d`, `atan2(vy,vx)` |
| **Kompas** | Utara | CW (searah jarum jam) | `compass_hdg`, setpoint PID |

Rumus jembatannya:

```
psi_kompas = (90 - chi_ENU_derajat) mod 360
```

Muncul di tiga tempat:

| Baris | Konteks |
|---|---|
| 139 | COG: `(90.0 - math.degrees(cog_enu)) % 360.0` |
| 245 | bearing ke WP awal (fase `TO_START`) |
| 266 | `chi_d` dari LOS: `des = (90.0 - math.degrees(chi_d)) % 360.0` |

Cek cepat: `chi_ENU = 0` (Timur) → `psi = 90` ✓ ; `chi_ENU = 90` (Utara) → `psi = 0` ✓

Semua perhitungan error PID dilakukan di **domain kompas (derajat)**, karena umpan balik
`/mavros/global_position/compass_hdg` memang dalam derajat kompas.

---

## 5. Umpan Balik Arah: HDG vs COG

Fungsi `current_dir()` (baris 135–140):

```python
spd = math.hypot(self.vx, self.vy)
if USE_COG and spd >= MIN_SPEED_COG:
    cog_enu = math.atan2(self.vy, self.vx)
    return (90.0 - math.degrees(cog_enu)) % 360.0, "COG"
return self.hdg, "HDG"
```

Dua pilihan sinyal "arah kapal":

| Sinyal | Arti | Kelebihan | Kelemahan |
|---|---|---|---|
| **HDG** (heading kompas) | ke mana **lambung menghadap** | responsif, tersedia meski diam | buta terhadap arus/angin — kapal bisa menghadap utara sambil hanyut ke timur laut |
| **COG** (course over ground) | ke mana kapal **benar-benar bergerak** | otomatis mengompensasi drift | tidak valid saat kapal pelan (vektor kecepatan jadi noise murni) |

COG dihitung dari vektor kecepatan GPS: `chi_COG = atan2(vy, vx)` (ENU), lalu dikonversi
ke kompas.

Karena itu ada gerbang kecepatan: di bawah `MIN_SPEED_COG = 0.3` m/s, sistem otomatis
jatuh balik ke heading.

Saat ini `USE_COG = False` — wajar untuk SITL yang tidak ada arusnya. **Di air nyata
berarus, `USE_COG = True` biasanya memberi XTE jauh lebih kecil.**

Kolom `arah_dipakai` di CSV merekam sumber mana yang aktif tiap tick.

---

## 6. Kendali PID

Fungsi `pid_to_rc()` (baris 154–200). Bagian yang paling banyak "trik"-nya — diurai per
lapisan.

### 6.1 Lapisan 1 — Pelembut setpoint (rate limiter), baris 162–168

```python
step = clamp(wrap180(des_compass - self.des_filt),
             -TURN_RATE_MAX*dt, TURN_RATE_MAX*dt)
self.des_filt = (self.des_filt + step) % 360.0
err = wrap180(self.des_filt - cur)
```

```
psi_d_filt[k] = psi_d_filt[k-1]
              + clamp( wrap180(psi_d - psi_d_filt[k-1]), ±TURN_RATE_MAX * dt )
```

**Masalah yang dipecahkan:** saat ganti waypoint, `chi_d` bisa meloncat puluhan derajat
dalam satu tick. Suku P (gain 4 PWM/deg) akan langsung menyentak steering. Rate limiter
mengubah loncatan itu jadi **sapuan bertahap** maksimal `TURN_RATE_MAX` = 120 °/s
(= 12° per tick pada 10 Hz).

Error akhir:

```
err = wrap180(psi_d_filt - psi_aktual)      # derajat, rentang ±180
```

### 6.2 Lapisan 2 — Derivative on measurement, baris 171–177

```python
dpsi = wrap180(cur - self.prev_meas)
raw  = -dpsi / dt
self.deriv_filt = (1 - DERIV_LP)*self.deriv_filt + DERIV_LP*raw
```

```
D_raw = -wrap180(psi[k] - psi[k-1]) / dt   ≈  -yaw_rate   [deg/s]
```

**Kenapa turunan pengukuran, bukan turunan error?**
Kalau `d(err)/dt` dipakai langsung, setiap perubahan setpoint menghasilkan spike turunan
raksasa (*derivative kick*). Karena `err = psi_d - psi`, maka saat `psi_d` konstan berlaku
`d(err)/dt = -d(psi)/dt` — jadi memakai `-yaw_rate` memberi efek peredaman yang sama
**tanpa** kick. Ini praktik standar PID industri.

Secara fisik, suku D di sini adalah **peredam laju yaw**: makin cepat kapal berputar,
makin kuat lawan yang diberikan. Inilah yang menahan ayunan (overshoot) setelah belok.

Lalu di-low-pass orde satu (EMA):

```
D[k] = (1 - a) * D[k-1] + a * D_raw          a = DERIV_LP = 0.3
```

Konstanta waktu ekuivalen `tau = dt*(1-a)/a = 0.1 * 0.7/0.3 ≈ 0.23 s`
(cutoff ≈ 0.7 Hz). Perlu karena turunan numerik dari GPS/kompas sangat berisik — tanpa
filter, suku D justru menyuntik noise ke motor.

### 6.3 Lapisan 3 — Persamaan PID, baris 179–184

```
u = KP * err + KI * integ + KD * D          [satuan: PWM]

dengan integ = integral dari err terhadap waktu  [deg·s]
```

Perhatikan `self.integ` menyimpan integral **mentah**; gain `KI` dikalikan di luar.

Peran tiap suku, dengan nilai gain di file:

| Suku | Gain | Arti fisik | Cek angka |
|---|---|---|---|
| **P** | `KP = 4.0` PWM/deg | koreksi proporsional error arah | error 75° sudah menjenuhkan steering (75 × 4 = 300 = `STEER_MAX`) |
| **I** | `KI = 0.2` PWM/(deg·s) | hapus bias tetap (arus, misalignment motor) | butuh error 5° selama 10 s untuk menghasilkan 10 PWM — sengaja lambat |
| **D** | `KD = 4.0` PWM/(deg/s) | redam laju yaw | yaw 30 °/s → 120 PWM melawan |

Komentar di kode merekam riwayat tuning: `KP` diturunkan dari 8 → 4 (belok terlalu
menyentak), `KD` dinaikkan (ayunan), `KI` dibuat kecil karena SITL tidak berarus.

### 6.4 Anti-windup (conditional integration), baris 180–183

```python
u_unsat = KP*err + KI*self.integ + KD*deriv
if self.prev_t is not None and abs(u_unsat) < STEER_MAX:
    self.integ = clamp(self.integ + err*dt,
                       -I_LIMIT/max(KI,1e-6), I_LIMIT/max(KI,1e-6))
u = KP*err + KI*self.integ + KD*deriv
```

Dua lapis proteksi:

1. **Conditional integration** — integral hanya bertambah bila keluaran belum jenuh
   (`|u| < 300`). Kalau steering sudah mentok, menambah integral tidak menambah aksi apa
   pun; ia hanya menumpuk "utang" yang harus dibayar dengan overshoot besar saat error
   akhirnya berbalik tanda. Ini yang disebut *integral windup*.
2. **Hard clamp** — `|integ| <= I_LIMIT/KI = 300/0.2 = 1500` deg·s, artinya kontribusi
   suku I tidak pernah melebihi 300 PWM (= `STEER_MAX`).

Perhatikan `u_unsat` sengaja dihitung dengan integral **lama** sebagai penguji, lalu `u`
dihitung ulang dengan integral yang sudah diperbarui — urutan yang benar untuk skema ini.

### 6.5 Steering → PWM + slew rate, baris 187–192

```
steer_raw = clamp(1500 + STEER_SIGN * u,  1200,  1800)
```

`STEER_SIGN` (±1) adalah **switch kalibrasi arah**. Kalau di lapangan kapal belok ke arah
berlawanan dari yang seharusnya, cukup ubah jadi `-1` — **jangan** otak-atik gain.

Lalu pembatas laju perubahan (slew):

```
steer[k] = steer[k-1] + clamp(steer_raw - steer[k-1], ±STEER_SLEW * dt)
```

`STEER_SLEW` = 800 PWM/s → 80 PWM per tick. Karena rentang penuh cuma 600 PWM, batas ini
praktis longgar (butuh 0.75 s dari mentok kiri ke mentok kanan). Komentar baris 53
mengonfirmasi ini disengaja: kelembutan sekarang datang dari gain rendah + rate limiter
setpoint, bukan dari slew.

### 6.6 Pencampuran diferensial, baris 194–199

```
turn = (steer - 1500) / STEER_MAX          # -1 .. +1
diff = turn * MAX_DIFF                     # MAX_DIFF = 150
kiri  = clamp(1590 - diff, 1300, 1700)
kanan = clamp(1590 + diff, 1300, 1700)
```

> **Ini hanya untuk pencatatan CSV** (komentar baris 194 menyatakannya eksplisit).
> Yang benar-benar menggerakkan motor adalah mixer skid-steer ArduPilot.

Angka ini berguna untuk memverifikasi bahwa perintah belok tidak menuntut selisih motor
yang tak masuk akal.

---

## 7. Antarmuka Aktuator (RC Override)

`send_rc()` (baris 202–208):

```python
ch = [0]*18
ch[0] = steer   # RC1 = steering
ch[2] = thr     # RC3 = throttle
```

Pesan `OverrideRCIn` meniru stik RC. Satu catatan MAVLink: dalam `RC_CHANNELS_OVERRIDE`,
nilai **0 = lepaskan kanal ini kembali ke radio**, sedangkan `UINT16_MAX` = abaikan
(pertahankan override). Jadi kode ini secara aktif melepas kanal 2 dan 4–18 setiap siklus.
Untuk skid-steer boat yang cuma butuh 2 kanal, ini tidak bermasalah.

---

## 8. Mesin Status `loop()`

Timer berjalan pada `LOOP_HZ = 10` Hz (periode 0.1 s). Urutan gerbang logika tiap tick:

```
          ┌──────────────┐
          │  tunggu fix  │  have_fix == False → return
          └──────┬───────┘
                 ▼
          ┌──────────────┐
          │ MANUAL + ARM │  ready == False → ensure_manual_armed()
          └──────┬───────┘
                 ▼
          ┌──────────────┐
          │  TO_START    │  lead-in menuju WP0
          └──────┬───────┘
                 ▼
          ┌──────────────┐
          │   FOLLOW     │  LOS + PID, per segmen
          └──────┬───────┘
                 ▼
          ┌──────────────┐
          │  HOLD/selesai│  idx >= N-1
          └──────────────┘
```

### Tahap 0 — tunggu GPS

```python
if not self.have_fix: return
```

### Tahap 1 — MANUAL + ARM, `ensure_manual_armed()` (baris 216–225)

Setiap tick mengirim RC netral (penting: ArduPilot menolak arming bila throttle tidak
netral). Setiap 10 tick (≈1 s) melakukan **satu** langkah berurutan:

1. Kalau mode ≠ MANUAL → request `SetMode("MANUAL")`
2. Kalau belum armed → request `CommandBool(True)`
3. Kalau keduanya beres → `self.ready = True`

Pakai `call_async` supaya tidak memblokir timer — pola yang benar di rclpy
single-threaded executor (`call()` sinkron di dalam callback timer akan deadlock).

### Tahap 2 — fase `TO_START` (lead-in), baris 235–249

Kapal biasanya di-spawn tidak tepat di WP0. Fase ini mendekatkan kapal ke titik awal
lebih dulu, dikendalikan PID yang sama tapi dengan setpoint = **bearing langsung ke WP0**:

```
beta  = atan2(y0 - y, x0 - x)
psi_d = (90 - degrees(beta)) mod 360
```

Syarat pindah ke `FOLLOW` (baris 240) ada **dua, OR**:

- `d0 < ACCEPT_RADIUS` (0.65 m) — sudah cukup dekat, **atau**
- `s0 >= 0` — proyeksi posisi kapal pada arah segmen WP0→WP1 sudah non-negatif, artinya
  kapal **sudah melewati bidang tegak lurus di WP0**

Syarat kedua adalah pengaman anti-orbit: kapal yang bergerak pasti akan melewati bidang
itu, meski gagal masuk radius 0.65 m.

### Tahap 3 — `FOLLOW`: mengikuti jalur

Tiap tick:

1. `los_course()` → `chi_d`, XTE, `along`, panjang segmen
2. konversi ke kompas → `des`
3. `pid_to_rc(des)` → steering PWM
4. rekam data gerbang (lihat §9)
5. cek syarat pindah waypoint
6. kirim RC + tulis CSV + log

**Syarat pindah waypoint** (baris 276–280):

```python
radius = FINAL_RADIUS if is_final else ACCEPT_RADIUS   # 0.30 : 0.65
switch = dist_t < radius or along >= seg
```

```
switch  <=>  jarak_ke_target < radius        (masuk lingkaran terima)
         OR  along >= panjang_segmen         (lewat bidang tegak lurus)
```

Kondisi kedua adalah kunci robustness. Kalau hanya pakai radius, kapal yang meleset
(misal karena arus) akan **mengorbit** waypoint selamanya karena tidak pernah masuk
lingkaran. Kriteria "sudah melewati garis tegak lurus di waypoint" pasti terpenuhi oleh
kapal yang bergerak maju. Komentar baris 278–279 menjelaskan ini persis.

Pada pindah waypoint, hanya **integral yang direset** (`reset_pid()`, baris 210–214).
State turunan, `des_filt`, dan `steer_cmd` sengaja dibiarkan kontinu supaya tidak ada
loncatan kemudi di transisi.

### Tahap 4 — selesai, baris 251–258

Saat `idx >= len(off) - 1` (semua segmen habis): request mode **HOLD**, cetak ringkasan,
kirim RC netral terus-menerus.

---

## 9. Penilaian Gerbang & Logging

`GATE_IDX = [1, 2, 4, 6, 8, 9]` — indeks waypoint yang berperan sebagai gerbang.

### Perekaman (baris 269–274)

Setiap tick, jika target adalah gerbang, hitung jarak ke gerbang. Simpan triplet
`(d, cross, herr)` **hanya bila jarak lebih kecil dari rekor sebelumnya**.

Efeknya: yang tercatat adalah kondisi kapal pada **titik pendekatan terdekat**
(*closest approach*) — momen paling representatif untuk menilai seberapa presisi kapal
melewati gerbang.

Heading error gerbang sengaja dihitung berbeda dari error PID:

```python
herr = wrap180(des - self.hdg)     # selalu heading lambung, bukan COG
```

Karena kriteria gerbang menilai **orientasi fisik kapal**, bukan arah geraknya.

### Penilaian (baris 283–289)

Dilakukan saat waypoint di-switch:

```
LULUS  <=>  |XTE| <= XTE_LIMIT (0.30 m)   AND   |hdg_err| <= HDG_LIMIT (10 deg)
```

Hasil dicetak per gerbang, lalu direkap di `summary()` sebagai "Lulus N/6 gerbang".

### CSV

Tujuan: `/tmp/los_diffdrive_log.csv` (konstanta `CSV_PATH`).

| Kolom | Isi |
|---|---|
| `t` | detik sejak node mulai |
| `wp_idx` | indeks segmen aktif |
| `xte_m` | cross-track error (m), + = kiri jalur |
| `theta_err_deg` | error sudut yang dipakai PID |
| `steer_pwm` | keluaran steering setelah clamp + slew |
| `thr_kiri`, `thr_kanan` | mixing diferensial (ilustratif, lihat §13) |
| `arah_dipakai` | `HDG` atau `COG` |
| `di_gerbang` | 1 bila target saat ini adalah gerbang |

Cukup untuk plot XTE vs waktu dan analisis tuning pasca-uji.

---

## 10. Siklus Hidup Program

`main()` (baris 315–323):

```python
plan = sys.argv[1] if len(sys.argv) > 1 else "reference_with_gates.plan"
rclpy.init(); node = DiffDrivePID(plan)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.send_rc(STEER_NEUTRAL, THROTTLE_STOP)   # ← pengaman
    node.csv.flush(); node.csv.close()
    node.destroy_node(); rclpy.shutdown()
```

Blok `finally` adalah **safety net**: apa pun penyebab keluar (Ctrl-C, exception, misi
selesai), perintah terakhir yang dikirim adalah steering netral + throttle stop. Tanpa
ini, ArduPilot akan menahan nilai override terakhir dan kapal terus melaju.

### Cara menjalankan

```bash
python3 los_diffdrive_pid_ros2.py reference_with_gates.plan
```

Prasyarat:

- MAVROS sudah jalan dan terhubung ke autopilot
- Kendaraan sudah punya GPS fix
- ArduPilot dikonfigurasi skid-steer (`SERVO1_FUNCTION=73`, `SERVO3_FUNCTION=74`)

---

## 11. Ringkasan Semua Rumus

Lima belas persamaan yang menyusun rantai kendali, masing-masing dalam dua bentuk:
**notasi matematis** untuk dibaca dan dipahami, serta **potongan kode aslinya** untuk
dicocokkan dengan berkas sumber.

Notasi: `sat(·)` = clamp/potong ke rentang · `wrap₁₈₀` = bungkus ke (−180°, 180°] ·
`Δt` = periode loop (0.1 s) · `[k]` = nilai pada tick sekarang.

---

### 1 · Proyeksi geodetik → bidang datar lokal
`ll_to_enu()` — baris 77

**Matematis**

```
x_E = R · cos φ₀ · (λ − λ₀)
y_N = R · (φ − φ₀)

φ lintang, λ bujur (radian);  R = 6 378 137 m
```

**Di kode**

```python
return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)),
        math.radians(lat - lat0) * EARTH_R)
```

---

### 2 · Sudut segmen jalur
`los_course()` — baris 145

**Matematis**

```
α = atan2(y_{k+1} − y_k,  x_{k+1} − x_k)

CCW dari sumbu Timur, satuan radian
```

**Di kode**

```python
alpha = math.atan2(yk1 - yk, xk1 - xk)
```

---

### 3 · Jarak sepanjang jalur
`los_course()` — baris 147

**Matematis**

```
s = Δx cos α + Δy sin α

(Δx, Δy) = posisi relatif terhadap WP_k
```

**Di kode**

```python
along =  dx * math.cos(alpha) + dy * math.sin(alpha)
```

---

### 4 · Cross-track error
`los_course()` — baris 148

**Matematis**

```
e = −Δx sin α + Δy cos α

e > 0  ⇒  kapal di KIRI jalur
```

**Di kode**

```python
cross = -dx * math.sin(alpha) + dy * math.cos(alpha)
```

---

### 5 · Hukum LOS lookahead
`los_course()` — baris 149

**Matematis**

```
χ_d = α + arctan( −e / Δ )

Δ = LOOKAHEAD = 2.5 m — jarak titik bidik di depan
```

**Di kode**

```python
chi_d = wrap_pi(alpha + math.atan2(-cross, LOOKAHEAD))
```

---

### 6 · Konversi ENU → kompas
`loop()` — baris 266

**Matematis**

```
ψ = (90° − χ) mod 360°

ENU (nol di Timur, CCW)  →  kompas (nol di Utara, CW)
```

**Di kode**

```python
des = (90.0 - math.degrees(chi_d)) % 360.0
```

---

### 7 · Course over ground
`current_dir()` — baris 138

**Matematis**

```
χ_COG = atan2(v_y, v_x)
ψ_COG = (90° − χ_COG) mod 360°

berlaku hanya bila √(v_x² + v_y²) ≥ 0.3 m/s
```

**Di kode**

```python
cog_enu = math.atan2(self.vy, self.vx)
return (90.0 - math.degrees(cog_enu)) % 360.0, "COG"
```

---

### 8 · Pelembut setpoint (rate limiter)
`pid_to_rc()` — baris 165

**Matematis**

```
ψ_d^f[k] = ψ_d^f[k−1] + sat( wrap₁₈₀(ψ_d − ψ_d^f[k−1]),  ±ψ̇_max · Δt )

ψ̇_max = TURN_RATE_MAX = 120 °/s
```

**Di kode**

```python
step = clamp(wrap180(des_compass - self.des_filt),
             -TURN_RATE_MAX * dt, TURN_RATE_MAX * dt)
self.des_filt = (self.des_filt + step) % 360.0
```

---

### 9 · Error sudut
`pid_to_rc()` — baris 168

**Matematis**

```
e_ψ = wrap₁₈₀( ψ_d^f − ψ )

ψ = heading kompas atau COG, lihat §5
```

**Di kode**

```python
err = wrap180(self.des_filt - cur)
```

---

### 10 · Turunan pengukuran + low-pass
`pid_to_rc()` — baris 174

**Matematis**

```
Δψ   = wrap₁₈₀( ψ[k] − ψ[k−1] )
D[k] = (1 − a) · D[k−1] + a · ( −Δψ / Δt )

a = DERIV_LP = 0.3  ⇒  τ ≈ 0.23 s
```

**Di kode**

```python
dpsi = wrap180(cur - self.prev_meas)
raw  = -dpsi / dt
self.deriv_filt = (1 - DERIV_LP) * self.deriv_filt + DERIV_LP * raw
```

---

### 11 · Persamaan PID
`pid_to_rc()` — baris 184

**Matematis**

```
u = K_p · e_ψ  +  K_i · ∫ e_ψ dt  +  K_d · D

keluaran dalam satuan PWM;  K_p / K_i / K_d = 4.0 / 0.2 / 4.0
```

**Di kode**

```python
u = KP * err + KI * self.integ + KD * deriv
```

---

### 12 · Kemudi ke satuan PWM
`pid_to_rc()` — baris 187

**Matematis**

```
u_pwm = sat( 1500 + σ · u,  1500 ± 300 )

σ = STEER_SIGN = ±1 (kalibrasi arah belok)
```

**Di kode**

```python
steer_raw = clamp(STEER_NEUTRAL + STEER_SIGN * u,
                  STEER_NEUTRAL - STEER_MAX,
                  STEER_NEUTRAL + STEER_MAX)
```

---

### 13 · Pembatas laju kemudi (slew)
`pid_to_rc()` — baris 190

**Matematis**

```
u_pwm[k] = u_pwm[k−1] + sat( u_raw − u_pwm[k−1],  ±Ṡ · Δt )

Ṡ = STEER_SLEW = 800 PWM/s  ⇒  80 PWM per tick
```

**Di kode**

```python
dmax  = STEER_SLEW * dt
steer = self.steer_cmd + clamp(steer_raw - self.steer_cmd, -dmax, dmax)
```

---

### 14 · Pencampuran diferensial *(log saja)*
`pid_to_rc()` — baris 195

**Matematis**

```
τ = (u_pwm − 1500) / 300  ∈ [−1, 1]
L = T − τ · D_max
R = T + τ · D_max

T = 1590 PWM, D_max = 150; mixer sebenarnya ada di ArduPilot
```

**Di kode**

```python
turn  = (steer - STEER_NEUTRAL) / STEER_MAX
diff  = turn * MAX_DIFF
kiri  = int(clamp(THROTTLE_PWM - diff, 1300, 1700))
kanan = int(clamp(THROTTLE_PWM + diff, 1300, 1700))
```

---

### 15 · Kriteria pindah waypoint
`loop()` — baris 280

**Matematis**

```
switch  ⟺  ‖p − p_{k+1}‖ < R_acc   ∨   s ≥ L_seg

R_acc = 0.65 m (0.30 m di titik akhir)
suku kedua = kapal sudah lewat bidang tegak lurus di waypoint
```

**Di kode**

```python
radius = FINAL_RADIUS if is_final else ACCEPT_RADIUS
switch = dist_t < radius or along >= seg
```

---

## 12. Tabel Parameter Tuning

| Konstanta | Nilai | Satuan | Efek bila dinaikkan |
|---|---|---|---|
| `THROTTLE_PWM` | 1590 | PWM | kapal lebih cepat; XTE cenderung membesar di tikungan |
| `STEER_MAX` | 300 | PWM | belokan maksimum lebih tajam |
| `STEER_SIGN` | 1 | ±1 | **balik ke −1 bila kapal belok ke arah salah** |
| `KP` | 4.0 | PWM/deg | respons cepat, tapi rawan osilasi |
| `KI` | 0.2 | PWM/(deg·s) | hapus bias arus lebih cepat, tapi rawan overshoot |
| `KD` | 4.0 | PWM/(deg/s) | peredaman lebih kuat, tapi lebih sensitif noise |
| `I_LIMIT` | 300 | PWM | batas kontribusi suku I |
| `DERIV_LP` | 0.3 | 0..1 | turunan lebih responsif tapi lebih berisik |
| `TURN_RATE_MAX` | 120 | deg/s | sapuan setpoint lebih cepat (turunkan ke 40–60 bila masih menyentak) |
| `STEER_SLEW` | 800 | PWM/s | kemudi boleh berubah lebih cepat |
| `LOOKAHEAD` | 2.5 | m | konvergensi ke jalur lebih lembut/lambat |
| `ACCEPT_RADIUS` | 0.65 | m | waypoint lebih mudah dianggap tercapai |
| `FINAL_RADIUS` | 0.30 | m | radius untuk titik akhir |
| `USE_COG` | False | bool | pakai arah gerak GPS, bukan heading lambung |
| `MIN_SPEED_COG` | 0.3 | m/s | ambang kecepatan agar COG dianggap valid |
| `LOOP_HZ` | 10 | Hz | frekuensi kendali |
| `XTE_LIMIT` | 0.30 | m | kriteria lulus gerbang (XTE) |
| `HDG_LIMIT` | 10 | deg | kriteria lulus gerbang (heading error) |

### Urutan tuning yang disarankan

1. Set `KI = 0`, `KD = 0`. Naikkan `KP` sampai kapal mengikuti jalur tapi mulai berosilasi.
2. Turunkan `KP` ~30%, lalu naikkan `KD` sampai ayunan teredam.
3. Baru naikkan `KI` pelan-pelan — hanya bila ada XTE steady-state (biasanya karena arus).
4. Kalau transisi waypoint menyentak, turunkan `TURN_RATE_MAX` (bukan `KP`).
5. Kalau kapal zig-zag di lintasan lurus, naikkan `LOOKAHEAD`.

---

## 13. Catatan & Temuan

Bukan bug fatal, tapi perlu diketahui saat mengembangkan/menjelaskan kode ini:

1. **`FINAL_OVERSHOOT` (baris 63) tidak pernah dipakai.** Sisa dari versi lama yang punya
   mode "homing" ke titik akhir. Logika homing sudah dihapus (lihat komentar baris
   264–265) tapi konstantanya tertinggal.

2. **Konvensi arah pada log `thr_kiri`/`thr_kanan` mungkin terbalik** dari mixer ArduPilot.
   Di kode, `steer` naik → `kanan` naik & `kiri` turun (belok kiri). Pada ArduPilot
   skid-steer, RC1 naik umumnya = belok kanan (motor kiri lebih cepat). Karena kolom ini
   murni pencatatan, perilaku kapal tidak terpengaruh — tapi **jangan pakai kolom ini
   untuk debug arah belok**; gunakan `STEER_SIGN`.

3. **`is_final` hampir tidak berpengaruh lagi** — sekarang hanya memilih radius 0.30 vs
   0.65 m dan mengubah teks log. Segmen terakhir diperlakukan sama dengan yang lain
   (pakai LOS penuh), yang justru lebih baik.

4. **Rate limiter setpoint praktis longgar** pada 120 °/s (12° per tick). Kalau masih ada
   sentakan saat ganti waypoint, ini parameter pertama yang layak diturunkan.

5. **`self.origin` selalu sama dengan `wp_ll[0]`**, jadi guard `if self.origin is None` di
   baris 126–127 secara efektif redundan — tapi harmless dan membuat maksudnya jelas.

6. **Throttle konstan** berarti kapal tidak melambat di tikungan tajam. Kalau XTE di
   tikungan sulit ditekan, opsi pengembangan berikutnya adalah menurunkan throttle
   proporsional terhadap `|err|`.
