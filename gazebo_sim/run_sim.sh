#!/bin/bash
# =============================================================================
# run_sim.sh — peluncur simulasi Gazebo ASV (Ignition Fortress + ArduPilot SITL)
#
# Urutan: (a) Gazebo world  (b) ArduPilot SITL backend JSON
#         (c) mavlink-router  (d) MAVROS.
# Setelah semuanya hidup, jalankan node navigasi seperti biasa di terminal lain:
#   cd ~/asv_upci/MAVROS_code
#   source /opt/ros/humble/setup.zsh      # zsh! (setup.bash bila shell bash)
#   python3 los_diffdrive_pid_ros2.py reference_with_gates.plan
#
# ----------------------------- PETA PORT -------------------------------------
#   UDP 9002  : paket servo SITL -> plugin AsvJsonBridge di Gazebo (JSON balik
#               ke port asal). Ubah di models/asv_boat/model.sdf <port> DAN
#               opsi --model JSON:... (port SITL tetap 9002 by konvensi).
#   TCP 5760  : MAVLink serial0 SITL <- mavlink-routerd (-p, klien TCP).
#   TCP 5765  : server TCP mavlink-routerd (-t) — QGroundControl bisa konek
#               ke sini (Comm Link TCP 127.0.0.1:5765).
#   UDP 14550 : endpoint router (-e)  -> QGroundControl (autoconnect UDP
#               bawaan QGC, nol konfigurasi).
#   UDP 14552 : endpoint router (-e)  -> MAVROS (fcu_url udp://:14552@).
# -----------------------------------------------------------------------------
# Pemakaian:
#   ./run_sim.sh            # dengan GUI Gazebo
#   HEADLESS=1 ./run_sim.sh # server saja (verifikasi/CI)
# Hentikan dengan Ctrl-C (semua proses ikut dimatikan).
# =============================================================================
DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/simulation/ardupilot}"
MAVLINK_ROUTERD="${MAVLINK_ROUTERD:-$DIR/../tools/x86/mavlink-routerd}"
LOGDIR="$DIR/logs"; mkdir -p "$LOGDIR"

# Origin geodetik = WP0 reference_with_gates.plan (harus sama dengan
# <spherical_coordinates> di worlds/asv_course.sdf).
HOME_LOC="-6.97296528,107.63037888,0,187"

export IGN_GAZEBO_RESOURCE_PATH="$DIR/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$DIR/plugin/build${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"

# ---- GUARD: pastikan tidak ada instans simulasi lain yang masih hidup. ------
# Dua server Gazebo dengan world sama membuat GUI menampilkan DUA kapal dan
# port 9002/5760/14550 bentrok (GPS kacau). PENTING: proses server/gui anak
# ign punya cmdline "ign gazebo server"/"ign gazebo gui" TANPA nama file world
# (bila induknya mati, mereka jadi yatim dan lolos dari pola asv_course.sdf).
STRAY=$(pgrep -f "asv_cours[e].sdf|[i]gn gazebo server|[i]gn gazebo gui|[a]rdurover|[m]avlink-routerd|[m]avproxy" | wc -l)
if [ "$STRAY" -gt 0 ]; then
  echo "[run_sim] PERINGATAN: $STRAY proses simulasi lama masih hidup — dimatikan dulu."
  pkill -f "asv_cours[e].sdf"; pkill -f "[i]gn gazebo server"; pkill -f "[i]gn gazebo gui"
  pkill -f "[a]rdurover"; pkill -f "[m]avlink-routerd"; pkill -f "[m]avproxy"
  pkill -f "[m]avros_node"
  sleep 2
fi

PIDS=()
cleanup() {
  echo; echo "[run_sim] menghentikan semua proses..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f "asv_cours[e].sdf"      2>/dev/null
  pkill -f "[i]gn gazebo server"   2>/dev/null
  pkill -f "[i]gn gazebo gui"      2>/dev/null
  pkill -f "[a]rdurover"           2>/dev/null
  pkill -f "[m]avlink-routerd"     2>/dev/null
  exit 0
}
trap cleanup INT TERM

# (a) Gazebo -------------------------------------------------------------------
GZ_ARGS=(-r -v 1 "$DIR/worlds/asv_course.sdf")
[ "${HEADLESS:-0}" = "1" ] && GZ_ARGS=(-s "${GZ_ARGS[@]}")
echo "[run_sim] (a) Gazebo: ign gazebo ${GZ_ARGS[*]}"
ign gazebo "${GZ_ARGS[@]}" > "$LOGDIR/gazebo.log" 2>&1 &
PIDS+=($!)
sleep 5

# (b) ArduPilot SITL (backend JSON, tanpa MAVProxy) ----------------------------
echo "[run_sim] (b) ArduPilot SITL Rover (JSON) di $ARDUPILOT_DIR"
python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" -v Rover \
    --model JSON:127.0.0.1 -N \
    -l "$HOME_LOC" \
    --add-param-file="$DIR/config/gazebo-boat.parm" \
    --no-mavproxy \
    > "$LOGDIR/sitl.log" 2>&1 &
PIDS+=($!)
sleep 10

# (c) mavlink-router: SITL(tcp 5760) -> QGC(udp 14550 / tcp 5765) + MAVROS(udp 14552)
echo "[run_sim] (c) mavlink-routerd: $MAVLINK_ROUTERD"
"$MAVLINK_ROUTERD" -p 127.0.0.1:5760 -t 5765 \
    -e 127.0.0.1:14550 -e 127.0.0.1:14552 \
    > "$LOGDIR/mavlink-router.log" 2>&1 &
PIDS+=($!)
sleep 3

# (d) MAVROS -------------------------------------------------------------------
echo "[run_sim] (d) MAVROS (fcu_url udp://:14552@)"
# Skrip ini SELALU berjalan di bash (shebang), jadi setup.bash yang benar di
# sini — meskipun shell interaktif Anda zsh. (setup.zsh di bash gagal; dan
# setup.bash di ZSH interaktif malah bisa mengeksekusi ./setup.sh proyek!)
if [ -n "${ZSH_VERSION:-}" ]; then
  source /opt/ros/humble/setup.zsh
else
  source /opt/ros/humble/setup.bash
fi
ros2 launch mavros apm.launch fcu_url:=udp://:14552@ > "$LOGDIR/mavros.log" 2>&1 &
PIDS+=($!)

# Minta telemetri 10 Hz (dulu MAVProxy diam-diam melakukannya; via mavlink-router
# harus diminta sendiri — param SR0_* tidak mempan di ArduPilot 4.8-dev, jadi
# pakai layanan set_stream_rate; tanpa ini heading/posisi cuma 1 Hz -> node LOS
# membaca data basi dan kemudi berosilasi).
echo "[run_sim] meminta stream rate 10 Hz..."
for i in $(seq 1 30); do
  if ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
       "{stream_id: 0, message_rate: 10, on_off: true}" >/dev/null 2>&1; then
    echo "[run_sim] stream rate 10 Hz OK"; break
  fi
  sleep 2
done

echo "[run_sim] semua komponen dijalankan. Log: $LOGDIR/"
echo "[run_sim] cek: ros2 topic echo /mavros/global_position/global --once"
echo "[run_sim] QGC: cukup buka QGC (autoconnect UDP 14550), atau TCP 127.0.0.1:5765"
echo "[run_sim] Ctrl-C untuk berhenti."
wait
