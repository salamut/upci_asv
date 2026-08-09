#!/usr/bin/env python3
"""Uji jembatan JSON tanpa SITL: kirim paket servo (magic 18458) ke UDP 9002
persis seperti ArduPilot, baca balasan JSON, cetak ringkasan tiap detik.

Pakai: python3 test_bridge.py [pwm_kiri] [pwm_kanan] [durasi_s]
"""
import json, math, socket, struct, sys, time

pwm_l = int(sys.argv[1]) if len(sys.argv) > 1 else 1590
pwm_r = int(sys.argv[2]) if len(sys.argv) > 2 else 1590
dur   = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.2)
addr = ("127.0.0.1", 9002)

pwm = [1500] * 16
pwm[0] = pwm_l   # SERVO1 = ThrottleLeft
pwm[2] = pwm_r   # SERVO3 = ThrottleRight

t0 = time.time(); count = 0; last_print = 0.0; last = None; first = None
while time.time() - t0 < dur:
    count += 1
    pkt = struct.pack("<HHI16H", 18458, 500, count, *pwm)
    sock.sendto(pkt, addr)
    try:
        data, _ = sock.recvfrom(2048)
    except socket.timeout:
        print("timeout menunggu balasan JSON"); continue
    d = json.loads(data.decode().strip())
    last = d
    if first is None:
        first = d
        print("balasan pertama:", json.dumps(d))
    now = time.time()
    if now - last_print > 2.0:
        last_print = now
        vn, ve, vd = d["velocity"]
        r, p, y = d["attitude"]
        n, e, dn = d["position"]
        print(f"t={d['timestamp']:8.2f} N={n:+7.2f} E={e:+7.2f} "
              f"spd={math.hypot(vn, ve):.2f} m/s "
              f"hdg={math.degrees(y) % 360:6.1f} rpy=({math.degrees(r):+.1f},{math.degrees(p):+.1f})")

if first and last:
    dt = last["timestamp"] - first["timestamp"]
    print(f"\nselesai: {count} paket, sim-dt total {dt:.2f} s, "
          f"laju balasan ~{count/max(dt,1e-9):.0f} Hz")
