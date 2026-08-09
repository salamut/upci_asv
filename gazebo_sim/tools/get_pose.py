#!/usr/bin/env python3
"""Baca pose asv_boat dari topic dynamic_pose (dipanggil: ign topic ... | get_pose.py)."""
import sys, re, math

t = sys.stdin.read()
m = re.search(r'name: "asv_boat".*?position \{(.*?)\}.*?orientation \{(.*?)\}', t, re.S)
if not m:
    print("NOTFOUND"); sys.exit(1)

def g(b, k):
    mm = re.search(k + r': ([-0-9.e+]+)', b)
    return float(mm.group(1)) if mm else 0.0

p, o = m.group(1), m.group(2)
qx, qy, qz, qw = g(o, 'x'), g(o, 'y'), g(o, 'z'), g(o, 'w')
roll = math.degrees(math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
pitch = math.degrees(math.asin(max(-1, min(1, 2*(qw*qy-qz*qx)))))
yaw = math.degrees(math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
hdg = (90.0 - yaw) % 360.0
print(f"x={g(p,'x'):+.3f} y={g(p,'y'):+.3f} z={g(p,'z'):+.3f} "
      f"roll={roll:+.1f} pitch={pitch:+.1f} yaw={yaw:+.1f} hdg={hdg:.1f}")
