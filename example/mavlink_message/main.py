from pymavlink import mavutil

conn = mavutil.mavlink_connection('tcp:0.0.0.0:5765')

conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))

while True:
    msg = conn.recv_match()
    if msg:
        if msg.get_type() == 'ATTITUDE':
            print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % (msg.roll, msg.pitch, msg.yaw))
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print("Lat: %d, Lon: %d, Alt: %d" % (msg.lat, msg.lon, msg.alt))