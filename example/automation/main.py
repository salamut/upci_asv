from pymavlink import mavutil

def gps_callback(conn,msg):
    if msg.get_type() == 'GPS_RAW_INT':
        print(f"GPS Raw Data: Lat {msg.lat}, Lon {msg.lon}, Alt {msg.alt}")

def rcin_callback(conn,msg):
    if msg.get_type() == 'RC_CHANNELS':
        print(f"RC Channels: {msg.chan1_raw}, {msg.chan2_raw}, {msg.chan3_raw}, {msg.chan4_raw}")
        if msg.chan6_raw > 1500:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_AUTO_ARMED, 0, 0, 0, 0, 0, 0)
        else:
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_MANUAL_DISARMED, 0, 0, 0, 0, 0, 0)

if __name__ == "__main__":
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:14550')
    connection.wait_heartbeat()
    connection.message_hooks.append(gps_callback)
    connection.message_hooks.append(rcin_callback)
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
    while True:
        msg = connection.recv_match(blocking=True)
        if not msg:
            continue