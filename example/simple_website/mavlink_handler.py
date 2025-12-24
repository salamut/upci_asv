import asyncio
import orjson
from pymavlink import mavutil
import websockets

pixhawk_sysid = 1
pixhawk_compid = 0

data={
    "Position": {
        "Latitude": 0,
        "Longitude": 0,
        "Altitude": 0,
        "Velocity": 0,
        "Heading": 0,
    },
    "Mission":{
        "Current": 0,
        "Total": 0,
        "Status": "NONE",
        "Items": []
    },
    "Status": {
        "Battery": 0,
        "FlightMode": "",
        "Armed": False,
    },
    "Raw": {}
}
async def mavlink_broadcast(connected_clients):
    # Connect to MAVLink device
    mavlink_connection = mavutil.mavlink_connection("tcp:0.0.0.0:5765")
    mavlink_connection.wait_heartbeat()
    print(
        f"Heartbeat from system {mavlink_connection.target_system} "
        f"component {mavlink_connection.target_component}"
    )
    """Read MAVLink messages and broadcast to clients."""
    # get_initial_mission_data(mavlink_connection)
    while True:
        msg = mavlink_connection.recv_match(blocking=False)
        sysid = msg.get_srcSystem() if msg else None
        compid = msg.get_srcComponent() if msg else None
        if msg and sysid == pixhawk_sysid:
            parse_mavlink_message(msg)
            msg_json = orjson.dumps(data).decode('utf-8')
            to_remove = set()
            for client in connected_clients:
                try:
                    await client.send(msg_json)
                except websockets.exceptions.ConnectionClosed:
                    to_remove.add(client)
            connected_clients.difference_update(to_remove)
        await asyncio.sleep(0.01)

def get_initial_mission_data(conn):
    """Request and retrieve the full mission list from the MAVLink device."""
    conn.waypoint_request_list_send()
    msg = conn.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if not msg:
        print("No MISSION_COUNT received")
        return
    print(f"Mission count: {msg.count}")
    data["Mission"]['Items'] = [None] * msg.count  # Preallocate list
    for seq in range(msg.count):
        conn.mav.mission_request_send(conn.target_system, conn.target_component, seq)
        item_msg = conn.recv_match(type=['MISSION_ITEM'], blocking=True, timeout=1)
        if item_msg:
            parse_mavlink_message(item_msg)
            print(f"Received MISSION_ITEM for seq {seq}")
        else:
            print(f"No MISSION_ITEM received for seq {seq}")
            continue

def parse_mavlink_message(msg):
    """Convert MAVLink message to JSON-serializable dict."""
    type = msg.get_type()
    msg_dict = msg.to_dict()
    if (type == "LOCAL_POSITION_NED"):
        Velocity = (msg_dict["vx"]**2 + msg_dict["vy"]**2 + msg_dict["vz"]**2)**0.5
        data["Position"]['Velocity'] = Velocity

    elif (type == "GLOBAL_POSITION_INT"):
        data["Position"]['Latitude'] = msg_dict["lat"] / 1e7
        data["Position"]['Longitude'] = msg_dict["lon"] / 1e7
        data["Position"]['Altitude'] = msg_dict["alt"] / 1e3
        data["Position"]['Heading'] = msg_dict["hdg"] / 100 if msg_dict["hdg"] != 65535 else 0

    elif (type == "HEARTBEAT"):
        modes = {
            0: "MANUAL",
            10: "AUTO",
        }
        data["Status"]['FlightMode'] = modes.get(msg_dict["custom_mode"], "UNKNOWN")
        data["Status"]['Armed'] = bool(msg_dict["base_mode"] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    elif (type == "BATTERY_STATUS"):
        data["Status"]['Battery'] = msg_dict["battery_remaining"]
    elif (type == "MISSION_CURRENT"):
        data["Mission"]['Current'] = msg_dict["seq"] + 1  # seq is zero-indexed
        data["Mission"]['Total'] = msg_dict["total"]
        status = {
            0: "NONE",
            1: "PLANNING",
            2: "EXECUTING",
            3: "COMPLETED",
            4: "FAILED"
        }
        data["Mission"]['Status'] = status.get(msg_dict["mission_state"], "UNKNOWN")
    elif (type == "MISSION_ITEM"):
        if msg_dict["seq"] not in data['Mission']['Items']:
            data['Mission']['Items'][msg_dict["seq"]] = {
                "X": msg_dict["x"],
                "Y": msg_dict["y"],
            }
    data['Raw'][type]= msg_dict