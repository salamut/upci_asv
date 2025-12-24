import aioredis
import asyncio
from pymavlink import mavutil
import orjson

ardupilot_sysid = 0
ardupilot_compid = 0

data = {
    "Position": {
        "Latitude": 0,
        "Longitude": 0,
        "Altitude": 0,
        "Velocity": 0,
        "Heading": 0,
    },
    "Status": {
        "Battery": 0,
        "FlightMode": "",
        "Armed": False,
    },
    "Raw": {},
}


def parse_mavlink_message(msg):
    type = msg.get_type()
    msg_dict = msg.to_dict()
    if type == "LOCAL_POSITION_NED":
        Velocity = (
            msg_dict["vx"] ** 2 + msg_dict["vy"] ** 2 + msg_dict["vz"] ** 2
        ) ** 0.5
        data["Position"]["Velocity"] = Velocity

    elif type == "GLOBAL_POSITION_INT":
        data["Position"]["Latitude"] = msg_dict["lat"] / 1e7
        data["Position"]["Longitude"] = msg_dict["lon"] / 1e7
        data["Position"]["Altitude"] = msg_dict["alt"] / 1e3
        data["Position"]["Heading"] = (
            msg_dict["hdg"] / 100 if msg_dict["hdg"] != 65535 else 0
        )

    elif type == "HEARTBEAT":
        modes = {
            0: "MANUAL",
            10: "AUTO",
        }
        data["Status"]["FlightMode"] = modes.get(msg_dict["custom_mode"], "UNKNOWN")
        data["Status"]["Armed"] = bool(
            msg_dict["base_mode"] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    elif type == "BATTERY_STATUS":
        data["Status"]["Battery"] = msg_dict["battery_remaining"]
    data["Raw"][type] = msg_dict


async def main():
    mav_conn = mavutil.mavlink_connection("tcp:0.0.0.0:5765")
    mav_conn.wait_heartbeat()
    print(
        f"Heartbeat from system {mav_conn.target_system} "
        f"component {mav_conn.target_component}"
    )
    redis = aioredis.from_url("redis://localhost:6379")
    pubsub = redis.pubsub()
    heading= subscribe("detection")
    while True:
        msg = mav_conn.recv_match(blocking=False)
        mav_conn.mav.rc_channels_ovveride_send(
            1, 2, 1500, 1500, 1500, 1500, 1500, 1500, 1500
        )
        if not msg:
            await asyncio.sleep(0.01)
            continue
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()
        if msg:
            parse_mavlink_message(msg)
            msg_json = orjson.dumps(data)
            # Publish to Redis channel
            channel = "channel:mavlink"
            # print(f"Publishing MAVLink data: {data}")
            await redis.publish(channel, msg_json)
        await asyncio.sleep(0.01)


if __name__ == "__main__":
    asyncio.run(main())
