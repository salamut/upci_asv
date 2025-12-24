import asyncio
import io
from math import sqrt
import os
from typing import Any, Dict, List

import httpx
import orjson
import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPull
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

# ---------------------------------------------------------------------------
# Cloudflare RTC Configuration
# ---------------------------------------------------------------------------

CLOUDFLARE_APP_ID = "155b08732447bf4e54acaac6d1706a19"
CLOUDFLARE_API_KEY = "fe6dd0f063e201ef4893e80385e779f327905f4a07563f87f54c58c10fef7e02"
CLOUDFLARE_API_BASE = "https://rtc.live.cloudflare.com/v1"
HEADERS = {
    "Authorization": f"Bearer {CLOUDFLARE_API_KEY}",
    "Content-Type": "application/json",
}


# ---------------------------------------------------------------------------
# Pydantic Models
# ---------------------------------------------------------------------------

class LocalDescription(BaseModel):
    type: str
    sdp: str


class TrackInfo(BaseModel):
    location: str
    sessionId: str
    trackName: str


class TrackPayload(BaseModel):
    sessionDescription: LocalDescription
    tracks: list[TrackInfo]


# ---------------------------------------------------------------------------
# ROS QoS Profiles
# ---------------------------------------------------------------------------

QOS_SENSOR_DATA = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ---------------------------------------------------------------------------
# ROS2 Backend Node
# ---------------------------------------------------------------------------
class BackendApp(Node):
    def __init__(self):
        super().__init__("backend_node")

        # State
        self.active_connections: List[WebSocket] = []
        self.data: Dict[str, Any] = {}
        self.waypoints: List[Dict[str, Any]] = []
        self.img: bytes | None = None
        self.publisher_session_id: str | None = None

        # Initialize ROS2 interfaces
        self._init_services()
        self._init_subscriptions()

    # ---- Initialization ----
    def _init_services(self):
        self.get_session_service = self.create_client(Trigger, "/get_publisher_session_id")
        self.get_session_service.wait_for_service()
        self.get_logger().info("get_publisher_session_id service is ready")

        req = Trigger.Request()
        future = self.get_session_service.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if result := future.result():
            if result.success:
                self.publisher_session_id = result.message
                self.get_logger().info(f"Obtained publisher session ID: {self.publisher_session_id}")

        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull")

    def _init_subscriptions(self):
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._cb_global_position, QOS_SENSOR_DATA)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self._cb_global_velocity, QOS_SENSOR_DATA)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._cb_compass_heading, QOS_SENSOR_DATA)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self._cb_waypoints, QOS_SENSOR_DATA)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self._cb_waypoint_reached, QOS_SENSOR_DATA)

    # ---- Callbacks ----
    def _cb_global_position(self, msg: NavSatFix):
        self.data["latitude"] = msg.latitude
        self.data["longitude"] = msg.longitude
        self.data["altitude"] = msg.altitude

    def _cb_global_velocity(self, msg: TwistStamped):
        velocity = sqrt(
            msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2 + msg.twist.linear.z ** 2
        )
        self.data["velocity"] = velocity

    def _cb_waypoints(self, msg: WaypointList):
        self.waypoints = [
            {
                "frame": wp.frame,
                "command": wp.command,
                "is_current": wp.is_current,
                "autocontinue": wp.autocontinue,
                "param1": wp.param1,
                "param2": wp.param2,
                "param3": wp.param3,
                "param4": wp.param4,
                "x_lat": wp.x_lat,
                "y_long": wp.y_long,
                "z_alt": wp.z_alt,
            }
            for wp in msg.waypoints
        ]

    def _cb_waypoint_reached(self, msg: WaypointReached):
        self.data["waypoint_reached"] = {"wp_seq": msg.wp_seq}

    def _cb_compass_heading(self, msg: Float64):
        self.data["heading"] = msg.data

    # ---- WebSocket Management ----
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        disconnected = []
        for ws in self.active_connections:
            try:
                await ws.send_text(message)
            except WebSocketDisconnect:
                disconnected.append(ws)
        for ws in disconnected:
            self.disconnect(ws)

    async def realtime_broadcast(self):
        """Continuously broadcast telemetry data."""
        while True:
            if self.data:
                msg_json = orjson.dumps(self.data).decode()
                await self.broadcast(msg_json)
            await asyncio.sleep(0.05)


backend = None


async def lifespan(app: FastAPI):
    global backend
    print("Starting up...")
    rclpy.init()
    backend = BackendApp()

    broadcast_task = asyncio.create_task(backend.realtime_broadcast())
    ros_spin_task = asyncio.create_task(ros_spin())

    yield

    print("Shutting down...")
    broadcast_task.cancel()
    ros_spin_task.cancel()
    backend.destroy_node()
    rclpy.shutdown()


app = FastAPI(lifespan=lifespan)


async def ros_spin():
    global backend
    while rclpy.ok():
        rclpy.spin_once(backend, timeout_sec=0.1)
        await asyncio.sleep(0.01)


@app.get("/")
async def get():
    file_path = os.path.join(
        get_package_share_directory("web_monitoring"), "web", "index.html"
    )
    return FileResponse(file_path)


@app.get("/image/up")
async def get_image():
    if not backend.img:
        return Response(status_code=404, content="No image available")
    img = io.BytesIO(backend.img)
    headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "Expires": "0",
    }
    return Response(content=img.getvalue(), media_type="image/jpeg", headers=headers)

@app.get("/image/down")
async def get_image():
    if not backend.img:
        return Response(status_code=404, content="No image available")
    img = io.BytesIO(backend.img)
    headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "Expires": "0",
    }
    return Response(content=img.getvalue(), media_type="image/jpeg", headers=headers)


@app.get("/waypoints")
async def get_image():
    if backend.waypoints is None:
        backend.waypoint_pull.wait_for_service()
        req = WaypointPull.Request()
        future = backend.waypoint_pull.call_async(req)
        await asyncio.wrap_future(future)
    return {"waypoints": backend.data.get("waypoints", [])}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await backend.connect(websocket)
    await backend.send_personal_message(
        orjson.dumps({"message": "connected to server"}).decode("utf-8"), websocket
    )
    try:
        while True:
            await asyncio.sleep(1)

    except WebSocketDisconnect:
        backend.disconnect(websocket)
        print("Client disconnected")


@app.get("/get_ephemeral_viewer")
async def get_viewer_session():
    """
    Backend creates ephemeral viewer session.
    Browser never sees CF_API_TOKEN.
    """
    async with httpx.AsyncClient() as client:
        # Create a new viewer session
        resp = await client.post(
            f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/new",
            headers=HEADERS,
        )
        if resp.status_code != 201:
            print("Error creating viewer session:", resp.text)
        session = resp.json()
        print("Viewer session response:", session)
        viewer_session_id = session["sessionId"]
        # Create a new track for the viewer
        resp = await client.get(
            f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/{backend.publisher_session_id}",
            headers=HEADERS,
        )
        if resp.status_code != 200:
            print("Error creating track:", resp.text)
        track = resp.json()
        track_name = track["tracks"][0]["trackName"]
        print("Track creation response:", track_name)

        # Return only ephemeral info
        return {
            "appId": CLOUDFLARE_APP_ID,
            "publisherSessionId": backend.publisher_session_id,
            "viewerSessionId": viewer_session_id,
        }


@app.post("/tracks/{session_id}")
async def post_tracks(session_id: str, payload: TrackPayload):
    async with httpx.AsyncClient() as client:
        resp = await client.post(
            f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/{session_id}/tracks/new",
            json=payload.model_dump(),
            headers=HEADERS,
        )
        if resp.status_code != 200:
            print("Error fetching tracks:", resp.text)
        result = resp.json()
        return result


def main():
    uvicorn.run(
        "web_monitoring.web_monitoring_node:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # disable reload for ROS2 runtime
    )


if __name__ == "__main__":
    main()
