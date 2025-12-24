import asyncio
import rclpy
from rclpy.node import Node
from rclpy import executors
from std_srvs.srv import Trigger
import cv2
from fractions import Fraction
from av import VideoFrame
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
import requests
import aiohttp
from aiortc.contrib.media import MediaPlayer

CLOUDFLARE_APP_ID = "155b08732447bf4e54acaac6d1706a19"
CLOUDFLARE_API_KEY = "fe6dd0f063e201ef4893e80385e779f327905f4a07563f87f54c58c10fef7e02"
CLOUDFLARE_API_BASE = "https://rtc.live.cloudflare.com/v1"

async def publish_track(session_id, pc, mids_tracks):
    """Send offer for one or more tracks, return Cloudflare SDP answer"""
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    async with aiohttp.ClientSession() as s:
        url = f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/{session_id}/tracks/new"
        resp = await s.post(
            url,
            headers={
                "Authorization": f"Bearer {CLOUDFLARE_API_KEY}",
                "Content-Type": "application/json",
            },
            json={
                "sessionDescription": {"type": "offer", "sdp": pc.localDescription.sdp},
                "tracks": mids_tracks,
            },
        )
        data = await resp.json()
        answer = data["sessionDescription"]
        await pc.setRemoteDescription(
            RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
        )

async def wait_for_ice_connection(pc):
    while pc.iceConnectionState not in ["connected", "completed"]:
        await asyncio.sleep(0.1)


class WebRTCPublisherNode(Node):
    def __init__(self):
        super().__init__("webrtc_publisher_node")
        self.session_id = None
        self.pc = None
        self.cam_player1 = None
        self.cam_player2 = None

        # ROS2 service
        self.srv = self.create_service(
            Trigger, "get_publisher_session_id", self.get_session_id_callback
        )

        self.get_logger().info("Starting Cloudflare publisher...")
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.publish_stream())

    async def generate_complete_offer(self, pc):
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.1)
        return pc.localDescription

    async def publish_stream(self):
        try:
            self.pc = RTCPeerConnection()
            
            @self.pc.on("iceconnectionstatechange")
            async def on_ice_state_change():
                self.get_logger().info(f"ICE state: {self.pc.iceConnectionState}")


            self.cam_player1 = MediaPlayer("/dev/video2", format="v4l2",options={"video_size": "640x480"})
            self.cam_player2 = MediaPlayer("/dev/video4", format="v4l2",options={"video_size": "640x480"})

            # Create Cloudflare session
            async with aiohttp.ClientSession() as s:
                resp = await s.post(
                    f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/new",
                    headers={"Authorization": f"Bearer {CLOUDFLARE_API_KEY}"},
                )
                session = await resp.json()
                self.session_id = session["sessionId"]
                print("Session ID:", self.session_id)
            self.get_logger().info(
                f"✅ Stream started on Cloudflare session: {self.session_id}"
            )

            # Publish first track (camera 1)
            self.pc.addTrack(self.cam_player1.video)
            await publish_track(
                self.session_id,
                self.pc,
                [{"location": "local", "mid": "0", "trackName": "cam_up", "kind": "video"},]
            )
            await wait_for_ice_connection(self.pc)
            # Publish second track (camera 2)
            self.pc.addTrack(self.cam_player2.video)
            await publish_track(
                self.session_id,
                self.pc,
                [{"location": "local", "mid": "1", "trackName": "cam_down", "kind": "video"},]
            )
            await wait_for_ice_connection(self.pc)
            await asyncio.Future()  # keep alive

        except Exception as e:
            self.get_logger().error(f"Failed to start stream: {e}")

    def get_session_id_callback(self, request, response):
        if self.session_id is None:
            response.success = False
            response.message = "Session not ready yet"
        else:
            response.success = True
            response.message = self.session_id
        return response


def main():
    rclpy.init()
    node = WebRTCPublisherNode()
    executor = executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        loop = asyncio.get_event_loop()
        loop.run_in_executor(None, executor.spin)
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
