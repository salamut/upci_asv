import asyncio
import aiohttp
import cv2
import numpy as np
import json
from av import VideoFrame
import uuid
import requests
from aiortc import RTCConfiguration, RTCPeerConnection, RTCIceServer, RTCSessionDescription,MediaStreamTrack,VideoStreamTrack, AudioStreamTrack
from aiortc.contrib.media import MediaPlayer
from aiortc.sdp import candidate_from_sdp
from fractions import Fraction


# Name : realtime-video
# App ID : d5493d021807cc99487480f8c0fc0872
# API Token : 3e45e9edfaeaf0db5399b269cba3bc3b564f1c37c827120edc5f96be0c7118e9

CLOUDFLARE_APP_ID = "d5493d021807cc99487480f8c0fc0872"
CLOUDFLARE_API_KEY = "3e45e9edfaeaf0db5399b269cba3bc3b564f1c37c827120edc5f96be0c7118e9"
CLOUDFLARE_API_BASE = "https://rtc.live.cloudflare.com/v1"

class CameraStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, fps: int = 30):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera not opened")
        self.frame_time = 1 / fps
        self.last_pts = 0
        self.time_base = 1 / 90000  # WebRTC clock, 90 kHz

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Camera read failed")

        # convert to VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")

        # generate pts manually (in 90 kHz units)
        self.last_pts += int(90000 * self.frame_time)
        video_frame.pts = self.last_pts
        video_frame.time_base = Fraction(1, 90000)

        # optional: small async yield so event loop stays responsive
        await asyncio.sleep(self.frame_time)
        return video_frame

async def generate_complete_offer(pc):
    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    # Wait for ICE gathering to complete
    while True:
        if pc.iceGatheringState == "complete":
            break
        await asyncio.sleep(0.1)

    return pc.localDescription

async def publish():
    pc = RTCPeerConnection()
    media_stream = CameraStreamTrack()
    pc.addTrack(media_stream)
    frame = await media_stream.recv()
    print(f"Frame : {frame.width}x{frame.height}, pts={frame.pts}, tb={frame.time_base}")

    local_desc = await generate_complete_offer(pc)


    # Create Cloudflare Realtime session
    headers = {
        "Authorization": f"Bearer {CLOUDFLARE_API_KEY}",
        "Content-Type": "application/json"
    }
    resp = requests.post(
        f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/new",
        headers=headers
    )
    session_info = resp.json()
    session_id = session_info["sessionId"]

    # Send track to Cloudflare
    payload = {
        "sessionDescription": {
            "type": local_desc.type,
            "sdp": local_desc.sdp
        },
        "autoDiscover": True
    }
    resp2 = requests.post(
        f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/{session_id}/tracks/new",
        headers=headers,
        json=payload
    )
    print("Track publish response:", resp2.json())

    answer = resp2.json()["sessionDescription"]
    await pc.setRemoteDescription(
        RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
    )

    print("✅ Stream started on Cloudflare session:", session_id)
    await asyncio.Future()  # keep alive

if __name__ == "__main__":
    asyncio.run(publish())