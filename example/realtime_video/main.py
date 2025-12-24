import asyncio
import aiohttp
import cv2
import numpy as np
import json
from av import VideoFrame
import uuid
import requests
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCIceServer,
    RTCSessionDescription,
    MediaStreamTrack,
    VideoStreamTrack,
    AudioStreamTrack,
)
from aiortc.contrib.media import MediaPlayer
from aiortc.sdp import candidate_from_sdp
from fractions import Fraction


CLOUDFLARE_APP_ID = "d5493d021807cc99487480f8c0fc0872"
CLOUDFLARE_API_KEY = "3e45e9edfaeaf0db5399b269cba3bc3b564f1c37c827120edc5f96be0c7118e9"
CLOUDFLARE_API_BASE = "https://rtc.live.cloudflare.com/v1"



async def wait_ice_gathering_complete(pc):
    async def wait_ice_complete():
        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.1)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    await wait_ice_complete()
    return


async def publish():
    pc = RTCPeerConnection()
    
    media_stream = MediaPlayer('/dev/video0')
    pc.addTrack(media_stream.video)

    await wait_ice_gathering_complete(pc)

    # Create Cloudflare Realtime session for
    headers = {
        "Authorization": f"Bearer {CLOUDFLARE_API_KEY}",
        "Content-Type": "application/json",
    }
    resp = requests.post(
        f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/new", headers=headers
    )
    session_info = resp.json()
    session_id = session_info["sessionId"]


    # Send track to Cloudflare
    payload = {
        "sessionDescription": {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp},
        "tracks": [
            {
                "location": "local",
                "mid": "0",
                "trackName": "cam_upper",
                "kind": "video"
            },
        ],
    }

    resp2 = requests.post(
        f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/{session_id}/tracks/new",
        headers=headers,
        json=payload,
    )
    # print("Track publish response:", resp2.json())

    answer = resp2.json()["sessionDescription"]
    await pc.setRemoteDescription(
        RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
    )


    # await wait_for_connection(pc)
    print("✅ Stream started on Cloudflare session:", session_id)
    await asyncio.Future()  # keep alive


if __name__ == "__main__":
    asyncio.run(publish())
