import asyncio
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer

CLOUDFLARE_APP_ID = "d5493d021807cc99487480f8c0fc0872"
CLOUDFLARE_TOKEN = "3e45e9edfaeaf0db5399b269cba3bc3b564f1c37c827120edc5f96be0c7118e9"
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
                "Authorization": f"Bearer {CLOUDFLARE_TOKEN}",
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


async def main():
    pc = RTCPeerConnection()

    @pc.on("iceconnectionstatechange")
    async def on_ice_state_change():
        print("ICE state:", pc.iceConnectionState)

    # Step 1. Create Cloudflare session
    async with aiohttp.ClientSession() as s:
        resp = await s.post(
            f"{CLOUDFLARE_API_BASE}/apps/{CLOUDFLARE_APP_ID}/sessions/new",
            headers={"Authorization": f"Bearer {CLOUDFLARE_TOKEN}"},
        )
        session = await resp.json()
        session_id = session["sessionId"]
        print("Session ID:", session_id)

    # Step 2. Publish first track (camera)
    player1 = MediaPlayer(
        "/dev/video2", format="v4l2", options={"video_size": "640x480"}
    )
    pc.addTrack(player1.video)
    await publish_track(
        session_id,
        pc,
        [{"location": "local", "mid": "0", "trackName": "/cam/main", "kind": "video"}],
    )

    await wait_for_ice_connection(pc)

    print("✅ First track connected!")

    # Step 3. Publish second track (e.g. screen or another camera)
    player2 = MediaPlayer(
        "/dev/video3", format="v4l2", options={"video_size": "640x480"}
    )
    pc.addTrack(player2.video)
    await publish_track(
        session_id,
        pc,
        [
            {
                "location": "local",
                "mid": "1",
                "trackName": "/cam/secondary",
                "kind": "video",
            }
        ],
    )

    print("✅ Second track added successfully!")
    await asyncio.sleep(3600)  # Keep session alive


asyncio.run(main())
