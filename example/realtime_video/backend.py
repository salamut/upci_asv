import os
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
import httpx
from dotenv import load_dotenv
from pydantic import BaseModel

load_dotenv()

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

CLOUDFLARE_APP_ID = "d5493d021807cc99487480f8c0fc0872"
CLOUDFLARE_API_KEY = "3e45e9edfaeaf0db5399b269cba3bc3b564f1c37c827120edc5f96be0c7118e9"
CLOUDFLARE_API_BASE = "https://rtc.live.cloudflare.com/v1"
HEADERS = {
    "Authorization": f"Bearer {CLOUDFLARE_API_KEY}",
    "Content-Type": "application/json",
}

class localDescription(BaseModel):
    type: str
    sdp: str
class trackInfo(BaseModel):
    location: str
    sessionId: str
    trackName: str
class payload(BaseModel):
    sessionDescription: localDescription
    tracks: list[trackInfo]



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
        viewer_session_id = session["sessionId"]

        # Return only ephemeral info
        return {
            "appId": CLOUDFLARE_APP_ID,
            "viewerSessionId": viewer_session_id,
        }

@app.post("/tracks/{session_id}")
async def post_tracks(session_id: str, payload: payload):
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

@app.get("/")
def index():
    return {"message": "Use /static/index.html to view"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
