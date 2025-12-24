from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
import uvicorn
import asyncio
import aioredis
import orjson
import os
import hashlib
import base64
import io


redis_url = "redis://localhost:6379"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class BackendApp:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
        self.data = {}
        self.waypoints = []
        self.img = None
        self.img_hash = None

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except WebSocketDisconnect:
                disconnected.append(connection)
        for connection in disconnected:
            self.active_connections.remove(connection)

    async def realtime_broadcast(self):
        while True:
            msg_json = orjson.dumps(self.data).decode("utf-8")
            await self.broadcast(msg_json)
            await asyncio.sleep(0.02)

    async def redis_listener(self):
        redis = aioredis.from_url(redis_url)
        pubsub = redis.pubsub()

        await pubsub.subscribe("channel:mavlink", "channel:img")
        async for message in pubsub.listen():
            if message and message["type"] == "message":
                channel = message["channel"].decode("utf-8")
                data = message["data"].decode()
                if channel == "channel:mavlink":
                    self.data = orjson.loads(data)
                    # print(f"Received MAVLink data: {self.data}")
                elif channel == "channel:img":
                    img = base64.b64decode(data)
                    self.img = img
                    self.img_hash = hashlib.md5(img).hexdigest()
                await asyncio.sleep(0.01)


backend = BackendApp()

async def lifespan(app: FastAPI):
    print("Starting up...")
    task1 = asyncio.create_task(backend.redis_listener())
    task2 = asyncio.create_task(backend.realtime_broadcast())
    yield
    print("Shutting down...")
    task1.cancel()
    task2.cancel()


app = FastAPI(lifespan=lifespan)

@app.get("/")
async def get():
    file_path = os.path.join(BASE_DIR, "index.html")
    return FileResponse(file_path)

@app.get("/image")
async def get_image():
    if not backend.img:
        return Response(status_code=404, content="No image available")
    img = io.BytesIO(backend.img)
    headers = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "Expires": "0"
    }
    return Response(content=img.getvalue(), media_type="image/jpeg", headers=headers)


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


if __name__ == "__main__":
    uvicorn.run(
        "backend:app",
        host="0.0.0.0",
        port=8080,
        reload=True,
    )
