import asyncio
import websockets
from mavlink_handler import mavlink_broadcast

# Track active WebSocket clients
connected_clients = set()

async def ws_handler(websocket):
    """Handle new WebSocket clients."""
    print("New client connected")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            print(f"Received from client: {message}")
    finally:
        print("Client disconnected")
        connected_clients.remove(websocket)

async def main():
    async with websockets.serve(ws_handler, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await mavlink_broadcast(connected_clients) 

if __name__ == "__main__":
    asyncio.run(main())
