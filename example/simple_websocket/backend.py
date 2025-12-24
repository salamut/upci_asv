import websockets
import random
import asyncio
import json

data = {
    "param1": 0,
    "param2": 0,
}

connected_client = set()


async def data_updater(connected_clients):
    while True:
        data["param1"] = random.randint(0, 100)
        data["param2"] = random.randint(0, 100)
        json_data = json.dumps(data)
        to_remove = set()
        for client in connected_clients:
            try:
                
                await client.send(json_data)
            except websockets.exceptions.ConnectionClosed:
                to_remove.remove(client)
            connected_clients.difference_update(to_remove)
        await asyncio.sleep(0.1)


async def ws_handler(websocket):
    """Handle new WebSocket clients."""
    print("New client connected")
    connected_client.add(websocket)
    try:
        async for message in websocket:
            print(f"Received from client: {message}")
    finally:
        print("Client disconnected")
        connected_client.remove(websocket)


async def main():
    async with websockets.serve(ws_handler, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await data_updater(connected_client) 

if __name__ == "__main__":
    asyncio.run(main())