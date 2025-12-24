import aioredis
import orjson

import asyncio
async def main():
    redis = aioredis.from_url("redis://localhost:6379")
    pubsub = redis.pubsub()
    await pubsub.subscribe( "channel:mavlink")
    async for message in pubsub.listen():
        if message and message["type"] == "message":
            channel = message["channel"].decode("utf-8")
            data = message["data"].decode()
            msg = orjson.loads(data)
            print(f"Received message on {channel}: {msg}...")  

if __name__ == "__main__":
    asyncio.run(main())