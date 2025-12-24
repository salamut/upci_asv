from ultralytics import YOLO
import aioredis
import asyncio
import cv2
import os
import base64

redis_url = "redis://localhost:6379"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(BASE_DIR, "yolov8n.pt")

model = YOLO(model_path)  # load a pretrained model (recommended for training)
async def main():
    redis = aioredis.from_url(redis_url)
    pubsub = redis.pubsub()
    cap = cv2.VideoCapture(0)  # open the default camera
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        results = model(frame)[0]
        annotated_frame = results.plot()
        _, img_encoded = cv2.imencode(".jpg", annotated_frame)
        base64_img = base64.b64encode(img_encoded).decode('utf-8')
        print (f"Publishing image of size: {len(base64_img)} bytes")
        await redis.publish("channel:img", base64_img)
        await asyncio.sleep(0.2)

if __name__ == "__main__":
    asyncio.run(main())