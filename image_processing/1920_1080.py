import base64
import time
import requests
from picamera2 import Picamera2
from time import sleep

# Drone config
drone_id = "1"
api_url = f"https://api.meritdrone.site/drone/{drone_id}/info"

# STEP 1: Capture image using PiCamera
print("[CAMERA] Initializing PiCamera2...")
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(config)
picam2.start()
print("[CAMERA] Warming up...")
picam2.capture_metadata()
sleep(0.2)

image_path = "image.jpg"
picam2.capture_file(image_path)
picam2.close()
print(f"[CAMERA] Image captured: {image_path}")

# STEP 2: Encode image as base64
with open(image_path, "rb") as f:
    image_bytes = f.read()
    encoded_image = base64.b64encode(image_bytes).decode("utf-8")

# STEP 3: Build payload
drone_status = {
    "location": {
        "latitude": 37.3352,
        "longitude": -121.8811,
        "altitude": 5
    },
    "timestamp": int(time.time()),
    "status": "flying",
    "image": encoded_image
}

# STEP 4: Send POST request
print("[UPLOAD] Sending data to backend...")
response = requests.post(api_url, json=drone_status)

# STEP 5: Handle response
print(f"[UPLOAD] Status Code: {response.status_code}")
try:
    print("[UPLOAD] Response JSON:", response.json())
except Exception:
    print("[UPLOAD] Non-JSON response:", response.text)
