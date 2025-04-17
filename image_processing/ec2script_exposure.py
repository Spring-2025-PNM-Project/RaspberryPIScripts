from picamera2 import Picamera2
from time import sleep
import requests
import os

# STEP 1: Initialize and warm up camera
picam2 = Picamera2()

# Optional: Configure preview size or mode (defaults work fine for most)
picam2.configure(picam2.create_still_configuration())
picam2.start()

print("[?] Warming up camera and waiting for autofocus...")
sleep(3)  # Give time for camera to auto-focus and adjust exposure

# Optional: Discard the first capture (helps with buffering)
picam2.capture_metadata()  # Trigger metadata to stabilize AE/AF
sleep(1)

# STEP 2: Capture stabilized image
image_path = "image.jpg"
picam2.capture_file(image_path)
picam2.close()
print("[?] Image captured.")

# STEP 3: Upload
url = "https://api.meritdrone.site/upload/"

with open(image_path, "rb") as f:
    files = {"file": ("sample.jpg", f, "image/jpeg")}
    response = requests.post(url, files=files)

# STEP 4: Print Result
try:
    print(response.json())
except Exception:
    print("Non-JSON response:", response.text)

# STEP 5: Cleanup
if os.path.exists(image_path):
    #os.remove(image_path)
    print(f"[?] Deleted temp file: {image_path}")
