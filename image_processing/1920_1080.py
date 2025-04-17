from picamera2 import Picamera2
from time import sleep
import requests
import os

# STEP 1: Initialize camera with 1920x1080 config
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(config)
picam2.start()

print("[?] Warming up camera...")
sleep(3)  # Let autofocus/exposure stabilize

# Optional buffer flush
picam2.capture_metadata()
sleep(1)

# STEP 2: Capture image
image_path = "image.jpg"
picam2.capture_file(image_path)
picam2.close()
print(f"[?] Image captured and saved as {image_path}")

# STEP 3: Upload
url = "https://api.meritdrone.site/upload/"

with open(image_path, "rb") as f:
    files = {"file": ("sample.jpg", f, "image/jpeg")}
    response = requests.post(url, files=files)

# STEP 4: Print result
try:
    print(response.json())
except Exception:
    print("Non-JSON response:", response.text)
