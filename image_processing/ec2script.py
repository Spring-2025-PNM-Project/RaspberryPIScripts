from picamera2 import Picamera2
from time import sleep
import requests
import os

# STEP 1: Capture Image
picam2 = Picamera2()
picam2.start()
sleep(2)
image_path = "image.jpg"
picam2.capture_file(image_path)
picam2.close()

# STEP 2: Upload
url = "https://api.meritdrone.site/upload/"

with open(image_path, "rb") as f:
    files = {"file": ("sample.jpg", f, "image/jpeg")}
    response = requests.post(url, files=files)

# STEP 3: Print Result and Delete
try:
    print(response.json())
except Exception:
    print("Non-JSON response:", response.text)

if os.path.exists(image_path):
    #os.remove(image_path)
    print(f"[?] Deleted temp file: {image_path}")
