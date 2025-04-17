from picamera2 import Picamera2
import time, requests
from config import CAMERA_IMAGE_PATH, UPLOAD_URL

def run_image_pipeline():
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)
    picam2.start()
    time.sleep(3)
    picam2.capture_file(CAMERA_IMAGE_PATH)
    picam2.close()
    print("Image captured.")

    with open(CAMERA_IMAGE_PATH, "rb") as f:
        files = {"file": ("sample.jpg", f, "image/jpeg")}
        res = requests.post(UPLOAD_URL, files=files)

    try:
        print(res.json())
    except:
        print(res.text)
