import threading
import base64
import time
import requests
from picamera2 import Picamera2

drone_id = "1"
api_url = f"https://api.meritdrone.site/drone/{drone_id}/info"
image_thread = None
stop_event = threading.Event()

# Shared GPS dictionary (populated by flight script)
gps_data = {
    "lat": None,
    "lon": None,
    "alt": None
}

def capture_and_send_images():
    print("[THREAD] Image capture thread started.")
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1920, 1080)})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.1)

    while not stop_event.is_set():
        image_path = "image.jpg"
        picam2.capture_file(image_path)

        try:
            with open(image_path, "rb") as f:
                image_bytes = f.read()
                encoded_image = base64.b64encode(image_bytes).decode("utf-8")
        except Exception as e:
            print(f"[THREAD] Image encoding failed: {e}")
            continue

        if None in gps_data.values():
            print("[THREAD] GPS data not ready, skipping image...")
            time.sleep(0.5)
            continue

        drone_status = {
            "location": {
                "latitude": gps_data["lat"],
                "longitude": gps_data["lon"],
                "altitude": gps_data["alt"]
            },
            "timestamp": int(time.time()),
            "status": "flying",
            "image": encoded_image
        }

        try:
            response = requests.post(api_url, json=drone_status)
            print(f"[THREAD] Sent image, status: {response.status_code}")
        except Exception as e:
            print(f"[THREAD] Upload failed: {e}")
        time.sleep(0.5)

    picam2.close()
    print("[THREAD] Image capture thread stopped.")

def start_image_capture():
    global image_thread, stop_event
    stop_event.clear()
    image_thread = threading.Thread(target=capture_and_send_images)
    image_thread.start()

def stop_image_capture():
    global stop_event, image_thread
    stop_event.set()
    if image_thread:
        image_thread.join()
