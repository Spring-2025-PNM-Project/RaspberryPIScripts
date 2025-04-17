import requests, time, json
from config import DRONE_ID, API_URL

def send_status():
    print("Sending status")
    payload = {
        "drone_id": DRONE_ID,
        "location": {"latitude": 0, "longitude": 0, "altitude": 0},
        "timestamp": int(time.time()),
        "status": "on"
    }

    try:
        res = requests.post(API_URL, json=payload)
        res.raise_for_status()
        print(res.json())
        return res.json()
    except Exception as e:
        print("[?] Status error:", e)
        return None
