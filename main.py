import time
import requests
import subprocess

drone_id = "1"
POLL_INTERVAL = 2  # seconds
API_ENDPOINT = f"https://api.meritdrone.site/drone/{drone_id}/info"

def poll_for_instructions():
    try:
        drone_status = {
                "location": {
                    "latitude": 0 / 1e7,
                    "longitude": 0 / 1e7,
                    "altitude": 0 / 1000.0
                },
                "timestamp": int(time.time()),
                "status": "grounded",
            }
        response = requests.post(API_ENDPOINT, json = drone_status)
        print(response.text)
        if response.status_code == 200:
            instructions = response.json().get("instructions", [])
            return instructions
    except Exception as e:
        print(f"[ERROR] Poll failed: {e}")
    return []

def main():
    print("[MAIN] Starting drone instruction poller...")
    while True:
        instructions = poll_for_instructions()
        if "takeoff" in instructions:
            print("[MAIN] 'takeoff' instruction received. Launching flight plan...")
            subprocess.run(["/usr/bin/python3", "flight/gps_enabled/vertical_horizontal_flight_velocity_based.py"])
            print("[MAIN] Flight plan execution finished.")
        else:
            print("[MAIN] No actionable instruction.")
        time.sleep(POLL_INTERVAL)

if __name__ == "__main__":
    main()
