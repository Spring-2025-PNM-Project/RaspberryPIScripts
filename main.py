from network.status_sender import send_status
from flight.controller import execute_flight_plan
from image_processing.capture import run_image_pipeline
import time

def main_loop():
    response_data = send_status()

    if response_data and "flight_plan" in response_data:
        print("Executing flight plan...")
        execute_flight_plan(response_data["flight_plan"])

        print("Running image capture pipeline...")
        run_image_pipeline()

        print("Mission complete.")
        return True 
    return False

while True:
    if main_loop():
        break
    time.sleep(5)
