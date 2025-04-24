from pymavlink import mavutil
import time

def connect():
    print("[CONNECT] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECT] Heartbeat received")
    return master

def listen_barometer(master, duration=30):
    print("[BAROMETER] Listening for pressure data...")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type == "SCALED_PRESSURE":
                print(f"[BARO1] Time: {msg.time_boot_ms} ms | Press: {msg.press_abs:.2f} mbar | Temp: {msg.temperature/100.0:.2f} C")
            elif msg_type == "SCALED_PRESSURE2":
                print(f"[BARO2] Time: {msg.time_boot_ms} ms | Press: {msg.press_abs:.2f} mbar | Temp: {msg.temperature/100.0:.2f} C")
            elif msg_type == "SCALED_PRESSURE3":
                print(f"[BARO3] Time: {msg.time_boot_ms} ms | Press: {msg.press_abs:.2f} mbar | Temp: {msg.temperature/100.0:.2f} C")
        time.sleep(0.1)

def main():
    master = connect()
    listen_barometer(master, duration=30)

if __name__ == "__main__":
    main()
