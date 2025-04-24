from pymavlink import mavutil
import time

def connect():
    print("[CONNECT] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECTED] Heartbeat received")
    return master

def monitor_gps(master, duration=30):
    print(f"[MONITOR] Listening for GPS updates for {duration} seconds...")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg:
            fix_type = msg.fix_type  # 0: no fix, 1: no GPS, 2: 2D fix, 3: 3D fix
            sats = msg.satellites_visible
            eph = msg.eph / 100.0  # horizontal dilution of precision (HDOP)

            print(f"[GPS] Fix Type: {fix_type}, Satellites: {sats}, HDOP: {eph:.2f}")
            if fix_type >= 3 and sats >= 6 and eph < 2.0:
                print("? Good GPS lock")
            elif fix_type >= 2:
                print("?? Limited GPS (2D), or weak signal")
            else:
                print("? No usable GPS fix")
        else:
            print("?? No GPS message received")
        time.sleep(1)

def main():
    master = connect()
    monitor_gps(master, duration=60)

if __name__ == "__main__":
    main()
