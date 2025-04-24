from pymavlink import mavutil
import time

def connect():
    print("[CONNECT] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECT] Heartbeat received")
    return master

def check_ekf_status(master):
    print("[EKF] Listening for EKF_STATUS_REPORT...")
    status_map = {
        0: "attitude",
        1: "velocity_horiz",
        2: "pos_horiz",
        3: "pos_vert",
        4: "const_pos_mode",
        5: "pred_pos_horiz",
        6: "pred_pos_vert",
        7: "gps_glitch",
        8: "accel_error"
    }

    while True:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True)
        if msg:
            flags = msg.flags
            print(f"\n[EKF] Flags: {flags} (binary: {bin(flags)})")
            for bit, name in status_map.items():
                status = "OK" if flags & (1 << bit) else "NOT OK"
                print(f"[EKF] {name}: {status}")
        time.sleep(2)

def main():
    master = connect()
    check_ekf_status(master)

if __name__ == "__main__":
    main()
