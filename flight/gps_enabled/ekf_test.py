from pymavlink import mavutil
import time

EKF_FLAGS = {
    0: "attitude",
    1: "velocity_horiz",
    2: "pos_horiz",
    3: "pos_vert",
    4: "const_pos_mode",
    5: "pred_pos_horiz",
    6: "pred_pos_vert",
    7: "terrain_alt"
}

def connect():
    print("[CONNECT] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECT] Heartbeat received")
    return master

def wait_for_gps_fix(master):
    print("[GPS] Waiting for GPS 3D fix...")
    while True:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True)
        if msg and msg.fix_type >= 3:
            print(f"[GPS] Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
            break
        else:
            print(f"[GPS] Waiting... Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
        time.sleep(1)

def print_ekf_flags(flags):
    print(f"[EKF] Flags: {flags} (binary: {bin(flags)})")
    for bit, name in EKF_FLAGS.items():
        status = "OK" if flags & (1 << bit) else "NOT OK"
        print(f"[EKF] {name}: {status}")

def wait_for_full_ekf_stability(master):
    print("[EKF] Waiting for full EKF stability...")
    while True:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True)
        if msg:
            print_ekf_flags(msg.flags)

            if all((msg.flags & (1 << bit)) for bit in [0,1,2,3,5,6]):
                print("[EKF] EKF is fully stable for arming and navigation.")
                break
            else:
                print("[EKF] EKF still stabilizing...\n")
        time.sleep(2)

def main():
    master = connect()
    wait_for_gps_fix(master)
    wait_for_full_ekf_stability(master)

if __name__ == "__main__":
    main()
