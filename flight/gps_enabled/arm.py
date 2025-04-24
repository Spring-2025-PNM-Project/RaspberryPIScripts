from pymavlink import mavutil
import time

FENCE_POINTS = [
    (37.7749, -122.4194),
    (37.7749, -122.4180),
    (37.7735, -122.4180),
    (37.7735, -122.4194)
]

def connect():
    print("[CONNECTED] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECTED] Heartbeat received")
    return master

def wait_for_gps(master):
    print("[GPS] Waiting for GPS fix...")
    while True:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True)
        if msg.fix_type >= 3:
            print(f"[GPS] Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
            print("[GPS] GPS fix acquired.")
            return
        else:
            print(f"[GPS] Waiting... Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
            time.sleep(1)

def upload_fence(master):
    print("[FENCE] Enabling fence and sending polygon...")
    master.mav.param_set_send(master.target_system, master.target_component, b'FENCE_TYPE', 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.5)
    master.mav.param_set_send(master.target_system, master.target_component, b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.5)
    print(f"[FENCE] Sending {len(FENCE_POINTS)} points...")
    for i, (lat, lon) in enumerate(FENCE_POINTS):
        master.mav.fence_point_send(
            master.target_system,
            master.target_component,
            i,
            len(FENCE_POINTS),
            int(lat * 1e7),
            int(lon * 1e7)
        )
        print(f"[FENCE] Point {i+1}: lat={lat}, lon={lon}")
        time.sleep(0.2)

def set_mode_guided(master):
    print("[MODE] Setting mode to GUIDED...")
    master.set_mode_apm("GUIDED")
    time.sleep(2)
    print(f"[MODE] Current mode: {master.flightmode}")

def wait_for_ekf_stable(master):
    print("[EKF] Waiting for required EKF stability (attitude, horiz+vert pos/vel)...")
    required_bits = [0, 1, 2, 3]
    while True:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True)
        if not msg:
            continue
        flags = msg.flags
        binary = f"{flags:#010b}"
        print(f"[EKF] Flags: {flags} (binary: {binary})")
        conditions = {
            "attitude": bool(flags & (1 << 0)),
            "velocity_horiz": bool(flags & (1 << 1)),
            "pos_horiz": bool(flags & (1 << 2)),
            "pos_vert": bool(flags & (1 << 3))
        }
        for k, v in conditions.items():
            print(f"[EKF] {k}: {'OK' if v else 'NOT OK'}")
        if all(conditions.values()):
            print("[EKF] Required EKF checks passed.")
            return
        print("[EKF] Waiting for required checks to pass...")
        time.sleep(1)
        
def arm(master):
    print("[ARM] Attempting to arm the drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"[ARM_ACK] Result = {msg.result}")
            if msg.result == 0:
                print("[ARM] Drone armed successfully.")
            else:
                print("[ARM] Failed to arm. Pre-arm check failure.")
            return msg.result
        time.sleep(0.2)
    print("[ARM] No response from flight controller.")
    return None

def listen_statustext(master, duration=5):
    print("[STATUS] Listening for STATUSTEXT messages...")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"[STATUSTEXT] {msg.text}")
        time.sleep(0.2)

def main():
    master = connect()
    wait_for_gps(master)
    upload_fence(master)
    set_mode_guided(master)
    wait_for_ekf_stable(master)
    result = arm(master)
    time.sleep(4)
    listen_statustext(master)
    if result != 0:
        print("[DEBUG] Arming failed. See logs above for more info.")

if __name__ == "__main__":
    main()
