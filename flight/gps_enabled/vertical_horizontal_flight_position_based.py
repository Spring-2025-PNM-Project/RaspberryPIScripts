from pymavlink import mavutil
import time
import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'capture')))
from image_capture_thread import start_image_capture, stop_image_capture


FENCE_POINTS = [
    (37.7749, -122.4194),
    (37.7749, -122.4180),
    (37.7735, -122.4180),
    (37.7735, -122.4194)
]

def connect():
    print("[CONNECT] Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECT] Heartbeat received")
    return master

def wait_for_gps(master):
    print("[GPS] Waiting for GPS fix...")
    while True:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True)
        if msg.fix_type >= 3:
            print(f"[GPS] Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
            break
        print(f"[GPS] Waiting... Fix type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
        time.sleep(1)

def wait_for_ekf(master):
    print("[EKF] Waiting for EKF position estimation...")
    while True:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True)
        if msg:
            flags = msg.flags
            print(f"[EKF] Flags: {flags} (binary: {bin(flags)})")
            attitude = bool(flags & (1 << 0))
            horiz_pos = bool(flags & (1 << 2))
            vert_pos = bool(flags & (1 << 3))
            print(f"[EKF] Attitude: {'OK' if attitude else 'NOT OK'}")
            print(f"[EKF] Pos Horiz: {'OK' if horiz_pos else 'NOT OK'}")
            print(f"[EKF] Pos Vert: {'OK' if vert_pos else 'NOT OK'}")
            if attitude and horiz_pos and vert_pos:
                print("[EKF] EKF ready for arming.")
                break
        time.sleep(1)

def upload_geofence(master):
    print("[FENCE] Uploading geofence...")
    master.mav.param_set_send(master.target_system, master.target_component, b'FENCE_TYPE', 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.3)
    master.mav.param_set_send(master.target_system, master.target_component, b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.3)
    for i, (lat, lon) in enumerate(FENCE_POINTS):
        master.mav.fence_point_send(
            master.target_system, master.target_component,
            i, len(FENCE_POINTS),
            int(lat * 1e7),
            int(lon * 1e7)
        )
        print(f"[FENCE] Point {i+1} sent: lat={lat}, lon={lon}")
        time.sleep(0.2)

def set_mode(master, mode="GUIDED"):
    print(f"[MODE] Setting mode to {mode}...")
    master.set_mode_apm(mode)
    time.sleep(2)
    print(f"[MODE] Mode confirmed: {master.flightmode}")

def arm(master):
    print("[ARM] Sending arm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("[ARM] Motors armed successfully")

def takeoff(master, alt=3):
    print(f"[TAKEOFF] Commanding takeoff to {alt} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    timeout = time.time() + 10
    while time.time() < timeout:
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print(f"[TAKEOFF_ACK] Result = {msg.result}")
            break
        time.sleep(0.2)

def monitor_altitude(master, target_alt=3.0):
    print("[ALTITUDE] Monitoring climb...")
    timeout = time.time() + 20
    while time.time() < timeout:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f"[ALTITUDE] Current: {alt:.2f} m")
            if alt >= target_alt:
                print("[ALTITUDE] Target altitude reached.")
                return
        time.sleep(0.3)
    print("[ALTITUDE] Timeout before reaching altitude.")

def move_forward_global(master, distance_meters=5):
    print("[GPS] Retrieving current GPS position...")
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if msg:
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        print(f"[GPS] Current location: lat={current_lat}, lon={current_lon}")

        delta_lat = distance_meters / 111139.0  # Rough lat meter conversion
        target_lat = current_lat + delta_lat
        target_lon = current_lon

        print(f"[MOVE] Sending drone VIA GLOBAL to lat={target_lat}, lon={target_lon}")
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b0000111111111000),  # only position is valid
            int(target_lat * 1e7),
            int(target_lon * 1e7),
            3,     # maintain 3m altitude
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0      # yaw, yaw_rate
        )
        time.sleep(5)
        
def move_forward_local(master, distance_meters=3):
    print("[GPS] Retrieving current GPS position...")
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    if msg:
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        print(f"[GPS] Current location: lat={current_lat}, lon={current_lon}")

        delta_lat = distance_meters / 111139.0  # Rough lat meter conversion
        target_lat = current_lat + delta_lat
        target_lon = current_lon
        print(f"[MOVE] Sending drone VIA NED lat={target_lat}, lon={target_lon}")
        type_mask = int(0b010111111000) # enables position + yaw

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        distance_meters, 0, -3,       # x, y, z
        0, 0, 0,       # vx, vy, vz
        0, 0, 0,       # afx, afy, afz
        0, 0     # yaw, yaw_rate (optional, but may help stabilize direction)
        ))

        time.sleep(1)

def land(master):
    print("[LAND] Sending LAND command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(5)
    print("[LAND] LAND command complete.")

def disarm(master):
    print("[DISARM] Sending disarm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("[DISARM] Motors disarmed")

def reset_mode_post_landing(master, target_mode="STABILIZE"):
    print("[RESET] Checking and resetting mode after landing...")
    hb = master.recv_match(type="HEARTBEAT", blocking=True)
    current_mode = master.flightmode
    print(f"[RESET] Current mode: {current_mode}")
    if current_mode == "LAND":
        print(f"[RESET] Mode is LAND. Changing to {target_mode}...")
        master.set_mode_apm(target_mode)
        time.sleep(2)
        print(f"[RESET] Mode changed to: {master.flightmode}")

def main():
    master = connect()
    wait_for_gps(master)
    upload_geofence(master)
    wait_for_ekf(master)
    set_mode(master, "GUIDED")
    arm(master)
    time.sleep(2)
    takeoff(master, alt=3)
    monitor_altitude(master, target_alt=3)
    #move_forward_global(master, distance_meters=2)
    move_forward_local(master, distance_meters = -3)
    land(master)
    disarm(master)
    reset_mode_post_landing(master)

if __name__ == "__main__":
    main()
