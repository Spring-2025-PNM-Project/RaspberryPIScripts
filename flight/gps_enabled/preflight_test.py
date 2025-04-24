from pymavlink import mavutil
import time

def connect_pixhawk(port='/dev/ttyAMA0', baud=57600):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    return master

def disable_fence(master):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_ENABLE',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )

def wait_for_gps_fix(master, min_fix_type=2):
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg and msg.fix_type >= min_fix_type:
            print(f"GPS fix acquired: Type={msg.fix_type}, Satellites={msg.satellites_visible}")
            break
        print("Waiting for GPS fix...")
        time.sleep(1)

def set_guided_mode(master):
    master.set_mode_apm("GUIDED")

def wait_for_ekf_ready(master):
    while True:
        msg = master.recv_match(type='SYS_STATUS', blocking=True)
        if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO:
            print("EKF is ready")
            return
        print("Waiting for EKF to stabilize...")
        return
        time.sleep(1)

def check_battery(master, threshold=10.5):
    msg = master.recv_match(type='SYS_STATUS', blocking=True)
    voltage = msg.voltage_battery / 1000.0
    print(f"Battery voltage: {voltage:.2f}V")
    if voltage < threshold:
        raise RuntimeError("Battery voltage too low for safe flight.")
        
def enable_fence(master):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_ENABLE',
        float(1),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_TYPE',
        float(3),  # Polygon fence
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    print("Geofence enabled")
    
def upload_fence_polygon(master):
    points = [
        (int(37.3335e7), int(-121.8809e7)),
        (int(37.3335e7), int(-121.8806e7)),
        (int(37.3338e7), int(-121.8806e7)),
        (int(37.3338e7), int(-121.8809e7)),
    ]

    master.mav.param_set_send(master.target_system, master.target_component,
        b'FENCE_TOTAL', len(points), mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    for i, (lat, lon) in enumerate(points):
        master.mav.fence_point_send(
            master.target_system,
            master.target_component,
            i,
            len(points),
            lat,
            lon
        )
    print("Fence polygon uploaded")


def preflight_check():
    print("Connecting to Pixhawk...")
    master = connect_pixhawk()

    #print("Disabling geofence...")
    #disable_fence(master)
    print("Enabling geofence...")
    enable_fence(master)
    
    print("Setting up geofence")
    upload_fence_polygon(master)

    print("Waiting for GPS fix...")
    wait_for_gps_fix(master)

    print("Setting flight mode to GUIDED...")
    set_guided_mode(master)

    print("Waiting for EKF stabilization...")
    wait_for_ekf_ready(master)

    print("Checking battery voltage...")
    check_battery(master)

    print("Preflight check complete. Vehicle is ready for arming.")
    return master

if __name__ == "__main__":
    try:
        preflight_check()
    except Exception as e:
        print(f"Preflight check failed: {e}")
