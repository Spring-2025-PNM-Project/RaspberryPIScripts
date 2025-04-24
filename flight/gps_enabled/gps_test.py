from pymavlink import mavutil
import time

def connect_to_vehicle(port='/dev/ttyAMA0', baud=57600):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print("Connected to vehicle.")
    return master

def check_ekf_status(master):
    print("Checking EKF status...")
    start_time = time.time()
    while time.time() - start_time < 30:
        msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
        if msg:
            flags = msg.flags
            if flags & 0x01:
                print("EKF is OK.")
                break
            else:
                print("EKF is not ready.")
        else:
            print("No EKF status report received.")
        time.sleep(1)

def check_gps_status(master):
    print("Checking GPS status...")
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg:
        print(f"GPS fix type: {msg.fix_type}, Satellites visible: {msg.satellites_visible}")
    else:
        print("No GPS status received.")

def check_compass_status(master):
    print("Checking compass status...")
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        sensors_enabled = msg.onboard_control_sensors_enabled
        if sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG:
            print("Compass is enabled.")
        else:
            print("Compass is not enabled.")
    else:
        print("No system status received.")

if __name__ == "__main__":
    master = connect_to_vehicle()
    check_ekf_status(master)
    check_gps_status(master)
    check_compass_status(master)
