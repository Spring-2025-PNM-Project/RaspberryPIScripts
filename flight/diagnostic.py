from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def check_param(master, param_name):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )

    start_time = time.time()
    timeout = 5  # seconds

    while time.time() - start_time < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            param_id = msg.param_id.strip('\x00')
            if param_id == param_name:
                print(f"[PARAM] {param_id} = {msg.param_value}")
                return msg.param_value

    print(f"ERROR: Did not receive {param_name} within timeout.")
    return None

def check_battery(master):
    msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    if msg:
        voltage = msg.voltage_battery / 1000.0
        print(f"[BATTERY] Voltage: {voltage:.2f} V")
        if voltage < 10.5:
            print("WARNING: Voltage is below 10.5V. May trigger failsafe.")
        return voltage
    else:
        print("ERROR: No SYS_STATUS message received.")
        return None

def check_ekf(master):
    print("Waiting for EKF to stabilize...")
    start_time = time.time()
    timeout = 15

    while time.time() - start_time < timeout:
        msg = master.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if msg:
            stable = msg.flags & (1 << 2)
            print(f"[EKF] Flags: {msg.flags} - {'Stable' if stable else 'Unstable'}")
            if stable:
                print("EKF stabilized with position data.")
                return True
            else:
                print("EKF not ready...")
    print("WARNING: EKF did not stabilize in time.")
    return False

def check_gps(master):
    msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=3)
    if msg:
        print(f"[GPS] Fix Type: {msg.fix_type}, Satellites: {msg.satellites_visible}")
        if msg.fix_type >= 2:
            print("GPS fix is acceptable.")
        else:
            print("WARNING: GPS fix is not ready.")
        return msg.fix_type
    else:
        print("ERROR: No GPS_RAW_INT message received.")
        return None

def check_arming_state(master):
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb:
        mode = master.flightmode
        armed = master.motors_armed()
        print(f"[HEARTBEAT] Mode: {mode}, Armed: {armed}")
        return mode, armed
    else:
        print("ERROR: No HEARTBEAT received.")
        return None, None

def main():
    master = connect()

    # Check parameters
    fence = check_param(master, 'FENCE_ENABLE')
    gps_required = check_param(master, 'GPS_REQUIRED')

    if fence is not None and fence != 0.0:
        print("WARNING: FENCE_ENABLE is active. May block flight without GPS.")
    if gps_required is not None and gps_required != 0.0:
        print("WARNING: GPS_REQUIRED is active. May block arming or mode changes.")

    # Sensor readiness
    check_gps(master)
    check_battery(master)
    check_ekf(master)
    check_arming_state(master)

    print("Preflight diagnostics complete.")

if __name__ == "__main__":
    main()
