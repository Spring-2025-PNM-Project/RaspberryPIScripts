from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def disable_fence_and_gps(master):
    print("Disabling FENCE_ENABLE and GPS_REQUIRED...")
    master.mav.param_set_send(master.target_system, master.target_component, b'FENCE_ENABLE', float(0), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    master.mav.param_set_send(master.target_system, master.target_component, b'GPS_REQUIRED', float(0), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(2)

def set_mode_alt_hold(master):
    print("Setting mode to ALT_HOLD...")
    master.set_mode_apm("ALT_HOLD")
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and master.flightmode == 'ALT_HOLD':
            print("Flight mode confirmed: ALT_HOLD")
            return
        time.sleep(0.2)
    print("Warning: ALT_HOLD not confirmed")

def arm(master):
    print("Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed")

def override_throttle(master, pwm):
    print(f"Sending throttle override: PWM={pwm}")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def throttle_ramp(master, start=1100, end=1500, step=50, delay=3):
    print("Ramping throttle...")
    for pwm in range(start, end + 1, step):
        override_throttle(master, pwm)
        time.sleep(delay)
    print("Throttle ramp complete. Holding...")
    time.sleep(3)  # Simulate hover at final throttle

def emergency_land(master):
    print("Initiating emergency landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            print(f"LAND ACK: Result = {msg.result}")
            break
        time.sleep(0.2)
    print("Landing command sent")

def disarm(master):
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed")

def main():
    master = connect()
    disable_fence_and_gps(master)
    set_mode_alt_hold(master)
    arm(master)
    time.sleep(5)  # Let sensors stabilize
    throttle_ramp(master)
    emergency_land(master)
    time.sleep(3)
    disarm(master)

if __name__ == "__main__":
    main()
