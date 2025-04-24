from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def disable_fence(master):
    print("Disabling fencing...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_ENABLE',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    while True:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if msg and msg.param_id == 'FENCE_ENABLE':
            if int(msg.param_value) == 0:
                print("FENCE_ENABLE confirmed disabled")
                break

def disable_gps_check(master):
    print("Disabling GPS requirement...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'GPS_REQUIRED',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )

def set_mode_alt_hold(master):
    master.set_mode_apm("ALT_HOLD")
    print("Mode set to ALT_HOLD")

def arm(master):
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed")

def climb_to_altitude(master, target_alt_m=1.0, duration=5):
    print(f"Climbing for {duration}s toward approx {target_alt_m} meters (no GPS)...")
    # Send RC override for throttle channel (typically channel 3)
    # You may need to adjust the PWM value depending on your ESCs
    throttle_pwm = 1200  # Slightly above mid-throttle
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, throttle_pwm, 0, 0, 0, 0, 0
    )
    time.sleep(duration)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 1200, 0, 0, 0, 0, 0
    )
    print("Throttle neutral (hover or descend)")

def disarm(master):
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed")

def main():
    master = connect()
    disable_fence(master)
    disable_gps_check(master)
    set_mode_alt_hold(master)
    arm(master)
    climb_to_altitude(master, target_alt_m=1.0, duration=5)
    disarm(master)

if __name__ == "__main__":
    main()
