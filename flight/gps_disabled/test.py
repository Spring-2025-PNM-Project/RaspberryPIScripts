from pymavlink import mavutil
import time

# -----------------------------------------
def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECTED] Heartbeat received")
    return master

# -----------------------------------------
def set_mode(mode="ALT_HOLD"):
    master = connect()
    print(f"[MODE] Setting mode to {mode}...")
    master.set_mode_apm(mode)
    time.sleep(2)
    print(f"[MODE] Current mode: {master.flightmode}")
    master.close()

# -----------------------------------------
def arm_motors():
    master = connect()
    print("[ARM] Sending arm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)
    master.motors_armed_wait()
    print("[ARM] Motors armed")
    master.close()

# -----------------------------------------
def disarm_motors():
    master = connect()
    print("[DISARM] Sending disarm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    master.motors_disarmed_wait()
    print("[DISARM] Motors disarmed")
    master.close()

# -----------------------------------------
def send_throttle(pwm=1200, duration=3):
    master = connect()
    print(f"[THROTTLE] Sending override: {pwm} PWM")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )
    for i in range(duration):
        print(f"[THROTTLE] Holding {pwm} PWM for {i+1}/{duration} seconds")
        time.sleep(1)
    master.close()

# -----------------------------------------
def send_takeoff(altitude=1.5):
    master = connect()
    print(f"[TAKEOFF] Sending MAV_CMD_NAV_TAKEOFF to {altitude}m")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(4)
    master.close()

# -----------------------------------------
if __name__ == "__main__":
    # You can call these one at a time for testing:
    set_mode("ALT_HOLD")
    arm_motors()
    send_throttle(2000, duration=2)
    send_throttle(1200, duration=3)
    send_throttle(900, duration=2)
    disarm_motors()
