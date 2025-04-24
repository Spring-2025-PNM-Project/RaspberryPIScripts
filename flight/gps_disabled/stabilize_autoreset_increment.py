from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("[CONNECTED] Heartbeat received")
    return master

def force_mode_reset(master, desired_mode="ALT_HOLD"):
    print("[MODE] Checking current mode...")
    master.recv_match(type="HEARTBEAT", blocking=True)
    current_mode = master.flightmode
    print(f"[MODE] Current mode: {current_mode}")
    if current_mode == "LAND":
        print("[MODE] Detected LAND mode. Resetting through STABILIZE...")
        master.set_mode_apm("STABILIZE")
        time.sleep(3)
    print(f"[MODE] Setting mode to {desired_mode}...")
    master.set_mode_apm(desired_mode)
    time.sleep(3)
    print(f"[MODE] Final mode: {master.flightmode}")

def arm(master):
    print("[ARM] Sending arm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("[ARM] Motors armed")

def override_throttle(master, pwm, hold_time=2):
    print(f"[THROTTLE] Holding override: {pwm} PWM for {hold_time} seconds")
    start = time.time()
    while time.time() - start < hold_time:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, pwm, 0, 0, 0, 0, 0
        )
        time.sleep(0.2)

def throttle_ramp_and_hold(master):
    print("[THROTTLE] Starting throttle ramp up...")
    for pwm in range(1200, 1501, 250):
        override_throttle(master, pwm, hold_time=3)

    print("[THROTTLE] Holding 2000 PWM for 1 seconds")
    #override_throttle(master, 2000, .5)
    for pwm in range(1500, 2000, 250):
        override_throttle(master, pwm, hold_time=1)
    #print("[THROTTLE] Holding 1500 PWM for 5 seconds")
    #override_throttle(master, 1500, 3)

    print("[THROTTLE] Starting throttle ramp down...")
    #for pwm in range(1700, 1499, -100):
    override_throttle(master, 1700, hold_time=4)
    #override_throttle(master, 1700, hold_time=1)
    print("[THROTTLE] Starting ramp down...")
    #override_throttle(master, 1400, hold_time=2)


def land(master):
    print("[LAND] Sending LAND command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)
    print("[LAND] LAND command sent")

def disarm(master):
    print("[DISARM] Sending disarm command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("[DISARM] Motors disarmed")

def reset_post_landing_mode(master):
    print("[RESET] Forcing mode reset to STABILIZE after LAND...")
    force_mode_reset(master, "STABILIZE")

def main():
    master = connect()
    force_mode_reset(master, "ALT_HOLD")
    arm(master)
    time.sleep(2)
    throttle_ramp_and_hold(master)
    land(master)
    disarm(master)
    reset_post_landing_mode(master)

if __name__ == "__main__":
    main()
