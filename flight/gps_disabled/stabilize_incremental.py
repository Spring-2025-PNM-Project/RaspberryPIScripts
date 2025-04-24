from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print(f"Connected to Pixhawk (system {master.target_system}, component {master.target_component})")
    return master

def set_stabilize_mode(master):
    print("Setting mode to STABILIZE...")
    master.set_mode_apm("STABILIZE")
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and master.flightmode == 'STABILIZE':
            print("STABILIZE mode confirmed")
            return
        time.sleep(0.2)
    print("Warning: STABILIZE mode not confirmed")

def arm(master):
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed")

def disarm(master):
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed")

def override_throttle(master, pwm):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, pwm, 1500, 1500, 1500, 1500, 1500
    )

def wait_for_statustext(master, timeout=3):
    print("Waiting for STATUSTEXT...")
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print(f"[STATUSTEXT] {msg.text}")
            if "DISARMED" in msg.text or "FAILSAFE" in msg.text:
                print("CRITICAL:", msg.text)

def monitor_debug(master):
    hb = master.recv_match(type="HEARTBEAT", blocking=False)
    if hb:
        mode = master.flightmode
        armed = master.motors_armed()
        print(f"[HEARTBEAT] Mode: {mode}, Armed: {armed}")

    for _ in range(10):
        st = master.recv_match(type="STATUSTEXT", blocking=False)
        if not st:
            break
        print(f"[STATUSTEXT] {st.text}")
        if "DISARMED" in st.text or "FAILSAFE" in st.text:
            print("CRITICAL:", st.text)

def throttle_ramp_debug(master, spin_pwm=900, max_pwm=1500, step=50, delay=3):
    print("Beginning throttle ramp in STABILIZE...")
    for pwm in range(spin_pwm, max_pwm + 1, step):
        print(f"\nSetting throttle to {pwm}")
        override_throttle(master, pwm)

        for _ in range(int(delay * 5)):
            monitor_debug(master)
            time.sleep(0.2)

            if not master.motors_armed():
                print("CRITICAL: Motors disarmed")
                wait_for_statustext(master)
                print("Re-arming motors...")
                arm(master)
                print("Re-armed. Re-sending throttle override.")
                override_throttle(master, pwm)
                break

    print("Throttle ramp complete. Holding final value for 3 seconds...")
    override_throttle(master, max_pwm)
    time.sleep(3)

def main():
    master = connect()
    set_stabilize_mode(master)
    arm(master)
    time.sleep(2)
    throttle_ramp_debug(master)
    disarm(master)

if __name__ == "__main__":
    main()
