from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print(f"Connected to Pixhawk (system {master.target_system}, component {master.target_component})")
    return master

def set_alt_hold_mode(master):
    print("Setting mode to ALT_HOLD...")
    master.set_mode_apm("ALT_HOLD")
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and master.flightmode == 'ALT_HOLD':
            print("ALT_HOLD mode confirmed")
            return
        time.sleep(0.2)
    print("Warning: ALT_HOLD not confirmed")

def arm(master):
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    master.motors_armed_wait()
    print("Motors armed")

def disarm(master):
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    master.motors_disarmed_wait()
    print("Motors disarmed")

def override_throttle(master, pwm):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, pwm, 1500, 1500, 1500, 1500, 1500
    )

def land(master):
    print("Sending LAND command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    wait_for_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_LAND)

def wait_for_statustext(master, timeout=3):
    print("Waiting for STATUSTEXT...")
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print(f"[STATUSTEXT] {msg.text}")
            if "DISARMED" in msg.text or "FAILSAFE" in msg.text:
                print("CRITICAL:", msg.text)

def wait_for_command_ack(master, command_id, timeout=5):
    end = time.time() + timeout
    while time.time() < end:
        ack = master.recv_match(type='COMMAND_ACK', blocking=False)
        if ack and ack.command == command_id:
            print(f"[COMMAND_ACK] Command: {ack.command}, Result: {ack.result}")
            return
        time.sleep(0.1)

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

    ekf = master.recv_match(type="EKF_STATUS_REPORT", blocking=False)
    if ekf:
        print(f"[EKF] Flags: {ekf.flags}, VelocityVar: {ekf.velocity_variance:.2f}, PosHVar: {ekf.pos_horiz_variance:.2f}")

    pos = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if pos:
        rel_alt = pos.relative_alt / 1000.0
        print(f"[ALTITUDE] Relative Altitude: {rel_alt:.2f} m")

def throttle_ramp_debug(master, spin_pwm=900, max_pwm=1500, step=50, delay=3):
    print("Beginning throttle ramp...")
    for pwm in range(spin_pwm, max_pwm + 1, step):
        print(f"\nSetting throttle to {pwm}")
        override_throttle(master, pwm)

        for _ in range(int(delay * 5)):
            monitor_debug(master)
            time.sleep(0.2)

            if not master.motors_armed():
                print("CRITICAL: Motors disarmed")
                wait_for_statustext(master)
                print("CRITICAL: Motors disarmed. Attempting auto re-arm.")
                arm(master)
                print("Re-armed. Re-sending throttle override.")
                override_throttle(master, pwm)
                break

    print("\nThrottle ramp complete. Holding final value for 3 seconds...")
    override_throttle(master, max_pwm)
    time.sleep(3)

def main():
    master = connect()
    set_alt_hold_mode(master)
    arm(master)
    time.sleep(2)
    throttle_ramp_debug(master)
    land(master)
    time.sleep(5)
    disarm(master)

if __name__ == "__main__":
    main()
