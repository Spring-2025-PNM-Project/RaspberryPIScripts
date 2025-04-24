from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk (system %d component %d)" %
          (master.target_system, master.target_component))
    return master

def set_mode_stabilize(master):
    print("Setting mode to STABILIZE...")
    master.set_mode_apm("STABILIZE")
    time.sleep(2)
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and master.flightmode == "STABILIZE":
            print("Mode confirmed: STABILIZE")
            return
        time.sleep(0.3)
    print("Warning: STABILIZE mode not confirmed")

def arm(master):
    print("Sending ARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)
    master.motors_armed_wait()
    print("Motors armed")

def disarm(master):
    print("Sending DISARM command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    master.motors_disarmed_wait()
    print("Motors disarmed")

def override_throttle(master, pwm):
    print(f"Setting throttle override: {pwm} PWM")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

def monitor_feedback(master, duration=4):
    print("Listening for ACK/STATUSTEXT...")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        if msg.get_type() == "COMMAND_ACK":
            print(f"[ACK] Command: {msg.command}, Result: {msg.result}")
        elif msg.get_type() == "STATUSTEXT":
            print(f"[STATUSTEXT] {msg.text}")
        elif msg.get_type() == "HEARTBEAT":
            print(f"[HEARTBEAT] Mode: {master.flightmode}, Armed: {master.motors_armed()}")
        time.sleep(0.2)

def try_takeoff_command(master, altitude=1.0):
    print("Attempting MAV_CMD_NAV_TAKEOFF...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    monitor_feedback(master, duration=5)

def main():
    master = connect()
    set_mode_stabilize(master)
    time.sleep(1)

    arm(master)
    time.sleep(2)

    print("Beginning throttle ramp...")
    for pwm in range(900, 1300, 100):
        override_throttle(master, pwm)
        monitor_feedback(master, duration=4)
        time.sleep(2)

    try_takeoff_command(master, altitude=1.0)

    try:
        print("Holding final throttle...")
        override_throttle(master, 1200)
        for i in range(10):
            time.sleep(1)
            print(f"[DEBUG] Holding throttle... {i+1}s")

            msg = master.recv_match(blocking=False)
            if msg:
                if msg.get_type() == "STATUSTEXT":
                    print(f"[STATUSTEXT] {msg.text}")
                elif msg.get_type() == "HEARTBEAT":
                    print(f"[HEARTBEAT] Mode: {master.flightmode}, Armed: {master.motors_armed()}")

    except Exception as e:
        print("Caught exception during final hold:")
        print(str(e))

    finally:
        disarm(master)

if __name__ == "__main__":
    main()
