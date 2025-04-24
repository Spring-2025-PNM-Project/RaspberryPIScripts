from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def disable_fence_and_gps(master):
    print("Disabling FENCE_ENABLE and GPS_REQUIRED...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_ENABLE',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'GPS_REQUIRED',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    print("Sent param set commands")
    time.sleep(2)

def set_mode_alt_hold(master):
    print("Setting flight mode to ALT_HOLD...")
    master.set_mode_apm("ALT_HOLD")
    print(f"Flight mode now: {master.flightmode}")

def arm(master):
    print("Attempting to arm...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"ARM ACK: Command={msg.command}, Result={msg.result}")
            print_ack_reason(msg.result)
            return msg.result
        time.sleep(0.1)
    print("No COMMAND_ACK received for arm.")
    return None

def print_ack_reason(result):
    reasons = {
        0: "ACCEPTED",
        1: "TEMPORARILY_REJECTED",
        2: "DENIED - Check safety switch, fence, GPS, EKF, or battery",
        3: "UNSUPPORTED",
        4: "FAILED",
        5: "IN_PROGRESS",
        6: "CANCELLED"
    }
    print(f"Result meaning: {reasons.get(result, 'Unknown code')}")

def emergency_land(master):
    print("Sending emergency land command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    timeout = time.time() + 3
    while time.time() < timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            print(f"LAND ACK ? Result: {msg.result}")
            break
        time.sleep(0.1)

def main():
    master = connect()
    disable_fence_and_gps(master)
    set_mode_alt_hold(master)
    result = arm(master)
    if result == 0:
        print("? ALT_HOLD mode arm successful")
        time.sleep(10)
        print("Disarming")
        emergency_land(master)
    else:
        print("? ALT_HOLD mode arm failed")
    print("Leaving if statement")
    emergency_land(master)

if __name__ == "__main__":
    main()
