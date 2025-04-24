from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def set_mode_safe(master):
    master.set_mode_apm("STABILIZE")
    print("Mode set to STABILIZE")

def arm_and_check_ack(master):
    print("Sending ARM command...")
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
            print(f"ARM ACK RECEIVED ? Command: {msg.command}, Result: {msg.result}")
            print_ack_reason(msg.result)
            return msg.result
        time.sleep(0.1)

    print("No COMMAND_ACK received.")
    return None

def print_ack_reason(result):
    reasons = {
        0: "ACCEPTED",
        1: "TEMPORARILY_REJECTED",
        2: "DENIED (permanent) ? check safety switch, parameters, or sensors",
        3: "UNSUPPORTED ? command not supported",
        4: "FAILED",
        5: "IN_PROGRESS",
        6: "CANCELLED"
    }
    print(f"Interpretation: {reasons.get(result, 'Unknown reason code')}")

def listen_statustext(master, duration=10):
    print("Listening for STATUSTEXT messages...")
    start = time.time()
    while time.time() - start < duration:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"STATUSTEXT: {msg.text}")
        time.sleep(0.1)

def main():
    master = connect()
    set_mode_safe(master)
    arm_and_check_ack(master)
    listen_statustext(master)

if __name__ == "__main__":
    main()