from pymavlink import mavutil
import time

def attempt_arm(master):
    print("Disabling fencing...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'FENCE_ENABLE',
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1) 

    print("Sending ARM command...")

    # Send ARM command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    # Wait for COMMAND_ACK and print it
    ack_timeout = time.time() + 5
    while time.time() < ack_timeout:
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f" COMMAND_ACK received: Result = {msg.result}")
            if msg.result != 0:
                print("Arming rejected. Reason:")
            break
        time.sleep(0.2)

    # Watch for arming status or errors for 10 seconds
    print(" Watching status messages...")
    start_time = time.time()
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        if msg.get_type() == "STATUSTEXT":
            print(f"STATUS: {msg.text}")

        elif msg.get_type() == "HEARTBEAT":
            armed = master.motors_armed()
            mode = master.flightmode
            print(f" Mode: {mode} | Armed: {armed}")
            if armed:
                print("[✅] Pixhawk is now armed!")	
                time.sleep(1)
                print("Pixhawk is disarming")
                master.mav.command_long_send(
				master.target_system,
				master.target_component,
				mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
				0,    
				0,    # param1 = 0 to disarm (1 = arm)
				0, 0, 0, 0, 0, 0)

                return True
		
        time.sleep(0.2)

    print("Failed to arm after retries.")
    return False


if __name__ == "__main__":
    connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    connection.wait_heartbeat()
    print("Heartbeat received")

    print("Setting mode to STABILIZE...")
    connection.set_mode_apm("STABILIZE")
    time.sleep(1)

    success = attempt_arm(connection)
    if not success:
        print("Error arming")
    else:
        print("Success: Pixhawk is armed")
