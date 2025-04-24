from pymavlink import mavutil
import time

def connect_pixhawk(port='/dev/ttyAMA0', baud=57600):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print("Connected to Pixhawk.")
    return master

def set_mode(master, mode='STABILIZE'):
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}")
        return
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print(f"Flight mode set to {mode}.")

def get_arming_status(master):
    return master.motors_armed()

def get_flight_mode(master):
    return master.flightmode

def print_heartbeat_info(master):
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print(f"Heartbeat received: system_status={msg.system_status}, base_mode={msg.base_mode}, custom_mode={msg.custom_mode}")
    else:
        print("No heartbeat received.")

def print_sys_status(master):
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000.0
        print(f"System status: voltage={voltage:.2f}V, load={msg.load}, errors_count1={msg.errors_count1}")
    else:
        print("No system status received.")

def print_statustext_messages(master, duration=5):
    start_time = time.time()
    while time.time() - start_time < duration:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"STATUSTEXT: {msg.text}")
        time.sleep(0.1)

def arm_and_disarm(master):
    print("Arming the vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed.")

    time.sleep(30)

    print("Disarming the vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle disarmed.")

if __name__ == "__main__":
    master = connect_pixhawk()
    set_mode(master, 'STABILIZE')
    print_heartbeat_info(master)
    print_sys_status(master)
    print_statustext_messages(master, duration=5)
    print(f"Current flight mode: {get_flight_mode(master)}")
    print(f"Arming status: {'Armed' if get_arming_status(master) else 'Disarmed'}")
    arm_and_disarm(master)
