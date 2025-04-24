from pymavlink import mavutil

def emergency_disarm():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Emergency disarm issued")

    master.set_mode_apm("GUIDED")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

if __name__ == "__main__":
    emergency_disarm()
