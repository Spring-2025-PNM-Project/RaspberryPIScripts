from pymavlink import mavutil

def emergency_land():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Emergency: landing")

    master.set_mode_apm("GUIDED")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

if __name__ == "__main__":
    emergency_land()
