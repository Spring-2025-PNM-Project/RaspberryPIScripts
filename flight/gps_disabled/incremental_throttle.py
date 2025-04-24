from pymavlink import mavutil
import time

def connect():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Pixhawk")
    return master

def disable_fence_and_gps(master):
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
    print("Disabled FENCE and GPS requirements")

def set_mode_stabilize(master):
    master.set_mode_apm("STABILIZE")
    print("Flight mode set to STABILIZE")

def arm(master):
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed")

def test_motor_spin(master):
    print("Testing throttle PWM range...")
    for pwm in range(1000, 1600, 25):
        print(f"Sending throttle PWM: {pwm}")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, pwm, 0, 0, 0, 0, 0
        )
        time.sleep(2)

    print("Resetting throttle to neutral")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 1500, 0, 0, 0, 0, 0
    )

def disarm(master):
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed")

def main():
    master = connect()
    disable_fence_and_gps(master)
    set_mode_stabilize(master)
    arm(master)
    test_motor_spin(master)
    disarm(master)

if __name__ == "__main__":
    main()
