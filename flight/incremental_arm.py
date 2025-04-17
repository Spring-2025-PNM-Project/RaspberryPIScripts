from pymavlink import mavutil
import time

# Connect to Pixhawk
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
print("🔌 Waiting for heartbeat...")
connection.wait_heartbeat()
print("✅ Heartbeat received")

# Disable arming checks (for bench test)
connection.mav.param_set_send(
    connection.target_system,
    connection.target_component,
    b'ARMING_CHECK',
    float(0),
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)
time.sleep(1)

# Set mode to STABILIZE
connection.set_mode_apm("STABILIZE")
print("🚦 Set mode to STABILIZE")
time.sleep(2)

# Arm motors
print("⚙️ Arming motors...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Wait for arm confirmation
for i in range(20):
    if connection.motors_armed():
        print("✅ Motors are armed!")
        break
    print("Waiting for arming...")
    time.sleep(0.5)

# 🕹️ Ramp throttle from 1100 to 1800
print("🚀 Ramping throttle up...")
for throttle in range(1100, 1801, 50):
    print(f"Throttle: {throttle}")
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        1500, 1500, throttle, 1500,  # Roll, Pitch, Throttle, Yaw
        65535, 65535, 65535, 65535
    )
    time.sleep(0.5)

# 🕹️ Ramp throttle down from 1800 to 1100
print("🛬 Ramping throttle down...")
for throttle in range(1800, 1099, -50):
    print(f"Throttle: {throttle}")
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        1500, 1500, throttle, 1500,
        65535, 65535, 65535, 65535
    )
    time.sleep(0.5)

# Reset RC override
print("🛑 Resetting RC override...")
connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
)

# Disarm motors
print("🔒 Disarming motors...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("✅ Test complete. Motors disarmed.")
