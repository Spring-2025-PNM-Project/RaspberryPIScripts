from pymavlink import mavutil
import time

# Connect to Pixhawk via USB
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("✅ Heartbeat received")

# 1. Disable arming checks (ARMING_CHECK = 0)
print("Disabling arming checks...")
connection.mav.param_set_send(
    connection.target_system,
    connection.target_component,
    b'ARMING_CHECK',
    float(0),
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)

# Give Pixhawk time to apply parameter
time.sleep(1)

# 2. Set flight mode to STABILIZE (non-GPS)
connection.set_mode_apm("STABILIZE")
print("Set mode to STABILIZE")
time.sleep(2)

# 3. Arm motors (forcefully)
print("⚙️ Arming motors...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Wait for motors to arm or timeout after 10s
for i in range(20):
    if connection.motors_armed():
        print("✅ Motors are armed!")
        break
    print("Waiting for motors to arm...")
    time.sleep(0.5)

# 4. Send throttle override
throttle_value = 2000  # Try 1200–1400 to spin motors
print(f"🕹️ Sending throttle: {throttle_value}")
connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    65535, 65535, throttle_value, 65535, 65535, 65535, 65535, 65535
)

time.sleep(10)

# 5. Reset throttle override
print("🛑 Resetting throttle override...")
connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
)

# 6. Disarm motors
print("🔒 Disarming motors...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

print("✅ Motors disarmed. Test complete.")
