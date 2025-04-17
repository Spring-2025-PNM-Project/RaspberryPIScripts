from pymavlink import mavutil
import time

# Connect to Pixhawk
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
connection.wait_heartbeat()
print("✅ Heartbeat received")

# Set output channel and PWM (example: channel 3, 1600 PWM)
# NOTE: SERVO numbering = output pin number
servo_channel = 3  # Output 3 = MAIN OUT 3
pwm_value = 1600

# Send PWM to motor output
print(f"⚡ Sending {pwm_value} PWM to servo output {servo_channel}")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    servo_channel,   # Servo output (1 = MAIN OUT 1, 2 = MAIN OUT 2, etc.)
    pwm_value,       # PWM value (1000–2000 typical range)
    0, 0, 0, 0, 0
)
