from pymavlink import mavutil

# Connect to Pixhawk over USB (USB ONLY ONLY ONLY !!!!!)
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("✅ Heartbeat received!")

# Request all data streams at 2 Hz
connection.mav.request_data_stream_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    2,
    1
)

# Loop to read and print selected telemetry
while True:
    msg = connection.recv_match(blocking=True)
    
    if not msg:
        continue

    # Print attitude: pitch, roll, yaw
    if msg.get_type() == "ATTITUDE":
        print(f"[ATTITUDE] Pitch: {msg.pitch:.2f}, Roll: {msg.roll:.2f}, Yaw: {msg.yaw:.2f}")
    
    # Print altitude and climb rate from VFR_HUD
    elif msg.get_type() == "VFR_HUD":
        print(f"[VFR_HUD] Altitude: {msg.alt:.2f} m, Climb rate: {msg.climb:.2f} m/s, Groundspeed: {msg.groundspeed:.2f} m/s")

    # Print battery info from SYS_STATUS
    elif msg.get_type() == "SYS_STATUS":
        voltage = msg.voltage_battery / 1000.0  # Convert mV to V
        print(f"[BATTERY] Voltage: {voltage:.2f} V, Current: {msg.current_battery / 100.0:.2f} A")
    elif msg.get_type() == "GLOBAL_POSITION_INT":
        lat = msg.lat / 1e7   # Convert from degrees * 1E7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # Altitude in meters
        print(f"[GPS] Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f} m")

