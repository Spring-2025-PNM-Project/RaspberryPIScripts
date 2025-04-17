from pymavlink import mavutil
import time
from config import PIXHAWK_CONNECTION_STRING

def connect_vehicle():
    master = mavutil.mavlink_connection(PIXHAWK_CONNECTION_STRING)
    master.wait_heartbeat()
    print("[?] Connected to vehicle")
    return master

def arm_and_takeoff(master, altitude):
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[?] Taking off...")
    master.set_mode("GUIDED")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(8)

def move_forward(master, meters):
    print("[??] Moving forward...")
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        int(0b0000111111000111),
        0, 0, 0,
        meters, 0, 0,
        0, 0, 0,
        0, 0
    )
    time.sleep(5)

def land(master):
    print("[?] Landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(10)

def execute_flight_plan(plan):
    master = connect_vehicle()
    arm_and_takeoff(master, plan.get("takeoff", 1.5))
    move_forward(master, plan.get("forward", 3.0))
    land(master)
