#!/usr/bin/env python

import sys
import os
from pymavlink import mavutil
import math

from .cv_target_finder import (
    initialize_camera, 
    find_target_center, 
    get_frame_center, 
    release_camera
)

# Add the project root to Python path so we can import modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../'))
sys.path.insert(0, project_root)

from scripts.utils.mav_utils import get_latest_gps

# FLOW
# - assume the pilot lines the drone with the target visible

# - INPUT:  (1) target location (long, lat, alt)
#           (2) direction target is facing (N/E/S/W/NE/NW/SE/SW)

# - fly drone to coordinates
#   - subtract spraying distance from lat/long depending on direction

# - (LATER) use lidar to normalize drone position to wall

# - run cv again to center drone at target
#   *** expects target in camera frame

# Define global constants
SPRAYING_DIST = 2

def main(lat, long, alt, dir):

    # Create the MAVLink connection
    # connection = mavutil.mavlink_connection('/dev/serial0', baud=57600) # HITL
    connection = mavutil.mavlink_connection('udp:host.docker.internal:14550') # SITL

    connection.wait_heartbeat()
    print("Pixhawk connected and heartbeat received")

    # Set flight mode to GUIDED
    connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,  # confirmation
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base mode
    4,  # custom mode: GUIDED
    0, 0, 0, 0, 0  # other params
    )
    print('GUIDED mode enabled')

    # Send arm command
    connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # param1: 1 to arm
    0, 0, 0, 0, 0, 0  # other params
    )
    print('Armed')

    # compute new long/ lat values based on target direction
    lat, long = compute_spraying_distance(lat, long, dir)

    # Send takeoff command
    connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, float('nan'),
    # latitude / longitude / altitude
    lat, long,
    alt  # Target altitude in meters
    )
    print("Taking off...")

    # (LATER) use lidar to normalize to the wall

    # CV centering 
    if not initialize_camera(camera_index=0): # Use 0 or appropriate index for the drone's camera
        print("Failed to initialize camera. Skipping CV positioning.")
        return

    # Constants for the CV loop
    Kp = 0.005              # Proportional gain (TUNE THIS: adjust based on how fast the drone moves per pixel of error)
    MAX_VEL = 0.5           # Max velocity in m/s
    PIXEL_TOLERANCE = 10    # Stop if error is less than this many pixels
    
    center_x, center_y = get_frame_center()
    print(f"Frame center: ({center_x}, {center_y})")

    print("Starting CV positioning loop...")
    target_centered = False
    
    # Loop for continuous fine-tuning
    while not target_centered:
        target_coords = find_target_center()
        
        if target_coords is None:
            # If target is lost, stop the drone and search slowly (or hover)
            send_target_velocity_command(connection, 0, 0, 0)
            print("Target lost! Hovering...")
            time.sleep(1) 
            continue

        t_x, t_y = target_coords
        
        # Calculate error in pixels (e.g., positive Ex means target is to the right of center)
        Ex = t_x - center_x # Horizontal error (affects East velocity, vy)
        Ey = center_y - t_y # Vertical error (positive Ey means target is too low, affects North velocity, vx)
        
        # Check for convergence
        if abs(Ex) < PIXEL_TOLERANCE and abs(Ey) < PIXEL_TOLERANCE:
            target_centered = True
            print("Target centered. Stopping movement.")
            send_target_velocity_command(connection, 0, 0, 0) # Final stop command
            break
            
        # Calculate velocity commands
        vx = Kp * Ey  # North velocity (moves drone forward/backward)
        vy = Kp * Ex  # East velocity (moves drone left/right)
        
        # Clamp velocities to prevent overshooting or unsafe speeds
        vx = max(min(vx, MAX_VEL), -MAX_VEL)
        vy = max(min(vy, MAX_VEL), -MAX_VEL)
        
        print(f"Error (Ex, Ey): ({Ex:.2f}, {Ey:.2f}) | Command (vx, vy): ({vx:.2f}, {vy:.2f})")

        # Send command to the drone
        send_target_velocity_command(connection, vx, vy, vz=0)

        time.sleep(0.1) # Loop rate: Adjust based on your system performance

    # administer spray

    # take picture and upload to drive

    release_camera()



def compute_spraying_distance(lat, long, dir):
    # takes lat/long/direction
    # outputs new lat, long pair

    # 45 degree angle so need to adjust SPRAYING_DIST
    ADJUSTMENT = SPRAYING_DIST / math.sqrt(2)

    if (dir == "N"):
        lat += SPRAYING_DIST
    elif (dir == "E"):
        long += SPRAYING_DIST
    elif (dir == "S"):
        lat -= dist
    elif (dir == "W"):
        long -= SPRAYING_DIST
    
    elif (dir == "NE"):
        # North (positive lat) and East (positive long)
        lat += ADJUSTMENT
        long += ADJUSTMENT
    elif (dir == "SE"):
        # South (negative lat) and East (positive long)
        lat -= ADJUSTMENT
        long += ADJUSTMENT
    elif (dir == "SW"):
        # South (negative lat) and West (negative long)
        lat -= ADJUSTMENT
        long -= ADJUSTMENT
    elif (dir == "NW"):
        # North (positive lat) and West (negative long)
        lat += ADJUSTMENT
        long -= ADJUSTMENT
        
    return lat, long


def send_target_velocity_command(connection, vx, vy, vz=0, duration=0.1):
    """
    Sends a velocity command for a short duration.
    vx, vy, vz are in m/s in the LOCAL_NED frame.
    """
    
    # MAV_FRAME_BODY_NED (8) is often used for velocity relative to the drone's body, 
    # but for centering, MAV_FRAME_LOCAL_NED (1) is usually safer for initial testing, 
    # as it doesn't require yaw alignment. Let's use LOCAL_NED for simplicity.
    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    
    # Mask to ignore position and yaw/yaw rate (only use velocity)
    # Bits 10, 9, 8, 7, 6, 5, 4 (x, y, z position & yaw/yaw rate) are ignored
    type_mask = 0b0000111111000111
    
    connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        connection.target_system,
        connection.target_component,
        frame,
        type_mask,
        0, 0, 0,  # x, y, z position (ignored)
        vx, vy, vz,  # velocities (m/s)
        0, 0, 0,  # accelerations (ignored)
        0, 0)     # yaw, yaw rate (ignored)

    # Note: ArduPilot will continue this velocity until a new command is sent.
    # The control loop below must run continuously.


if __name__ == "__main__":
    main()
