
import math
import os
import logging
import datetime
import socket
import time
import novatel_oem7_msgs
import numpy as np
import rospy
from novatel_oem7_msgs.msg import BESTPOS, BESTVEL, INSPVA
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

# Define the MABX(vehicle control controller) IP address and port for sending data
mabx_IP = "192.168.50.1"
mabx_PORT = 30000

# Define buffer size and local interface
BUFFER_SIZE = 4096
local_interface = "eth0"

# Conversion factor for latitude and longitude to meters
LAT_LNG_TO_METER = 1.111395e5
CW_flag = 0
pot_time = 0


# Global variables for traffic-light
prev = ''
curr = None
cntr = 0


# Initialize the ROS node for the algorithm
rospy.init_node("GNSS_navigation", anonymous=True)

# Initialize the UDP socket for MABX communication
mabx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mabx_addr = (mabx_IP, mabx_PORT)


def setup_logging(file_path):
    log_dir = os.path.expanduser(
        "~/Desktop/Solio-ADAS/Solio-Suzuki/navigation/logs")
    base_filename = os.path.basename(file_path)
    # Remove the file extension to get the desired string
    logFileName = os.path.splitext(base_filename)[0]

    # Create the log directory if it does not exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # Set up the logging format
    log_format = "%(asctime)s [%(levelname)s]: %(message)s"
    logging.basicConfig(level=logging.DEBUG, format=log_format)

    # Create a log file with the current timestamp as the name
    current_time = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    log_filename = f"{logFileName}-{current_time}.log"
    log_path = os.path.join(log_dir, log_filename)

    # Add a file handler to save logs to the file
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(logging.DEBUG)

    # Set the log format for the file handler
    file_handler.setFormatter(logging.Formatter(log_format))

    # Add the file handler to the logger
    logger = logging.getLogger("")
    logger.addHandler(file_handler)

    return logger


def log_and_print(str):
    logger.info(str)
    print(str)

# Function to parse and retrieve coordinates from a file
def get_coordinates(file_path):
    coordinates_list = []
    try:
        with open(file_path, "r") as file:
            for line in file:
                try:
                    coordinates = [
                        float(coord) for coord in line.strip().strip("[],").split(",")
                    ]
                    coordinates_list.append(coordinates)
                except ValueError:
                    # Handle the exception if a value cannot be converted to float
                    print(
                        f"Error: Unable to convert coordinates in line '{line}' to float."
                    )
    except FileNotFoundError:
        # Handle the exception if the file is not found
        print(f"Error: The file '{file_path}' could not be found.")
    except Exception as e:
        # Handle any other unexpected exceptions
        print(f"An error occurred: {e}")

    return coordinates_list


def callback_collision_warning(data):
    global CW_flag
    CW_flag = 0
    CW_flag = data.data

def callback_pothole(data):
    global pothole_flag
    pothole_flag = 0
    pothole_flag = data.data

def callback_traffic(data):
    global go_signal            # 0: Red    ;   1: Green 
    go_signal = data.data

def callback_traffic_depth(data):
    global depth_signal            
    depth_signal = data.data


# rospy.Subscriber("/sub1_label_topic",String, callback_traffic)
# rospy.Subscriber("/sub1_depth_topic",Float32, callback_traffic_depth)
# rospy.Subscriber("/collision", Float32, callback_collision_warning)
# rospy.Subscriber("/pothole", Float32, callback_pothole)


# Callback functions for handling GNSS data
def callback_velocity(data):
    global current_vel
    current_vel = 3.6 * data.hor_speed

def callback_heading(data):
    global heading
    heading = (
        data.azimuth
    )  # Left-handed rotation around z-axis in degrees clockwise from North.

def callback_latlng(data):
    global lat, lng, lat_delta, lng_delta
    lat = data.lat
    lng = data.lon
    lat_delta = data.lat_stdev
    lng_delta = data.lon_stdev

def callback_gnss_imu(data):
    global acc_z
    global hit_time
    acc_z = hit_time = 0
    acc_z = data.linear_acceleration.z
    hit_time = data.header.stamp.secs


rospy.Subscriber("/novatel/oem7/bestvel", BESTVEL, callback_velocity)
rospy.Subscriber("/novatel/oem7/inspva", INSPVA, callback_heading)
rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback_latlng)
rospy.Subscriber("/imu/data_raw", Imu, callback_gnss_imu)

time.sleep(0.1)

# Function to calculate and set steering angle based on current angle and target angle
def set_angle(current_angle, angle_change):
    # Steering gear ratio
    gear_ratio = 17.75
    # Limit steering angle within a range
    if 40 * gear_ratio < current_angle:
        current_angle = 40 * gear_ratio
    elif -40 * gear_ratio > current_angle:
        current_angle = -40 * gear_ratio
    # Calculate scaled angle for transmission
    scaled_angle = (current_angle / gear_ratio - (-65.536)) / 0.002
    high_angle_byte, low_angle_byte = (int)(
        scaled_angle) >> 8, (int)(scaled_angle) & 0xFF
    return high_angle_byte, low_angle_byte

def calc_checksum(message_bytes):
    checksum = 0
    for m in message_bytes:
        checksum += m
    checksum = (0x00 - checksum) & 0x000000FF
    checksum = checksum & 0xFF
    return checksum

def set_speed(speed):
    speed = speed * 128
    high_byte_speed = (int)(speed) >> 8
    low_byte_speed = (int)(speed) & 0xFF
    return high_byte_speed, low_byte_speed

def get_speed(high_byte_speed, low_byte_speed):
    """
    Calculates the speed from the high and low bytes of the scaled speed.

    Args:
      high_byte_speed: The high byte of the scaled speed.
      low_byte_speed: The low byte of the scaled speed.

    Returns:
      The speed in km/h.
    """

    # Combine the high and low bytes into a single integer.
    speed = (high_byte_speed << 8) | low_byte_speed

    # Divide the speed by 128 to convert it back to km/h.
    speed = speed / 128

    return speed

def send_message_to_mabx(speed, current_angle, delta_angle, flasher_light, message_counter):
    H_Angle, L_Angle = set_angle(current_angle, -1 * delta_angle)
    H_Speed, L_Speed = set_speed(speed)

    message_bytes = [
        1,
        message_counter,
        0,
        1,
        52,
        136,
        215,
        1,
        H_Speed,
        L_Speed,
        H_Angle,
        L_Angle,
        0,
        flasher_light,
        0,
        0,
        0,
        0,
    ]
    message_bytes[2] = calc_checksum(message_bytes)
    message = bytearray(message_bytes)
    # print("         Reading the Speed from MABX: ", get_speed(message[8], message[9]))
    # print("Reading the sent Angle from MABX: ", message[10], message[11])
    log_and_print(
        "================================================================")
    return message

def calculate_steer_output(currentLocation, Current_Bearing):
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    STEER_GAIN = 750  # For Tight Turns 1200 can be used or 900 in general

    off_y = -currentLocation[0] + waypoints[wp][0]
    off_x = -currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION

    # convert negative bearings to positive by adding 360 degrees
    if target_bearing < 0:
        target_bearing += 360.00

    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading

    Current_Bearing = float(Current_Bearing)
    # log_and_print(f"Current Bearing : {Current_Bearing:.1f} , Target Bearing : {target_bearing:.1f}")

    bearing_diff = Current_Bearing - target_bearing

    # normalize bearing difference to range between -180 and 180 degrees
    if bearing_diff < -180:
        bearing_diff += 360
    elif bearing_diff > 180:
        bearing_diff -= 360

    if abs(bearing_diff) < 1:  # Nullify the small the bearing difference
        temp = bearing_diff
        STEER_GAIN = 300
    elif abs(bearing_diff) > 20:
        STEER_GAIN = 900
    log_and_print(
        f"Bearing Difference : {bearing_diff:.1f} with {STEER_GAIN:.0f} Steer Gain")

    steer_output = STEER_GAIN * np.arctan(-1 * 2 * 3.5 * np.sin(np.radians(bearing_diff)) / 8)
    return steer_output, bearing_diff

def calculate_bearing_difference_for_speed_reduction(currentLocation, Current_Bearing):
    global wp
    RAD_TO_DEG_CONVERSION = 57.2957795
    next_wp = 4  # changed from 3 to 4

    if wp + next_wp < wp_len:
        off_y = -currentLocation[0] + waypoints[wp + next_wp][0]
        off_x = -currentLocation[1] + waypoints[wp + next_wp][1]
    else:
        off_y = -currentLocation[0] + waypoints[wp][0]
        off_x = -currentLocation[1] + waypoints[wp][1]

    # calculate bearing based on position error
    target_bearing = 90.00 + math.atan2(-off_y, off_x) * RAD_TO_DEG_CONVERSION

    # convert negative bearings to positive by adding 360 degrees
    if target_bearing < 0:
        target_bearing += 360.00

    Current_Bearing = heading
    while Current_Bearing is None:
        Current_Bearing = heading

    Current_Bearing = float(Current_Bearing)
    future_bearing_diff = Current_Bearing - target_bearing

    # normalize bearing difference to range between -180 and 180 degrees
    if future_bearing_diff < -180:
        future_bearing_diff += 360
    elif future_bearing_diff > 180:
        future_bearing_diff -= 360

    # log_and_print(f"    Future Bearing Difference for ({next_wp}) waypoint : {future_bearing_diff:.1f}")
    return future_bearing_diff

def navigation_output(latitude, longitude, Current_Bearing, curr):
    global counter, speed, wp, CW_flag, saw_pothole, const_speed, pothole_flag, acc_z, frame_count, pot_time, hit_time
    pothole_flag = 0              # if pothole camera is not connected
    flasher = 3  # 0 None, 1 Left, 2 Right, 3 Both ; For Indicator
    counter = (counter + 1) % 256

    #--- for traffic-light-------#
    global prev, cntr, depth_signal
    green_signal = ['go','goLeft','goRight','warning','warningLeft']
    red_signal = ['stop', 'stopLeft']
    null_signal = ['']

    # log_and_print(f"Current :- Latitude: {latitude} , Longitude: {longitude}")
    log_and_print(f"2D Standard Deviation(in cms): {100*(lat_delta+lng_delta)/2:.2f} cm")
    print(f"\t Flag Status : CW-{CW_flag} | SP-{pothole_flag} | TF-{depth_signal:.1f}, {prev}, {curr}")
    currentLocation = [latitude, longitude]
    distance_to_final_waypoint = (np.linalg.norm(np.array(currentLocation) - waypoints[-1]) * LAT_LNG_TO_METER)

    # to check if the final point is not less than 1m
    if (distance_to_final_waypoint > 1 and wp < wp_len):
        
        steer_output, bearing_diff = calculate_steer_output(
            currentLocation, Current_Bearing)
        steer_output *= -1.0

        future_bearing_diff = calculate_bearing_difference_for_speed_reduction(
            currentLocation, Current_Bearing)
        # next_bearing_diff = bearing_diff - future_bearing_diff
        # log_and_print(f"Future & Current Bearing diff : {next_bearing_diff:.1f}")

        const_speed = speed

        if wp < 10 or wp_len - 10 < wp:  # Slow start and end in the waypoints
            const_speed = reduction_factor * speed

        if abs(bearing_diff) > 5:
            const_speed = reduction_factor * speed
            log_and_print(f"Turning Speed from code : {const_speed:.0f} kmph")
        elif abs(future_bearing_diff) > 5:
            const_speed = reduction_factor * speed
            log_and_print(f"Predicted Turning Speed from code : {const_speed:.0f} kmph")


        # Pothole Detection
        # OLD SCHOOL speed bump and pothole
        # if (pothole_flag == 1):  # saw pothole/ speedbreaker
        #     frame_count = 0
        #     const_speed = pothole_speed
        #     print("               SpeedBump Detected")
        # # pothole not visible now, but under the hood
        # elif (saw_pothole == 1 and pothole_flag == 0):
        #     frame_count += 1                            # counting frames for 70
        #     if (frame_count < 70):
        #         const_speed = pothole_speed
        #         log_and_print(f" Speedbump Frame Count : {frame_count}")
        #     else:
        #         saw_pothole = 0
        #         const_speed = speed

        # if (pothole_flag == 1):       # Pothole just crossed from the vision but will the vehicle
        #     saw_pothole = pothole_flag

        # if pothole_flag == 1 or saw_pothole == 1:
        #     print(f"SpeedBump was           Detected")

        # Testing for single speedbump, actuation based on IMU
        if pothole_flag == 1:  # saw pothole/ speedbreaker
            frame_count = 0
            const_speed = pothole_speed
        # pothole not visible now, but under the hood
        elif (saw_pothole == 1 and pothole_flag == 0):
            print("\t INSIDE ....................... HOOD")
            if acc_z > 12:
                saw_pothole = 0
                const_speed = speed

            frame_count += 1
            const_speed = pothole_speed
            # print(f" Frame Count {frame_count}")
            if (frame_count > 25):
                saw_pothole = 0
                const_speed = speed
                print("\t False Detection of Speedbump")

            # if acc_z > 13:
            #     pot_time = hit_time
            #     print("$$$$$$$$$$$$$$$$$$ Jackpot")

            # if acc_z < 12 and pot_time + 1 == hit_time:
            #     saw_pothole = 0
            #     const_speed = speed

            # frame_count += 1
            # const_speed = pothole_speed
            # print(f" Frame Count {frame_count}")
            # if frame_count > 20:
            #     saw_pothole = 0
            #     const_speed = speed
            #     print("                         LOOK_AHEAD_DISTANCE     False Detection of Speedbump")

        # print(f"\t GNSS Z-Acceleration : {acc_z:.1f}")          

        if (pothole_flag == 1):  # Pothole just crossed from the vision but will the vehicle
            saw_pothole = pothole_flag

        # if pothole_flag == 1 or saw_pothole == 1:
        #     log_and_print(f"SpeedBump Detected via present:{pothole_flag}  past:{saw_pothole}")

        ########################################
        # Testing for Traffic Light through depth: 
        # Totally independent decision from Detection-code
        # Navigation-decisions will be taken at a distance of 10 meters or below
        # depth_signal : euclidean distance from the nearest traffic-light in the 3D-Region of Interest
        # print(f"\t Depth Signal: {depth_signal:.1f} m | {prev} -> {curr}")
        bias = 16
        if(15.0<=depth_signal and depth_signal <10000000.0):
            if prev in null_signal and curr in red_signal:
                const_speed = 0
                print("[] -> R")

            if  prev in red_signal and curr in null_signal:
                cntr += 1
                print("R -> []")

            if  prev in red_signal and curr in red_signal:
                const_speed = 0
                print("R -> R")

            if prev in null_signal and curr in null_signal:
                print("[] -> []")
                cntr += 1
                if cntr > 20: # number of frame delays = 10
                    const_speed = speed
                    cntr = 0

            if prev in null_signal and curr in green_signal:
                print("[] -> G")
                const_speed = speed
                cntr = 0 # resetting counter to improve one failure case

            if prev in green_signal and curr in null_signal:
                print("G -> []")
                const_speed = speed

            if prev in green_signal and curr in green_signal:
                print("G -> G")
                const_speed = speed

            if prev in red_signal and curr in green_signal:
                print("R -> G")
                const_speed = speed

            if prev in green_signal and curr in red_signal:
                print("G -> R")
                const_speed = 0
            # Prints the speed after a navigation decision is taken
            # print(f"Speed: {const_speed}")
        # testing for Traffic Light ends here  

        # Collision WARNING Starts
        if CW_flag == 1:  # Caution Flag
            const_speed = reduction_factor * speed
            log_and_print(f"Collision Warning Status : Caution | {const_speed:.0f} kmph")
        elif CW_flag == 2:  # Brake Flag
            const_speed = 0
            log_and_print(f"Collision Warning Status : Brake Signal | {const_speed:.0f} kmph")
        else:
            log_and_print(f"Collision Warning Status : Safe")
        # Collision WARNING ends here

        distance_to_nextpoint = (np.linalg.norm(np.array(currentLocation) - waypoints[wp]) * LAT_LNG_TO_METER)

        # For Testing
        log_and_print(f"{wp} out of {wp_len} | Next Coordinate distance : {distance_to_nextpoint:.1f} m")
        try:
            if wp < wp_len and distance_to_nextpoint < LOOK_AHEAD_DISTANCE:
                wp += 1
        except IndexError:
            log_and_print("Waypoint index out of range! - Seems like you are at wrong location or Inputted wrong waypoint")
    else:
        log_and_print(f"----- FINISHED  -----")
        log_and_print(f"Brake Activated")
        steer_output = 0
        const_speed = 0
        flasher = 0

    log_and_print(f"Speed: {const_speed:.0f} kmph")
    try:
        message = send_message_to_mabx(
            const_speed, steer_output, 0, flasher, counter)
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        log_and_print(f"Error sending message to MABX: {e}")


def mainLoop():
    while not rospy.is_shutdown():
        try:
            log_and_print(f"Current Coordinate No. : {wp}")
            log_and_print(" ")
            # log_and_print(f"Velocity in kmph as per GNSS= {current_vel:.0f} kmph")

            latitude = float(lat)
            longitude = float(lng)
            Current_Bearing = float(heading)
            
            curr = go_signal
            # time.sleep(SLEEP_INTERVAL/1000)
            navigation_output(latitude, longitude, Current_Bearing, curr)
            prev = curr

            time.sleep(SLEEP_INTERVAL / 1000)
        except ValueError as ve:
            log_and_print(f"ValueError occurred: {ve}")
        except IOError as ioe:
            log_and_print(f"IOError occurred: {ioe}")
        except KeyboardInterrupt:  # Currently not working
            log_and_print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)
            raise SystemExit

        # except Exception as e:
        #     log_and_print(f"An error occurred: {e}")


if __name__ == "__main__":
    global speed, reduction_factor, steer_output, counter, wp, file_path, pothole_flag, pothole_speed, depth_signal

    # Define the path(not relative path) to the waypoints file
    file_path = "/home/s186/Desktop/Solio-ADAS/Solio-Suzuki/navigation/Waypoints/Solio1/waypoints-2ndyear-lap1.txt"

    # log_dir = "devLogs"
    logger = setup_logging(file_path)
    logger.info("Main Code Starting")

    # Set sleep interval and lookahead distance
    SLEEP_INTERVAL = 100  # CHANGED FROM 5 TO 100
    LOOK_AHEAD_DISTANCE = 3

    # Define initial speeds, pothole_speed, reduction_factor
    speed = 12
    pothole_speed = 6
    reduction_factor = 0.7
    wp = 0
    steer_output = 0
    counter = 0
    go_signal= ''
    depth_signal = 0.0

    global saw_pothole, frame_count
    saw_pothole = 0
    frame_count = 0

    # Get the list of waypoints from the file
    waypoints = get_coordinates(file_path)
    wp_len = len(waypoints)

    # Start the main loop
    mainLoop()


# Last Error:
# Current Coordinate No. : 15

# 2D Standard Deviation(in cms): 83.36 cm
# Collision Warning Status : Safe
# Curve Speed from code : 8 kmph
# INSIDE ....................... HOOD
# Traceback (most recent call last):
#   File "test-navigation.py", line 507, in <module>
#     mainLoop()
#   File "test-navigation.py", line 458, in mainLoop
#     navigation_output(latitude, longitude, Current_Bearing)
#   File "test-navigation.py", line 367, in navigation_output
#     if acc_z < 12 and pot_time + 1 == hit_time:
# NameError: name 'pot_time' is not defined
