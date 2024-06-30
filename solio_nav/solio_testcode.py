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

def set_speed(speed):
    speed = speed * 128
    high_byte_speed = (int)(speed) >> 8
    low_byte_speed = (int)(speed) & 0xFF
    return high_byte_speed, low_byte_speed

def calc_checksum(message_bytes):
    checksum = 0
    for m in message_bytes:
        checksum += m
    checksum = (0x00 - checksum) & 0x000000FF
    checksum = checksum & 0xFF
    return checksum

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
    print(
        "================================================================")
    return message


def nav(const_speed,steer_output,flasher,counter):
    try:
        message = send_message_to_mabx(
            const_speed, steer_output, 0, flasher, counter)
        mabx_socket.sendto(message, mabx_addr)
    except Exception as e:
        print(f"Error sending message to MABX: {e}")

if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            counter = (counter + 1) % 256
            const_speed = 10.0
            steer_output = 0.0
            flasher = 3
            nav(const_speed,steer_output,flasher,counter)
        except ValueError as ve:
            print(f"ValueError occurred: {ve}")
        except IOError as ioe:
            print(f"IOError occurred: {ioe}")
        except KeyboardInterrupt:  # Currently not working
            print("Autonomous Mode is terminated manually!")
            message = send_message_to_mabx(0, 0, 0, 0, counter)
            mabx_socket.sendto(message, mabx_addr)
            raise SystemExit

        # except Exception as e:
        #     log_and_print(f"An error occurred: {e}")
