#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
import ros_numpy as rnp
import time
import sys
import math
# from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from array import array
import dronekit_sitl
import time
from pymavlink.quaternion import QuaternionBase
from pd_control import motor_mixing_batt
from pd_control import pid_control
import matplotlib.pyplot as plt

def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    # in rad
    return roll, pitch, yaw




# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))


def request_message_interval(message_id, frequency_hz):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

def set_target_pos_ned(cmd_x, cmd_y, cmd_z):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
    

    """
    connection.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        connection.target_system, connection.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=3576, #position mask -- https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        x=cmd_x, y=cmd_y, z=cmd_z, # command xyz in meters
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
    # connection.mav.set_position_target_global_int_send(
    #     int(1e3 * (time.time() - boot_time)), # ms since boot
    #     connection.target_system, connection.target_component,
    #     coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    #     type_mask=3576, #position mask -- https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    #     lat_int=cmd_x, lon_int=cmd_y, alt=cmd_z, # command xyz in meters
    #     vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
    #     afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    #     # accelerations in NED frame [N], yaw, yaw_rate
    #     #  (all not supported yet, ignored in GCS Mavlink)
    # )


def set_target_attitude_throttle(roll, pitch, yaw, z_rate):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    connection.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        connection.target_system, connection.target_component,
        # allow throttle to be controlled by depth_hold mode
        0b00000111,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        [1,0,0,0],
        0, 0, 0, z_rate, 0 # roll rate, pitch rate, yaw rate, thrust/z_rate/z_ratio, thrust_3d
    )


    # 0.5 throttle means hover, this implies that thrust 0.5 == gravity == mass * 9.8 == 2.1 * 9.8 N
    # saturation 0.05 - 0.95
    # ratio->thrust      thrust == 2.1*9.8/(0.5-0.05) * (ratio - 0.05)
    # thrust->ratio      ratio == (thrust/(2.1*9.8/(0.5-0.05))) + 0.05
    # more details in website: https://docs.google.com/spreadsheets/d/1_75aZqiT_K1CdduhUe4-DjRgx3Alun4p8V2pt6vM5P8/edit#gid=0
    # more details in website: https://docs.google.com/spreadsheets/d/1dymF2TC28N2Mu_rkw31S5rMkdSGRO-Go9Q7r3SkbNuA/edit#gid=0



# Get some information !
def set_rc_channel_pwm(channel_id, pwm):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    for i in range(np.size(channel_id)):
        rc_channel_values[channel_id[i] - 1] = pwm[i]
        
    connection.mav.rc_channels_override_send(
        connection.target_system,                # target_system
        connection.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

def yaw3force_to_rpythrust(fx, fy, fz, yaw):
    
    roll = math.atan2(-fy, fz) / math.pi * 180
    pitch = math.atan2(fx, fz) / math.pi * 180
    yaw = yaw
    thrust = (fz/(2.1*9.8/(0.5-0.05))) + 0.05
    # return rpy in degree, thrust in ratio
    return roll, pitch, yaw, thrust


# Configure AHRS2 message to be sent at xxx Hz

request_message_interval(31, 400)
#attitude_quat msg id = 31
request_message_interval(32, 400)
#local position NED msg id = 32

ned_num = 0
quat_num = 0
boot_time = time.time()


target_mode =  0
connection.set_mode(target_mode)

time.sleep(1)

target_mode =  4
connection.set_mode(target_mode)





connection.arducopter_arm()
# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
connection.motors_armed_wait()
print('Armed!')



r_31_record = []
p_31_record = []
y_31_record = []
roll_speed_record = []
pitch_speed_record = []
yaw_speed_record = []
x_record = []
y_record = []
z_record = []




#takeoff 10m
connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        22, 0,
        0,  0, 0, 0, 10, 10, # Unused parameters
        10, # Target height in meters
    )
time_now = time.time()

time_record = []

#full throttle climb up for 15s
while True:
    if time.time() - time_now > 5:
        set_target_attitude_throttle(0,0,0,0.285)

    msg = connection.recv_match()
    if not msg:
        continue

    if msg.get_type() == 'ATTITUDE_QUATERNION':
        # print("mavutil.mavlink.MAVLINK_MSG_QUAT",31)
        r_31,p_31,y_31 = quart_to_rpy(msg.q2, msg.q3, msg.q4, msg.q1)
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("roll, pitch, yaw", r_31,p_31,y_31)
        # print("Message: %s" % msg)
        quat_num += 1
        r_31_record.append(r_31)
        p_31_record.append(p_31)
        y_31_record.append(y_31)


    if msg.get_type() == 'LOCAL_POSITION_NED':
        # print("mavutil.mavlink.LOCAL_POSITION_NED",32)
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
        ned_num += 1
        x_record.append(msg.x)
        y_record.append(msg.y)
        z_record.append(msg.z)
        time_record.append(time.time() - time_now)

    if time.time() - time_now > 10:
        print("quat_num",quat_num)
        print("ned_num",ned_num)
        target_mode =  4
        connection.set_mode(target_mode)
        break


while True:
    set_target_attitude_throttle(-0,0,0,0.285)
    msg = connection.recv_match()
    if not msg:
        continue

    if msg.get_type() == 'ATTITUDE_QUATERNION':
        # print("mavutil.mavlink.MAVLINK_MSG_QUAT",31)
        r_31,p_31,y_31 = quart_to_rpy(msg.q2, msg.q3, msg.q4, msg.q1)
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("roll, pitch, yaw", r_31,p_31,y_31)
        # print("Message: %s" % msg)
        quat_num += 1
        r_31_record.append(r_31)
        p_31_record.append(p_31)
        y_31_record.append(y_31)

    if msg.get_type() == 'LOCAL_POSITION_NED':
        # print("mavutil.mavlink.LOCAL_POSITION_NED",32)
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
        ned_num += 1
        x_record.append(msg.x)
        y_record.append(msg.y)
        z_record.append(msg.z)
        time_record.append(time.time() - time_now)
        print("z",msg.z)



    if time.time() - time_now > 30:
        print("quat_num",quat_num)
        print("ned_num",ned_num)
        
        break
    





#land
connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        21, 0, #mode land, empty
        0,  0, 0, 0, 0, 0,  0, # arbot-alt(0), landmode(0), empty, yaw, lat, lon, alt
    )


time.sleep(2)


connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        21, 0, #mode land, empty
        0,  0, 0, 0, 0, 0,  0, # arbot-alt(0), landmode(0), empty, yaw, lat, lon, alt
    )

while True:
    

    msg = connection.recv_match()
    if not msg:
        continue


    if msg.get_type() == 'LOCAL_POSITION_NED':
        # print("mavutil.mavlink.LOCAL_POSITION_NED",32)
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
        ned_num += 1
        x_record.append(msg.x)
        y_record.append(msg.y)
        z_record.append(msg.z)
        time_record.append(time.time() - time_now)
        print("z",msg.z)

        if z_record[-1] > -4:
            
            break
    

plt.plot(time_record, x_record, label = "x")
plt.plot(time_record, y_record, label = "y")
plt.plot(time_record, z_record, label = "z")
plt.legend()
plt.show()
    

plt.plot(r_31_record, label = "roll")
plt.plot(p_31_record, label = "pitch")
plt.plot(y_31_record, label = "yaw")
plt.legend()
plt.show()

plt.plot(roll_speed_record, label = "roll_v")
plt.plot(pitch_speed_record, label = "pitch_v")
plt.plot(yaw_speed_record, label = "yaw_v")
plt.legend()
plt.show()


    
connection.arducopter_disarm()
connection.motors_disarmed_wait()



