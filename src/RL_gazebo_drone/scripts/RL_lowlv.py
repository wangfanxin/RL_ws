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
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from array import array
import dronekit_sitl


############variables############
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()
# print('Connecting to vehicle on: %s' % (connection_string))
# vehicle = connect(connection_string, wait_ready=True)
print('Connecting to vehicle')
# vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=False)
print('wait ready')
vehicle.wait_ready(True, timeout=300)
print('Connected to vehicle')

vehicle.parameters['PLND_ENABLED'] = 1 #0 for vision, 1 for GPS
vehicle.parameters['PLND_TYPE'] = 1 #0 for vision, 1 for GPS
vehicle.parameters['PLND_EST_TYPE'] = 0 #0 for vision, 1 for GPS
vehicle.parameters['LAND_speed'] = 30 #cm/s

velocity = .5 #m/s
takeoff_height = 4 #m
#################################
print('Connected to vehicle')

newing_pub = rospy.Publisher('camera/color/image_new', Image, queue_size=10)

ids_to_find = [129, 72]
marker_sizes = [40, 20]
marker_heights = [10, 4]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_resolution = 640
vertical_resolution = 480

horizontal_fov = 62.2 * math.pi / 180 # 62.2 degrees for v2, 53.5 degrees for v1
vertical_fov = 48.8 * math.pi / 180 # 48.8 degrees for v2, 41.41 degrees for v1

found_count = 0
notfound_count = 0


dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[1061.6538553425996, 0.0, 640.5], [0.0, 1061.6538553425996, 360.5], [0.0, 0.0, 1.0]]


np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)


time_last = 0
time_to_wait = 0.1 # [s]

time_to_sleep =  5 # seconds the drone will wait after landing
sub = '' # initialize subscriber variable to make it global

############Functions############
def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon

    return math.sqrt((dLon*dLon) + (dLat*dLat))*1.113195e5

def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name == "GUIDED":
        currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
        if currentDistance < distanceToTargetLocation*.02:
            print("Reached target waypoint")
            time.sleep(2)
            break
    return None

def arm_and_takeoff(target_altitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Vehicle is armable")
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        print(" Waiting for drone to enter GUIDED flight mode...")
        time.sleep(1)

    print("Vehicle now in GUIDED mode. Ready for taking off!")

    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    vehicle.simple_takeoff(target_altitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=target_altitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    print('Target altitude reached')

    return None

def send_local_ned_velocity(vx, vy, vz):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, #-- BITMASK -> Consider only the velocities
        0, 0, 0, #-- POSITION
        vx, vy, vz, #-- VELOCITY
        0, 0, 0, #-- ACCELERATIONS
        0, 0)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_land_message(x, y):
    """
    Send MAVLink LAND command to vehicle
    """
    msg = vehicle.message_factory.landing_target_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #command
        x, y, 
        0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def msg_receiver(msg):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, sub

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(msg) # convert ROS image to numpy array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY) # convert to grayscale

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(gray_img, dictionary=aruco_dict, parameters=parameters)


        altitude = vehicle.location.global_relative_frame.alt ## meters
        id_to_find = 0
        marker_height = 0
        marker_size = 0

        if altitude > marker_heights[1]:
            id_to_find = ids_to_find[0]
            marker_height = marker_heights[0]
            marker_size = marker_sizes[0]
        elif altitude < marker_heights[1]:
            id_to_find = ids_to_find[1]
            marker_height = marker_heights[1]
            marker_size = marker_sizes[1]

        ids_array_index = 0
        found_id = 0
        print("Looking for marker: " + str(id_to_find))

        if vehicle.mode != 'LAND':
            vehicle.mode = VehicleMode('LAND')
            while vehicle.mode != 'LAND':
                time.sleep(1)
            print('Vehicle is on LAND mode.')

        ### have landed the drone no need to track images
        if vehicle.armed == False:
            sub.unregister()
            return None

        try:
            if ids is not None:
                for id in ids:
                    if id == id_to_find:
                        corners_single = [corners[ids_array_index]]
                        corners_single_np = np.asarray(corners_single)
                        ret = aruco.estimatePoseSingleMarkers(corners_single, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # get rotation and translation vectors
                        x = '{:.2f}'.format(tvec[0]) # get x error/dist between camera and marker in cm
                        y = '{:.2f}'.format(tvec[1]) # get y error/dist between camera and marker in cm
                        z = '{:.2f}'.format(tvec[2]) # get z error/dist between camera and marker in cm

                        x_sum = 0
                        y_sum = 0

                        x_sum += corners_single_np[0][0][0][0] + corners_single_np[0][0][1][0] + corners_single_np[0][0][2][0] + corners_single_np[0][0][3][0]
                        y_sum += corners_single_np[0][0][0][1] + corners_single_np[0][0][1][1] + corners_single_np[0][0][2][1] + corners_single_np[0][0][3][1]
                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        x_angle = (x_avg - horizontal_resolution*.5)*horizontal_fov / horizontal_resolution
                        y_angle = (y_avg - vertical_resolution*.5)*vertical_fov / vertical_resolution

                        if vehicle.mode != "LAND":
                            vehicle.mode = VehicleMode("LAND")
                            while vehicle.mode != "LAND":
                                print(" Waiting for drone to enter LAND flight mode...")
                                time.sleep(1)
                            print("Vehicle now in LAND mode. Ready for landing!")
                            send_land_message(x_angle, y_angle)
                        else:
                            send_land_message(x_angle, y_angle)
                        
                        marker_position = 'Marker position: x = ' + x + ' y = ' + y + ' z = ' + z

                        aruco.drawDetectedMarkers(np_data, corners) # draw a square around the markers
                        aruco.drawAxis(np_data, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff, rvec=rvec, tvec=tvec, length=10) # draw axis
                        cv2.putText(np_data, marker_position, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 0), thickness=1) # draw text
                        print(marker_position)
                        print('FOUND MARKER: ' + str(found_count) + ' NOT FOUND: ' + str(notfound_count))
                        found_count += 1
                        found_id += 1
                        break
                    ids_array_index += ids_array_index
                if found_id == 0:
                    notfound_count += notfound_count
            else:
                notfound_count += 1
        except Exception as e:
            print('Target not found')
            print(e)
            notfound_count += 1
        new_msg = rnp.msgify(Image, np_data, encoding='bgr8') # convert numpy array to ROS image
        newing_pub.publish(new_msg) # publish the new image
        time_last = time.time()
    else:
        return None


def lander():
    global sub
    rospy.init_node('drone_node', anonymous=True)
    sub = rospy.Subscriber('camera/color/image_raw', Image, msg_receiver)
    
    while vehicle.armed == True:
        time.sleep(1)
    return None



if __name__ == '__main__':
    try:
        lat_home = vehicle.location.global_relative_frame.lat
        lon_home = vehicle.location.global_relative_frame.lon

        wp_home = LocationGlobalRelative(lat_home, lon_home, takeoff_height)
        wp_dest = LocationGlobalRelative(-35.36303741, 149.1652374, takeoff_height) # +25m

        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        goto(wp_dest)

        lander()
        print("")
        print("---------------------------------------")
        print("Arrived at the destination")
        time.sleep(time_to_sleep)

        arm_and_takeoff(takeoff_height)
        goto(wp_home)
        lander()
        print("")
        print("---------------------------------------")
        print("Arrived at the home position")



    except rospy.ROSInterruptException:
        print('ROS Interrupt Exception')
        pass