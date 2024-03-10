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

newing_pub = rospy.Publisher('camera/color/image_new', Image, queue_size=10)

id_to_find = 72
marker_size = 20 #- [cm]

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
time_to_wait = 1.0 # [s]

def msg_receiver(msg):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(msg) # convert ROS image to numpy array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY) # convert to grayscale

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(gray_img, dictionary=aruco_dict, parameters=parameters)

        try:
            if ids is not None:
                if ids[0] == id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # get rotation and translation vectors
                    x = '{:.2f}'.format(tvec[0]) # get x, y, z coordinates
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    marker_position = 'Marker position: x = ' + x + ' y = ' + y + ' z = ' + z

                    aruco.drawDetectedMarkers(np_data, corners) # draw a square around the markers
                    aruco.drawAxis(np_data, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff, rvec=rvec, tvec=tvec, length=10) # draw axis
                    cv2.putText(np_data, marker_position, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 0), thickness=1) # draw text

                    found_count += 1
                else:
                    notfound_count += 1
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


def subscriber():
    rospy.init_node('drone_node', anonymous=True)
    rospy.Subscriber('camera/color/image_raw', Image, msg_receiver)
    rospy.spin()



if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass