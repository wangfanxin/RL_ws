from dynamics import QuadrotorEnv, render1
from agent import SAC
from pyquaternion import Quaternion
import numpy as np
import argparse
import numpy as np
from sensor_msgs.msg import Image
import time
import sys
import math
# from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from array import array
import time
from pymavlink.quaternion import QuaternionBase
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation



# define quat multiply, wxyz
def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


#quat from ned to enu
def quat_ned2enu(quat_ned):
    ned2enu_quaternion = np.array([
    [0],
    [-np.sqrt(2)/2],
    [-np.sqrt(2)/2],
    [0]
])
    quat_enu = quaternion_multiply(ned2enu_quaternion, quat_ned)

    return quat_enu


#quat from enu to ned
def quat_enu2ned(quat_enu):
    ned2enu_quaternion = np.array([
    [0],
    [np.sqrt(2)/2],
    [np.sqrt(2)/2],
    [0]
])  # w,x,y,z
    quat_ned = quaternion_multiply(ned2enu_quaternion, quat_enu)

    return quat_ned



def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw



def rpy_to_quat(r,p,y):
    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler('xyz', [r,p,y], degrees=False)
    rot_quat = rot.as_quat() # x,y,z,w
    quat = [rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]]  # w,x,y,z
    return quat



def test():
    north_dir_ned = np.array([1, 0, 0])
    north_dir_enu = np.array([0, 1, 0])
    quat_north_dir_enu = np.array([1, 0, 0, 0]) # w,x,y,z
    quat_north_dir_ned = quat_enu2ned(quat_north_dir_enu)
    print(quat_north_dir_ned)

if __name__ == '__main__':
    
    test()