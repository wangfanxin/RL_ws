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



def motor_mixing_batt(thrustcmd, battvolt, real_or_sitl):
    testvolt = 16.6; # battery voltage when testing for motormixing curve
    battcompscale = testvolt / battvolt;

    w = np.zeros(4);

    if real_or_sitl == 0: # SITL
        L = 0.25;
        D = 0.25;
        a_F = 0.0014597;
        b_F = 0.043693;
        a_M = 0.000011667;
        b_M = 0.0059137;
    elif real_or_sitl == 1: # parameters for real drone
        L = 0.175;
        D = 0.131;  
        a_F = 0.0009251;
        b_F = 0.021145;
        a_M = 0.00001211;
        b_M = 0.0009864;
    if real_or_sitl == 0 or real_or_sitl == 1:
        w0 = (-b_F + math.sqrt(b_F * b_F + a_F * thrustcmd[0])) / 2 / a_F;
        c_F = 2 * a_F * w0 + b_F;
        c_M = 2 * a_M * w0 + b_M;
        thrust_biased = 2 * thrustcmd[0] - 4 * b_F * w0;
        M1 = thrustcmd[1];
        M2 = thrustcmd[2];
        M3 = thrustcmd[3];
    
        # motor mixing for x layout
        c_F4_inv = 1 / (4 * c_F);
        c_FL_inv = 1 / (2 * L * c_F);
        c_FD_inv = 1 / (2 * D * c_F);
        c_M4_inv = 1 / (4 * c_M);

        w[0] = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
        w[1] = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
        w[2] = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
        w[3] = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

        # 2nd shot on solving for motor speed
        # output: VectorN<float, 4> new motor speed
        # input: a_F, b_F, a_M, b_M, w, L, D, thrustMomentCmd
    
        # motor speed after the second iteration
        w2 = iterativeMotorMixing(w, thrustcmd, a_F, b_F, a_M, b_M, L, D);

        # motor speed after the third iteration
        w3 = iterativeMotorMixing(w2, thrustcmd, a_F, b_F, a_M, b_M, L, D);
    
        # logging
        # AP::logger().Write("L1A1", "m1,m2,m3,m4", "ffff",
        #                      (double)w[0],
        #                      (double)w[1],
        #                      (double)w[2],
        #                      (double)w[3]);
        # AP::logger().Write("L1A2", "m1,m2,m3,m4", "ffff",
        #                      (double)w2[0],
        #                      (double)w2[1],
        #                      (double)w2[2],
        #                      (double)w2[3]);
        # AP::logger().Write("L1A3", "m1,m2,m3,m4", "ffff",
        #                      (double)w3[0],
        #                      (double)w3[1],
        #                      (double)w3[2],
        #                      (double)w3[3]);
        return w3 * battcompscale;

    elif real_or_sitl == 2: #for holybro
        a_F_1 =0.001993;
        b_F_1 =-0.006193;
        a_F_2 =0.0002735;
        b_F_2 =0.1589;
        c_F_2 =-3.5860;
        thrust = thrustcmd[0];
        M1 = thrustcmd[1];
        M2 = thrustcmd[2];
        M3 = thrustcmd[3];
        f1 = 0.25*thrust+(-1.4286)*M1+(1.4286)*M2+(10.1446)*M3;
        f2 = 0.25*thrust+(1.4286)*M1+(-1.4286)*M2+(10.1446)*M3;
        f3 = 0.25*thrust+(1.4286)*M1+(1.4286)*M2+(-10.1446)*M3;
        f4 = 0.25*thrust+(-1.4286)*M1+(-1.4286)*M2+(-10.1446)*M3;
        
        if (f1<0):
            w[0] = 0;
        elif (f1>5.0428):
            w[0]=(-b_F_2+math.sqrt(b_F_2*b_F_2+4*a_F_2*(f1-c_F_2)))/(2*a_F_2);
        else:
            w[0] = (-b_F_1+math.sqrt(b_F_1*b_F_1+4*a_F_1*f1))/(2*a_F_1);
        
        if (f2<0):
            w[1] = 0;
        elif (f2>5.0428):
            w[1]=(-b_F_2+math.sqrt(b_F_2*b_F_2+4*a_F_2*(f2-c_F_2)))/(2*a_F_2);
        else:
            w[1] = (-b_F_1+math.sqrt(b_F_1*b_F_1+4*a_F_1*f2))/(2*a_F_1);
        
        if (f3<0):
            w[2] = 0;
        elif (f3>5.0428):
            w[2]=(-b_F_2+math.sqrt(b_F_2*b_F_2+4*a_F_2*(f3-c_F_2)))/(2*a_F_2);
        else:
            w[2] = (-b_F_1+math.sqrt(b_F_1*b_F_1+4*a_F_1*f3))/(2*a_F_1);

        if (f4<0):
            w[3] = 0;
        elif (f4>5.0428):
            w[3]=(-b_F_2+math.sqrt(b_F_2*b_F_2+4*a_F_2*(f4-c_F_2)))/(2*a_F_2);
        else:
            w[3] = (-b_F_1+math.sqrt(b_F_1*b_F_1+4*a_F_1*f4))/(2*a_F_1);
        

        return w * battcompscale;


def iterativeMotorMixing(w_input, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D):
    # The function iterativeMotorMixing computes the motor speed to achieve the desired thrustMoment command
    # input:
    # VectorN<float, 4> w_input -- initial guess of the motor speed (linearizing point)
    # VectorN<float, 4> thrustMomentCmd -- desired thrust and moment command
    # float a_F -- 2nd-order coefficient for motor's thrust-speed curve
    # float b_F -- 1st-order coefficient for motor's thrust-speed curve
    # float a_M -- 2nd-order coefficient for motor's torque-speed curve
    # float b_M -- 1st-order coefficient for motor's torque-speed curve
    # float L -- longer distance between adjacent motors
    # float D -- shorter distance between adjacent motors

    # output:
    # VectorN<float, 4> w_new new motor speed

    w_new = np.zeros(4); # new motor speed

    w1_square = w_input[0] * w_input[0];
    w2_square = w_input[1] * w_input[1];
    w3_square = w_input[2] * w_input[2];
    w4_square = w_input[3] * w_input[3];

    c_F1 = -a_F * w1_square;
    c_F2 = -a_F * w2_square;
    c_F3 = -a_F * w3_square;
    c_F4 = -a_F * w4_square;

    c_M1 = -a_M * w1_square;
    c_M2 = -a_M * w2_square;
    c_M3 = -a_M * w3_square;
    c_M4 = -a_M * w4_square;

    d_F1 = 2 * a_F * w_input[0] + b_F;
    d_F2 = 2 * a_F * w_input[1] + b_F;
    d_F3 = 2 * a_F * w_input[2] + b_F;
    d_F4 = 2 * a_F * w_input[3] + b_F;

    d_M1 = 2 * a_M * w_input[0] + b_M;
    d_M2 = 2 * a_M * w_input[1] + b_M;
    d_M3 = 2 * a_M * w_input[2] + b_M;
    d_M4 = 2 * a_M * w_input[3] + b_M;

    coefficientMatrix = np.zeros([4,4]);


    coefficientMatrix[0,0] = d_F1;
    coefficientMatrix[0,1] = d_F2;
    coefficientMatrix[0,2] = d_F3;
    coefficientMatrix[0,3] = d_F4;

    coefficientMatrix[1,0] = -d_F1;
    coefficientMatrix[1,1] = d_F2;
    coefficientMatrix[1,2] = d_F3;
    coefficientMatrix[1,3] = -d_F4;

    coefficientMatrix[2,0] = d_F1;
    coefficientMatrix[2,1] = -d_F2;
    coefficientMatrix[2,2] = d_F3;
    coefficientMatrix[2,3] = -d_F4;

    coefficientMatrix[3,0] = d_M1;
    coefficientMatrix[3,1] = d_M2;
    coefficientMatrix[3,2] = -d_M3;
    coefficientMatrix[3,3] = -d_M4;


    coefficientMatrixInv = np.linalg.inv(coefficientMatrix);

    shiftedCmd = np.zeros([4,1]);
    shiftedCmd[0] = thrustMomentCmd[0] - c_F1 - c_F2 - c_F3 - c_F4;
    shiftedCmd[1] = 2 * thrustMomentCmd[1] / L + c_F1 - c_F2 - c_F3 + c_F4;
    shiftedCmd[2] = 2 * thrustMomentCmd[2] / D - c_F1 + c_F2 - c_F3 + c_F4;
    shiftedCmd[3] = thrustMomentCmd[3] - c_M1 - c_M2 + c_M3 + c_M4;

    w_new = np.matmul(coefficientMatrixInv, shiftedCmd)


    return w_new;



def pid_control(reference, state_now, last_error_d, last_error_i, pid):
    p = pid[0]
    i = pid[1]
    d = pid[2]

    error = reference - state_now

    error_d = error - last_error_d

    error_i = error + last_error_i + error

    control = p*error + d* error_d + i*error_i
    
    return control, error_d, error_i;