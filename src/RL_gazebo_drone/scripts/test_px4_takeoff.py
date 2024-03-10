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
    # in rad
    return roll, pitch, yaw



def rpy_to_quat(r,p,y):
    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler('xyz', [r,p,y], degrees=False)
    rot_quat = rot.as_quat() # x,y,z,w
    quat = [rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]]  # w,x,y,z
    return quat

def request_message_interval(connection, message_id, frequency_hz):
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


# def set_target_attitude_throttle(connection, boot_time, thrust_3d_ned):
#     """ Sets the target attitude while in depth-hold mode.

#     'roll', 'pitch', and 'yaw' are angles in degrees.

#     """
#     connection.mav.set_attitude_target_send(
#         int(1e3 * (time.time() - boot_time)), # ms since boot
#         connection.target_system, connection.target_component,
#         # allow throttle to be controlled by depth_hold mode
#         0b11100111,
#         # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
#         [1,0,0,0],0,0,0, # quat, roll rate, pitch rate, yaw rate
#         0, [(thrust_3d_ned[0]/(2.1*9.8/(0.5-0.05))) + 0.06, 
#             (thrust_3d_ned[1]/(2.1*9.8/(0.5-0.05))) + 0.06, 
#             (thrust_3d_ned[2]/(2.1*9.8/(0.5-0.05))) + 0.06] #  thrust/z_rate/z_ratio, thrust_3d_ned
#     )


    # 0.5 throttle means hover, this implies that thrust 0.5 == gravity == mass * 9.8 == 2.1 * 9.8 N
    # saturation 0.05 - 0.95
    # ratio->thrust      thrust == 2.1*9.8/(0.5-0.05) * (ratio - 0.05)
    # thrust->ratio      ratio == (thrust/(2.1*9.8/(0.5-0.05))) + 0.05
    # more details in website: https://docs.google.com/spreadsheets/d/1_75aZqiT_K1CdduhUe4-DjRgx3Alun4p8V2pt6vM5P8/edit#gid=0
    # more details in website: https://docs.google.com/spreadsheets/d/1dymF2TC28N2Mu_rkw31S5rMkdSGRO-Go9Q7r3SkbNuA/edit#gid=0


def set_target_attitude_throttle_easy(connection, boot_time, action):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """

    quat, thrust_k = yaw3force_to_quat_thrust(action[0],action[1],action[2], 0)

    connection.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        connection.target_system, connection.target_component,
        # allow throttle to be controlled by depth_hold mode
        0b00000111, #use throttle_k and rpy only
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        [1,0,0,0],0,0,0, # quat, roll rate, pitch rate, yaw rate
        thrust_k, 0 #  thrust/z_rate/z_ratio, thrust_3d_ned
    )


    # 0.5 throttle means hover, this implies that thrust 0.5 == gravity == mass * 9.8 == 2.1 * 9.8 N
    # saturation 0.05 - 0.95
    # ratio->thrust      thrust == 2.1*9.8/(0.5-0.05) * (ratio - 0.05)
    # thrust->ratio      ratio == (thrust/(2.1*9.8/(0.5-0.05))) + 0.05
    # more details in website: https://docs.google.com/spreadsheets/d/1_75aZqiT_K1CdduhUe4-DjRgx3Alun4p8V2pt6vM5P8/edit#gid=0
    # more details in website: https://docs.google.com/spreadsheets/d/1dymF2TC28N2Mu_rkw31S5rMkdSGRO-Go9Q7r3SkbNuA/edit#gid=0




def yaw3force_to_quat_thrust(fx, fy, fz, yaw):
    
    f_total = np.sqrt(fx**2 + fy**2 + fz**2)


    roll = -np.arcsin(fy/f_total) # phi
    pitch = np.arctan2(fx, -fz) # theta   y, x == y/x ??????????
    yaw = yaw
    
    while roll > np.pi  or roll < -np.pi:
        if roll > np.pi :
            roll -= np.pi *2
        elif roll < -np.pi:
            roll += np.pi *2

    while pitch > np.pi  or pitch < -np.pi:
        if pitch > np.pi :
            pitch -= np.pi *2
        elif pitch < -np.pi:
            pitch += np.pi *2

    print('rpy',roll, pitch, yaw)


    quat = rpy_to_quat(roll, pitch, yaw)

    # quat_ned = quat_enu2ned(quat)

    thrust_k = (f_total/(2*9.81/(0.29-0.00))) + 0.00
    # thrust_k = (-fz/(1.75*9.81/(0.5-0.06))) + 0.06
    # return rpy in degree, thrust in ratio
    print(quat, thrust_k)
    return quat, thrust_k


# Configure AHRS2 message to be sent at xxx Hz


def test(args):
    sys.path.append('/home/adrian/RL_ws/src/RL_gazebo_drone/scripts/checkpoints/')
    env = QuadrotorEnv()
    env.control_mode = 'takeoff'
    agent = SAC(env.observation_space.shape[0], env.action_space, args)
    agent.load_model(args.load_model_path, evaluate=True)

    state = env.reset()
    done = False
    state[10:] = [0,0,0,0]
    angles = []

    # Start a connection listening on a UDP port
    connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # Wait for the first heartbeat to set the system and component ID of remote system for the link
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    #message request definitions

    request_message_interval(connection, 31, 200)
    #attitude_quat msg id = 31
    request_message_interval(connection, 32, 200)
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
    r_31_record = [0]
    p_31_record = [0]
    y_31_record = [0]
    roll_speed_record = [0]
    pitch_speed_record = [0]
    yaw_speed_record = [0]
    x_record = [0]
    y_record = [0]
    z_record = [0]

    while not done:
        
        msg = connection.recv_match()
        if not msg:
            continue
        
        if msg.get_type() == 'LOCAL_POSITION_NED':
            # print("mavutil.mavlink.LOCAL_POSITION_NED",32)
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("Message: %s" % msg)
            ned_num += 1
            # append ned position into enu records
            x_record[0] = msg.x
            y_record[0] = msg.y
            z_record[0] = msg.z
            # append ned velocity into enu records
        if x_record[0] != 0:
            break
        

    vx_record = [0]
    vy_record = [0]
    vz_record = [0]
    time_last = time.time()
    time_start = time.time()

    time_record = []
    flag = 0

    while not done:
        
        msg = connection.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE_QUATERNION':
            # print("mavutil.mavlink.MAVLINK_MSG_QUAT",31)
            quat_num += 1
            # append ned velocity into enu records
            quat_ned_collect = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
            

            r_31,p_31,y_31 = quart_to_rpy(msg.q2, msg.q3, msg.q4, msg.q1)
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("roll, pitch, yaw", r_31,p_31,y_31)
            # print("Message: %s" % msg)
            quat_num += 1
            r_31_record.append(r_31)
            p_31_record.append(p_31)
            y_31_record.append(y_31)

            # quat_enu_collect = quat_ned2enu(quat_ned_collect)

            # # quat_enu_collect = quat_ned_collect
            # quat_enu_collect = np.array([1,0,0,0])
            
            # print('quat_ned_collect')
            # print(quat_ned_collect)
            # print('quat_enu_collect')
            # print(quat_enu_collect)

        if msg.get_type() == 'LOCAL_POSITION_NED':
            # print("mavutil.mavlink.LOCAL_POSITION_NED",32)
            # print("\n\n*****Got message: %s*****" % msg.get_type())
            # print("Message: %s" % msg)
            ned_num += 1
            # append ned position into enu records
            x_record.append(msg.x - x_record[0])
            y_record.append(msg.y - y_record[0])
            z_record.append(msg.z - z_record[0])
            # append ned velocity into enu records
            vx_record.append(msg.vx)
            vy_record.append(msg.vy)
            vz_record.append(msg.vz)

            time_record.append(time.time() - time_start)
        
        #frequency control
        time_now = time.time()

        if time_now - time_last < 0.0175:
            continue
        time_last = time_now
        # state[0:10] = [x_record[-1],y_record[-1],z_record[-1],vx_record[-1],vy_record[-1],vz_record[-1],
        #                quat_enu_collect[0],quat_enu_collect[1], quat_enu_collect[2], quat_enu_collect[3]]
        # state[0:10] = [0,0,z_record[-1],vx_record[-1],vy_record[-1],vz_record[-1],
        #                quat_enu_collect[0],quat_enu_collect[1], quat_enu_collect[2], quat_enu_collect[3]]
        
        
        ang_vel = 0.05 / 3.0
        theta = ang_vel * time_record[-1] / np.pi * 180

        state[10] = 3.0 * np.cos(theta) # x
        state[11] = 3.0 * np.sin(theta) # y
        state[12] = 0.05 * np.sin(theta) # dx
        state[13] = 0.05 * np.cos(theta) # dy

        state[0:10] = [x_record[-1],y_record[-1],z_record[-1],vx_record[-1],vy_record[-1],vz_record[-1],
                       quat_ned_collect[0],quat_ned_collect[1], quat_ned_collect[2], quat_ned_collect[3]]
        state[0:10] = [0.01*np.random.rand(),0.01*np.random.rand(),z_record[-1],0.01*np.random.rand(),0.01*np.random.rand(),vz_record[-1],
                       quat_ned_collect[0],quat_ned_collect[1], quat_ned_collect[2], quat_ned_collect[3]]
        
        action = agent.select_action(state, eval=True)
        # print(x_record[-1],y_record[-1],z_record[-1],action)
        print(action)
        flag = flag+1

        # set_target_attitude_throttle(connection, boot_time, [action[1], action[0], action[2]])
        set_target_attitude_throttle_easy(connection, boot_time, action)

        if time.time() - time_start > 50:
            done = True
        # next_state, reward, done, _ = env.step(action)
        # state = next_state

        # q = np.array(state[6:10])
        # quaternion = Quaternion(q[0], q[1], q[2], q[3])
        # yaw, pitch, roll  = quaternion.yaw_pitch_roll
        # angles.append([roll, pitch, yaw])
        # state = next_state
    # states = np.array(states)
    # angles = np.array(angles)
    # render(states, angles)

    time.sleep(2)


    connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            21, 0, #mode land, empty
            0,  0, 0, 0, 0, 0,  0, # arbot-alt(0), landmode(0), empty, yaw, lat, lon, alt
        )

    plt.plot(time_record,x_record[1:], label='x')
    plt.plot(time_record,y_record[1:], label='y')
    plt.plot(time_record,z_record[1:], label='z')
    print(flag)

    plt.show()

    plt.plot(r_31_record[1:], label='roll')
    plt.plot(p_31_record[1:], label='pitch')
    plt.plot(y_31_record[1:], label='yaw')

    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PyTorch Soft Actor-Critic Args')
    parser.add_argument('--num_episodes', type=int, nargs='?', default=800, help='total number of episode')
    parser.add_argument('--updates_per_step', type=int, nargs='?', default=1, help='total number of updates per step')
    parser.add_argument('--batch_size', type=int, nargs='?', default=256, help='batch size (default: 256)')
    parser.add_argument('--replay_size', type=int, default=10000000, metavar='N',
                        help='size of replay buffer (default: 10000000)')
    parser.add_argument('--hidden_size', type=int, nargs='?', default=256, metavar='N',
                        help='hidden size (default: 256)')
    parser.add_argument('seed', type=int, nargs='?', default=12345, help='random seed')
    parser.add_argument('--gamma', type=float, nargs='?', default=0.99, metavar='G',
                        help='discount factor for reward (default: 0.99)')
    parser.add_argument('--tau', type=float, nargs='?', default=0.005, metavar='G',
                        help='target smoothing coefficient(τ) (default: 0.005)')
    parser.add_argument('--alpha', type=float, nargs='?', default=0.2, metavar='G',
                        help='Temperature parameter α determines the relative importance of the entropy\
                                term against the reward (default: 0.2)')
    parser.add_argument('--target_update_interval', type=int, nargs='?', default=1, metavar='N',
                        help='Value target update per no. of updates per step (default: 1)')
    parser.add_argument('--automatic_entropy_tuning', type=bool, nargs='?', default=True, metavar='G',
                        help='Automatically adjust α (default: False)')
    parser.add_argument('--lr', type=float, nargs='?', default=0.0003, metavar='G',
                        help='learning rate (default: 0.0003)')
    parser.add_argument('--policy', default="Gaussian", type=str,  nargs='?', help='Policy Type: Gaussian | Deterministic')
    parser.add_argument('--start_steps', type=int, default=10000, metavar='N',
                    help='Steps sampling random actions (default: 10000)')
    
    parser.add_argument('--env_name', type=str, nargs='?', default='Quadrotor', help='env name')
    parser.add_argument('--output', default='output', type=str, help='')
    parser.add_argument('--control_mode', default='takeoff', type=str, help='')
    parser.add_argument('--load_model', default=False, type=bool, help='load trained model for train function')
    parser.add_argument('--load_model_path', default='/home/adrian/RL_ws/src/RL_gazebo_drone/scripts/checkpoints/takeoff_NED_25m_50hz_03', type=str, help='path to trained model (caution: do not use it for model saving)')
    parser.add_argument('--save_model_path', default='checkpoints', type=str, help='path to save model')
    parser.add_argument('--mode', default='test', type=str, help='train or evaluate')
    

    args = parser.parse_args()
    test(args)