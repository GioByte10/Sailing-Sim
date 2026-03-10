### For validation tests with full U-joint. Runs sine wave on each joint, phase shifted by pi/4 to generate conical trajectory
### 
from asyncore import loop
from audioop import mul
from cProfile import run
from doctest import run_docstring_examples
from os.path import dirname, realpath
from pdb import post_mortem
import sys
from unittest import mock

from more_itertools import sample
arcsnake_v2_path = dirname(dirname(realpath(__file__)))
sys.path.append(arcsnake_v2_path)

import os
import can
import math as m
import numpy as np
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pyinstrument import Profiler

import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanMotor import CanMotor
from core.CanScrewMotor import CanScrewMotor


# This code is meant to generate a trajectory, and run a position loop based
# on that trajectory, and finally spit out the plots between the two

if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
    gear_ratio = 11
    joint1 = CanMotor(can0, 7, gear_ratio) #ID: 2
    joint1.motor_start()
    joint2 = CanMotor(can0, 8, gear_ratio)
    joint2.motor_start()

    read_s_pos_joint = [[] ,[]]
    read_m_pos_joint = [[] ,[]]
    read_speeds_joint = [[] ,[]]
    read_torque_joint = [[] ,[]]
    t_vec = []
    loop_vec = []

    input('Press Enter to read joint current pos')
    joint1_zero_pos = joint1.read_multiturn_position()
    joint2_zero_pos = joint2.read_multiturn_position()
    print('Joint 1 pos: ', joint1_zero_pos)
    print('Joint 2 pos: ', joint2_zero_pos)
    # input()

    # print(trajectory(0))

    amp = .1 * m.pi
    loop_rate = 1000 #Hz
    run_time = 40 #seconds


    def trajectory(t, zero_pos, phase_shift=0):
        traj = amp*m.sin(2*m.pi*t/20 + phase_shift) + zero_pos
        return traj

    # print(trajectory(0))
    # input()

    # joint2_start_pos = trajectory(0, joint2_zero_pos, math.pi/2)
    input("Press Enter to Set Joints to starting positions")
    joint1.pos_ctrl(trajectory(0, joint1_zero_pos), 1)
    joint2.pos_ctrl(trajectory(0, joint2_zero_pos, m.pi/2), 1)


    # time.sleep(2)

    

    input('Press Enter to start trajectory:')
    time.sleep(2)

    time_zero = time.time()
    t_start = 0
    err_count = 0

    while t_start < run_time:
        t_start = time.time() - time_zero
        print("Time: ", round(t_start,3), " seconds")
        t_vec.append(t_start)

        joint1.pos_ctrl(trajectory(t_start, joint1_zero_pos),7)
        joint2.pos_ctrl(trajectory(t_start, joint2_zero_pos, m.pi/2), 7)

        (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
        # joint_m_pos = joint1.read_multiturn_position()
        # joint_m_pos = 0
        read_s_pos_joint[0].append(joint_s_pos)
        # read_m_pos_joint.append(joint_m_pos)
        read_speeds_joint[0].append(joint_speed)
        read_torque_joint[0].append(joint_torque)

        (joint_torque, joint_speed, joint_s_pos) = joint2.read_motor_status()
        # # joint_m_pos = joint1.read_multiturn_position()
        # # joint_m_pos = 0
        read_s_pos_joint[1].append(joint_s_pos)
        # # read_m_pos_joint.append(joint_m_pos)
        read_speeds_joint[1].append(joint_speed)
        read_torque_joint[1].append(joint_torque)

        t_expected = 1/loop_rate
        t_execute = time.time() - time_zero - t_start
        print('Expected rate: ', round(1/t_expected,1), ' Hz')
        print('Execute rate: ', round(1/t_execute,1), 'Hz')
        print('---------------------')

        if t_execute > t_expected:
            err_count = err_count + 1
            print('Loop rate too fast.')
        elif t_execute < t_expected:
            time.sleep(t_expected - t_execute)
        
        loop_vec.append(1/t_execute)

        try:
            time.sleep(.00000001)
        except(KeyboardInterrupt) as e:
            print(e)
            joint1.motor_stop()
            # joint2.motor_stop()    
            break

    joint1.motor_stop()
    joint2.motor_stop()    

    print('Test Done')
    print('Error count: ', err_count)
    print('Recomend Loop Rate: ', min(loop_vec))

    cut_amount = 10
    for i in range(2):
        read_s_pos_joint[i] = read_s_pos_joint[i][cut_amount:-1]
        read_m_pos_joint[i] = read_m_pos_joint[i][cut_amount:-1]
        read_speeds_joint[i] = read_speeds_joint[i][cut_amount:-1]
        read_torque_joint[i] = read_torque_joint[i][cut_amount:-1]


    def smooth(y, box_pts):
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y,box,mode='same')
        return y_smooth

    traj_expected = [[], []]

    for i in range(len(t_vec)):
        traj_expected[0].append(6*trajectory(t_vec[i], joint1_zero_pos))
        traj_expected[1].append(6*trajectory(t_vec[i], joint2_zero_pos))


    def multiturn(singleturn_values):
        multiturn_values = singleturn_values.copy()
        count = 0
        diff_amount = 4
        amplifiy = abs(max(singleturn_values))
        for i in range(len(multiturn_values)-2):
            diff = singleturn_values[i+1] - singleturn_values[i]
            if diff < -diff_amount:
                count = count + 1
            elif diff > diff_amount:
                count = count - 1
            multiturn_values[i+1] = multiturn_values[i+1] + amplifiy*count
            multiturn_values[i+2] = multiturn_values[i+2] + amplifiy*count
        return multiturn_values

    read_m_pos_joint[0] = multiturn(read_s_pos_joint[0])
    read_m_pos_joint[1] = multiturn(read_s_pos_joint[1])

    # Joint 1
    fig, axs = plt.subplots(5)
    fig.suptitle("Joint 1")
    axs[0].plot(read_s_pos_joint[0],'b-')
    axs[0].set_title('Position (Singleloop)')
    axs[1].plot(read_m_pos_joint[0],'m-')
    axs[1].set_title('Position (Multiloop)')
    axs[2].plot(read_speeds_joint[0],'r-')
    axs[2].set_title('Speed')
    axs[3].plot(smooth(read_torque_joint[0],1000),'g-')
    axs[3].set_title('Torque')
    axs[4].plot(loop_vec)
    axs[4].plot(loop_rate*np.ones(len(loop_vec)))
    axs[4].set_title('Execute rate (Hz)')
    plt.show()

    plt.figure()
    plt.title("Joint 1")
    plt.plot(read_torque_joint[0],'r')
    plt.plot(smooth(read_torque_joint[0],50),'b')
    plt.plot(smooth(read_torque_joint[0],100),'g')
    plt.plot(smooth(read_torque_joint[0],300),'y')
    plt.plot(smooth(read_torque_joint[0],1000),'k')
    plt.plot(smooth(read_torque_joint[0],2000),'k')
    plt.plot(smooth(read_torque_joint[0],3000),'k')
    plt.show()

    plt.plot(smooth(read_torque_joint[0],3000),'k')
    plt.plot(read_speeds_joint[0],'r-')
    # plt.plot(read_m_pos_joint,'b-')
    plt.show()

    # Joint 2
    fig, axs = plt.subplots(5)
    fig.suptitle("Joint 2")
    axs[0].plot(read_s_pos_joint[1],'b-')
    axs[0].set_title('Position (Singleloop)')
    axs[1].plot(read_m_pos_joint[1],'m-')
    axs[1].set_title('Position (Multiloop)')
    axs[2].plot(read_speeds_joint[1],'r-')
    axs[2].set_title('Speed')
    axs[3].plot(smooth(read_torque_joint[1],1000),'g-')
    axs[3].set_title('Torque')
    axs[4].plot(loop_vec)
    axs[4].plot(loop_rate*np.ones(len(loop_vec)))
    axs[4].set_title('Execute rate (Hz)')
    plt.show()

    plt.figure()
    plt.title("Joint 2")
    plt.plot(read_torque_joint[1],'r')
    plt.plot(smooth(read_torque_joint[1],50),'b')
    plt.plot(smooth(read_torque_joint[1],100),'g')
    plt.plot(smooth(read_torque_joint[1],300),'y')
    plt.plot(smooth(read_torque_joint[1],1000),'k')
    plt.plot(smooth(read_torque_joint[1],2000),'k')
    plt.plot(smooth(read_torque_joint[1],3000),'k')
    plt.show()

    plt.plot(smooth(read_torque_joint[1],3000),'k')
    plt.plot(read_speeds_joint[1],'r-')
    # plt.plot(read_m_pos_joint,'b-')
    plt.show()

    # plt.plot(read_m_pos_joint, "r*")
    # plt.plot(traj_expected, 'b')
    # plt.show()


    core.CANHelper.cleanup("can0")

