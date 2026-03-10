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
    gear_ratio = 6
    joint1 = CanMotor(can0, 1, gear_ratio) #ID: 2
    joint1.motor_start()

    read_s_pos_joint = []
    read_m_pos_joint = []
    read_speeds_joint = []
    read_torque_joint = []
    t_vec = []
    loop_vec = []

    zero_pos = 0
    amp = .1 * m.pi
    loop_rate = 1000 #Hz
    run_time = 10 #seconds


    def trajectory(t):
        traj = amp*m.sin(2*m.pi*t/10)+zero_pos
        return traj


    print('Starting Test:')
    time.sleep(1)

    joint1.pos_ctrl(trajectory(0),5)
    time.sleep(1)

    time_zero = time.time()
    t_start = 0
    err_count = 0

    while t_start < run_time:
        t_start = time.time() - time_zero
        print("Time: ",round(t_start,3), " seconds")
        t_vec.append(t_start)

        joint1.torque_ctrl(trajectory(t_start))


        (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
        # joint_m_pos = joint1.read_multiturn_position()
        # joint_m_pos = 0
        read_s_pos_joint.append(joint_s_pos)
        # read_m_pos_joint.append(joint_m_pos)
        read_speeds_joint.append(joint_speed)
        read_torque_joint.append(joint_torque)

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
            break

    joint1.motor_stop()    

    print('Test Done')
    print('Error count: ', err_count)

    cut_amount = 10
    del read_s_pos_joint[1:cut_amount]
    del read_m_pos_joint[1:cut_amount]
    del read_speeds_joint[1:cut_amount]
    del read_torque_joint[1:cut_amount]


    def smooth(y, box_pts):
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y,box,mode='same')
        return y_smooth

    traj_expected = []

    for i in range(len(t_vec)) :
        traj_expected.append(trajectory(t_vec[i]))


    def multiturn(singleturn_values):
        multiturn_values = singleturn_values.copy()
        count = 0
        for i in range(len(multiturn_values)-2):
            diff = singleturn_values[i+1] - singleturn_values[i]
            if diff < -4:
                count = count + 1
            elif diff > 4:
                count = count - 1
            multiturn_values[i+1] = multiturn_values[i+1] + 2*count
            multiturn_values[i+2] = multiturn_values[i+2] + 2*count
        return multiturn_values

    read_m_pos_joint = smooth(multiturn(read_s_pos_joint),10)  #need to smooth data becuase there small errors right when the single turn switchess

    fig, axs = plt.subplots(5)
    axs[0].plot(read_s_pos_joint,'b-')
    axs[0].set_title('Position (Singleloop)')
    axs[1].plot(read_m_pos_joint,'m-')
    axs[1].set_title('Position (Multiloop)')
    axs[2].plot(smooth(read_speeds_joint,20),'r-')
    axs[2].set_title('Speed')
    axs[3].plot(smooth(read_torque_joint,300),'g-')
    axs[3].set_title('Torque')
    axs[4].plot(loop_vec)
    axs[4].plot(loop_rate*np.ones(len(loop_vec)))
    axs[4].set_title('Execute rate (Hz)')
    plt.show()

    plt.plot(read_m_pos_joint, "r*")
    plt.plot(traj_expected, 'b')
    plt.show()

    core.CANHelper.cleanup("can0")

