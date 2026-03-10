from asyncore import loop
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


# This code is meant to test the latency of certain commands or chains of commands and spit out a graph of the latency at the end

if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
    gear_ratio = 6
    joint1 = CanMotor(can0, 1, gear_ratio) #ID: 2
    joint1.motor_start()

    time_zero = time.time()
    t_start = 0
    err_count = 0
    t_vec1 = []
    t_vec2 = []
    pos_vec1 = []
    pos_vec2 = []
    

    for i in range(1000):
        t_start = time.time() - time_zero


        #### COMANDS HERE ####

        # joint1.pos_ctrl(0)
        time.sleep(.001)
        # (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
        joint_m_pos = joint1.read_multiturn_position()


        ##################


        t_execute = time.time() - time_zero - t_start
        t_vec1.append(1/t_execute)
        pos_vec1.append(joint_m_pos)

        try:
            time.sleep(.0000000001)
        except(KeyboardInterrupt) as e:
            print(e)
            joint1.motor_stop()    
            break


    for i in range(1000):
        t_start = time.time() - time_zero


        #### COMANDS HERE ####

        # joint1.pos_ctrl(0)
        time.sleep(.001)
        # (joint_torque, joint_speed, joint_s_pos) = joint1.read_motor_status()
        joint_m_pos = joint1.read_multiturn_position()


        ##################

        t_execute = time.time() - time_zero - t_start
        t_vec2.append(1/t_execute)
        pos_vec2.append(joint_m_pos)

        try:
            time.sleep(.0000000001)
        except(KeyboardInterrupt) as e:
            print(e)
            joint1.motor_stop()    
            break

    joint1.motor_stop()    

    avg1 = np.ones(len(t_vec1)) * np.mean(t_vec1)
    avg2 = np.ones(len(t_vec2)) * np.mean(t_vec2)

    print('Average rate for loop 1 (blue): ', np.mean(t_vec1), ' Hz')
    print('Average rate for loop 2 (red): ', np.mean(t_vec2), ' Hz')

    fig, axs = plt.subplots(2)
    axs[0].plot(t_vec1,'b.')
    axs[0].plot(avg1, 'b')
    axs[0].plot(t_vec2,'r.')
    axs[0].plot(avg2,'r')
    axs[0].set_title('Loop rate (Hz)')

    axs[1].plot(pos_vec1, 'b')
    axs[1].plot(pos_vec2,'r')
    axs[1].set_title('Position')
    plt.show()

    core.CANHelper.cleanup("can0")

