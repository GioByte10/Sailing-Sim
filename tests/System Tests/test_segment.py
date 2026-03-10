### Test script for validating single segment (screw motor + 2 joints)
from asyncore import loop
from audioop import mul
from cProfile import run
from doctest import run_docstring_examples
from os.path import dirname, realpath
from pdb import post_mortem
import sys
from unittest import mock
import csv

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
from core.timeout import TimeoutError
import math


if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    gear_ratio = 11

    # folder = "tests/System Tests/PCBTesting"
    # test_name = "4screws_2joints_test"

    print("Trying to initialize motors")
    screw1 = CanMotor(can0, 4, 1)
    joint1 = CanMotor(can0, 6, gear_ratio)
    joint2 = CanMotor(can0, 9, gear_ratio)

    print('Motor initialization complete')

    input('Press Enter to spin screw motor')
    command_speed = 10
    screw1.speed_ctrl(command_speed)

    # input('Press Enter to read joint current pos')
    # joint1_zero_pos = joint1.read_multiturn_position()
    # joint2_zero_pos = joint2.read_multiturn_position()
    # print('Joint 1 pos: ', joint1_zero_pos)
    # print('Joint 2 pos: ', joint2_zero_pos)
    # input('Press Enter to set joint current pos')
    # joint1.pos_ctrl(joint1_zero_pos, 2) # set read pos
    # joint2.pos_ctrl(joint2_zero_pos, 2) # set read pos

    # zero_pos = joint1_m_pos
    amp = .1 * m.pi
    i=0
    # joint2_start_pos = amp*math.sin(2*math.pi*i/500 + math.pi/2) + joint2_zero_pos

    # input("Press Enter to set joint 2 to start pos")
    # joint2.pos_ctrl(joint2_start_pos, 0.1)

    # input("Press Enter to start dual sine wave trajectory.")
    # time.sleep(3)
    # command_speed = 10
    # screw1.speed_ctrl(command_speed)
    # joint1.pos_ctrl(trajectory_1(0),2)
    # joint2.pos_ctrl(trajectory_2(0),2)
    # time.sleep(2)

    # t0 = time.time()
    # for i in range(1000):

    #     joint1.pos_ctrl(amp*math.sin(2*math.pi*i/500) + joint1_zero_pos)
    #     joint2.pos_ctrl(amp*math.sin(2*math.pi*i/500 + math.pi/2) + joint2_zero_pos)
    #     print(time.time() - t0, joint1.read_motor_status())


    #     time.sleep(0.02)

    # joint1.motor_stop()    
    # joint2.motor_stop()    

    # print('Test Done')

    input('Press Enter to stop motors')
    screw1.motor_off()
    # joint1.motor_off()
    # joint2.motor_off()
    # # joint3.motor_off()
    # # joint4.motor_off()
    # screw2.motor_off()
    # screw3.motor_off()
    # screw4.motor_off()
    # screw5.motor_off()

    print('Done')

    # plt.plot(motor_voltages)
    # plt.legend(["joint 3", "joint 5", "screw 9", "screw8", "screw6"]) #, "screw 3", "joint 1", "joint 2", "joint 3", "joint 4"])
    # plt.title(test_name)
    # plt.xlabel("time")
    # plt.ylabel("Motor Voltage")
    # plt.savefig(f"{folder}/{test_name}.png")
    # plt.show()

    core.CANHelper.cleanup("can0")

