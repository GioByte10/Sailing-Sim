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

    motor_id = 10
    # config = "load_fullsegment"
    # amp_multiplier = 0.2

    folder = f"tests/System Tests/LinearTrajectoryTests/screw{motor_id}"
    os.makedirs(folder, exist_ok=True)

    # Trajectory params
    angle_to = math.pi / 2 # target angle in radians
    command_speed = 0.2 # radians per second
    num_waypoints = 1600 # num trajectory waypoints
    t_loop = abs(angle_to / command_speed) / num_waypoints

    test_name = f"set_pi2_w{command_speed}_testingreadposfunctions"

    print("Trying to initialize motors")
    joint1 = CanMotor(can0, motor_id, gear_ratio)
    print('Motor initialization complete')

    while True:
        try:
            print(joint1.read_motor_pid())
            joint1.override_PI_values(100, 100, 40, 30, 50, 50)
            print(joint1.read_motor_pid())
            break
        except TimeoutError:
            print('Timeout Error')
            continue

    input("Press Enter to read zero position.")
    joint1_zero_pos = joint1.read_multiturn_position()
    joint1_zero_s_pos = joint1.read_singleturn_position()
    print(f"s_pos = {joint1.read_singleturn_position()}, m_pos = {joint1_zero_pos}, diy_m_pos = {joint1.read_DIY_multiturn_position()}")
    # print(joint1.read_multiturn_position())
    # print(joint1.read_multiturn_position())
    # print(joint1.read_multiturn_position())
    input("Enter to set to zero pos")
    # time.sleep(5)
    print("Commanding to zero position...")
    joint1.pos_ctrl(joint1_zero_pos, 1)
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    # joint1.pos_ctrl(joint1_zero_pos, 1)
    # print("commanding..")
    # time.sleep(1)
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    # print(f"Double check of m_pos after pos_ctrl = {joint1.read_multiturn_position()}")
    input("Press Enter to start test.")
    # time.sleep(5)
    print("Starting Test.")
    log_data = [["time", "s_pos", "m_pos", "diy_m_pos", "raw_encoder", "speed", "torque"]]
    raw_multiturn_byte_list = []
    err_count = 0
    t0 = time.time()
    for i in range(num_waypoints):

        cur_t = time.time() - t0
        

        joint1.pos_ctrl((angle_to * ((i+1)/num_waypoints)) + joint1_zero_pos)
        cur_m_pos = joint1.read_multiturn_position()
        print(f"\n{cur_m_pos}\n")

        # cur_m_pos = joint1.read_multiturn_position()
        # cur_m_pos = joint1.read_multiturn_position()
        # cur_m_pos = joint1.read_multiturn_position()
        # cur_m_pos = joint1.read_multiturn_position()
        (cur_torque, cur_speed, cur_s_pos) = joint1.read_motor_status()
        cur_diy_m_pos = joint1.read_DIY_multiturn_position()
        raw_encoder = joint1.read_raw_position()
        log_data.append([cur_t, cur_s_pos, cur_m_pos, cur_diy_m_pos, raw_encoder, cur_speed, cur_torque])
        # print(f"Iteration {i}\n-------------")
        # print(f"Time = {cur_t}, s_pos = {cur_s_pos}, m_pos = {cur_m_pos}, diy_m_pos = {cur_diy_m_pos}, speed = {cur_speed}, torque = {cur_torque}\n")

        # Ensures proper loop rate
        t_execute = time.time() - cur_t - t0 # Time to execute code inside loop
        if t_execute >= t_loop: # If loop takes longer to execute than expected loop time, loop rate is too fast
            err_count = err_count + 1
            print('Loop rate too fast.')
        elif t_execute < t_loop: # Otherwise wait difference between executed time and expected loop time
            time.sleep(t_loop - t_execute)

    joint1.motor_stop()    

    print('Test Done')
    print("Loop rate error count = ", err_count)

    joint1.pos_ctrl(joint1_zero_pos, 0.5)
    time.sleep(5)
    joint1.save_message_log("/home/myeoh/Documents/GitHub/arcsnake_v2/tests/System Tests/TestCanMessages/linear_traj_test_post_recv_fix.csv")
    joint1.motor_off()


    # Save data to csv
    with open(os.path.join(folder, f"{test_name}.csv"), 'w') as f:
        writer = csv.writer(f)
        writer.writerows(log_data)

    # with open(os.path.join(folder, f"{test_name}_raw_multiturn_bytes.csv"), 'w') as f:
    #     writer = csv.writer(f)
    #     writer.writerows(raw_multiturn_byte_list)


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

    log_data = np.array(log_data[1:])
    step = log_data[:, 0]
    s_pos = log_data[:,1] - joint1_zero_s_pos
    m_pos = log_data[:,2] - joint1_zero_pos
    diy_m_pos = log_data[:,3]
    peter_diy_m_pos = multiturn(s_pos)
    raw_encoder = log_data[:,4]
    speed = log_data[:,5]
    torque = log_data[:,6]

    print(s_pos[-1], diy_m_pos[-1], peter_diy_m_pos[-1])

    plt.figure()
    plt.title("Position")

    plt.plot(step, s_pos, marker='o', label ="Singleturn")
    plt.plot(step, m_pos, label="Multiturn")
    plt.plot(step, diy_m_pos, marker='x', label ="Ming's DIY Multiturn")
    plt.plot(step, peter_diy_m_pos, label ="Peter's DIY Multiturn")
    plt.axhline(angle_to, label = "Set Angle")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(folder, f"{test_name}.png"))
    # plt.show()

    plt.figure()
    plt.title("Builtin Multiturn")
    plt.plot(step, m_pos, label="Multiturn")
    # plt.show()

    plt.figure()
    plt.title("Raw Encoder")
    plt.plot(step, raw_encoder)
    plt.show()
    
    
    # plt.figure()
    # plt.title("Speed")
    # plt.plot(t, speed)
    # plt.tight_layout()
    # plt.show()

    # plt.figure()
    # plt.title("Torque")
    # plt.plot(t, torque)
    # plt.tight_layout()
    # plt.show()

    # fig1.savefig(os.path.join(folder, f"{test_name}_pos.png"))
    # fig2.savefig(os.path.join(folder, f"{test_name}_speed.png"))
    # fig3.savefig(os.path.join(folder, f"{test_name}_torque.png"))
    # plt.show()

    # input("Press Enter to return to zero position")


    core.CANHelper.cleanup("can0")

