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

    motor_id = 7
    config = "load_fullsegment"
    amp_multiplier = 0.3

    folder = f"tests/System Tests/JointRepeatabilityTests/joint{motor_id}"
    os.makedirs(folder, exist_ok=True)
    test_name = f"{config}_amp{amp_multiplier}pi_torquemaxed"

    print("Trying to initialize motors")
    # screw1 = CanMotor(can0, 0, 1)
    joint1 = CanMotor(can0, motor_id, gear_ratio)
    joint2 = CanMotor(can0, 9, gear_ratio)
    print('Motor initialization complete')

    input("Press Enter to read zero position.")
    joint1_zero_pos = joint1.read_multiturn_position()
    joint2_zero_pos = joint2.read_multiturn_position()

    amp = amp_multiplier * math.pi
    num_periods = 3
    t_period = 16 # s
    loop_rate = 200 # Hz
    t_loop = (1/loop_rate) # expected time between loops
    num_samples = int(loop_rate * t_period) # per period
    
    log_data = [["time", "s_pos", "m_pos", "speed", "torque"]]
    input("Enter to set to zero pos")
    time.sleep(5)
    print("Commanding to zero position...")
    joint1.pos_ctrl(joint1_zero_pos, 1)
    joint2.pos_ctrl(joint2_zero_pos, 1)

    input("Press Enter to start test.")
    time.sleep(5)
    print("Starting Test.")
    err_count = 0
    t0 = time.time()
    for i in range(num_periods * num_samples + 1):

        cur_t = time.time() - t0


        joint1.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint1_zero_pos)
        (cur_torque, cur_speed, cur_s_pos) = joint1.read_motor_status()
        # joint2.pos_ctrl(amp*math.sin(2*math.pi*i/500 + math.pi/2) + joint2_zero_pos)
        cur_m_pos = joint1.read_DIY_multiturn_position()
        log_data.append([cur_t, cur_s_pos, cur_m_pos, cur_speed, cur_torque])
        print(f"Period {int(i/num_samples)}, Iteration {i}\n-------------")
        print(f"Time = {cur_t}, s_pos = {cur_s_pos}, m_pos = {cur_m_pos}, speed = {cur_speed}, torque = {cur_torque}\n")

        # Ensures proper loop rate
        t_execute = time.time() - cur_t - t0 # Time to execute code inside loop
        if t_execute >= t_loop: # If loop takes longer to execute than expected loop time, loop rate is too fast
            err_count = err_count + 1
            print('Loop rate too fast.')
        elif t_execute < t_loop: # Otherwise wait difference between executed time and expected loop time
            time.sleep(t_loop - t_execute)

    joint1.motor_stop()    
    joint2.motor_stop()    

    joint1.motor_off()
    joint2.motor_off()

    print('Test Done')
    print("Loop rate error count = ", err_count)

    # Save data to csv
    with open(os.path.join(folder, f"{test_name}.csv"), 'w') as f:
        writer = csv.writer(f)
        writer.writerows(log_data)

    # Plot sine waves on top of eachother
    log_data = np.array(log_data[1:])
    step = log_data[:, 0]
    s_pos = log_data[:,1]
    m_pos = log_data[:,2]
    speed = log_data[:,3]
    torque = log_data[:,4]
    # fig, axs = plt.subplots(4, 1)
    fig1 = plt.figure()
    fig2 = plt.figure()
    fig3 = plt.figure()
    # fig.suptitle("Position")
    for i in range(num_periods):
        
        start_idx = i*num_samples
        end_idx = (i+1)*num_samples + 1

        # axs[0].set_title("Singlturn Position")
        # axs[0].plot(s_pos[start_idx:end_idx], label=f"Period {i+1}")

        fig1.suptitle("Mutliturn Position")
        fig1.gca().plot(m_pos[start_idx:end_idx], label=f"Period {i+1}")

        fig2.suptitle("Speed")
        fig2.gca().plot(speed[start_idx:end_idx], label=f"Period {i+1}")

        fig3.suptitle("Torque")
        fig3.gca().plot(torque[start_idx:end_idx], label=f"Period {i+1}")

    fig1.legend()
    fig2.legend()
    fig3.legend()

    plt.tight_layout()

    fig1.savefig(os.path.join(folder, f"{test_name}_pos.png"))
    fig2.savefig(os.path.join(folder, f"{test_name}_speed.png"))
    fig3.savefig(os.path.join(folder, f"{test_name}_torque.png"))
    plt.show()

    core.CANHelper.cleanup("can0")

