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
from core.CanArduinoSensors import CanArduinoSensors


def printAllImuData(sensor):
  print("\n--------------------------------------\n")
  print("Calibration and Temp Data: ", sensor.readImuCalibrationAndTemp())
  print("Orientation Euler Angles: ", sensor.readImuOrientation())
  print("Orientation Quaternion: ", sensor.readImuQuaternion())
  print("Gyroscope: ", sensor.readImuGyroscope())
  print("Magnetometer: ", sensor.readImuMagnetometer())
  print("Accelerometer: ", sensor.readImuAccelerometer())
  print("Linear Acceleration: ", sensor.readImuLinearAccel())
  print("Gravity Vector: ", sensor.readImuGravity())
  print("\n--------------------------------------\n")


if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    gear_ratio = 11

    folder = "tests/System Tests/PCBTesting"
    test_name = "assembled_screwblocks_014"

    print("Trying to initialize motors")
    joint1 = CanMotor(can0, 8, gear_ratio)
    joint5 = CanMotor(can0, 10, gear_ratio)
    joint4 = CanMotor(can0, 5, gear_ratio)
    joint3 = CanMotor(can0, 6, gear_ratio)
    joint2 = CanMotor(can0, 9, gear_ratio)
    joint6 = CanMotor(can0, 7, gear_ratio)
    screw1 = CanMotor(can0, 0, 1) # negative
    screw2 = CanMotor(can0, 1, 1) # negative
    screw3 = CanMotor(can0, 4, 1) # positive
    screw4 = CanMotor(can0, 3, 1) # negative
    # screw5 = CanMotor(can0, 6, 1)

    print('Motor initialization complete')

    print("Initialize and read IMU Sensor.")
    sensor_tail = CanArduinoSensors(can0, 0x01)
    sensor_head = CanArduinoSensors(can0, 0x02)
    printAllImuData(sensor_tail)
    printAllImuData(sensor_head)
    
    input('Press Enter to read joint current pos')
    joint1_pos = joint1.read_multiturn_position()
    joint2_pos = joint2.read_multiturn_position()
    joint3_pos = joint3.read_multiturn_position()
    joint4_pos = joint4.read_multiturn_position()
    joint5_pos = joint5.read_multiturn_position()
    joint6_pos = joint6.read_multiturn_position()

    print('Joint 1 pos: ', joint1_pos)
    print('Joint 2 pos: ', joint2_pos)
    print('Joint 3 pos: ', joint3_pos)
    print('Joint 4 pos: ', joint4_pos)
    print('Joint 5 pos: ', joint5_pos)
    print('Joint 6 pos: ', joint6_pos)


    # input('Press Enter to set joint current pos')
    # joint1.pos_ctrl(joint1_pos, 0.5) # set read pos
    # joint2.pos_ctrl(joint2_pos, 0.5) # set read pos
    # joint3.pos_ctrl(joint3_pos, 0.5) # set read pos
    # joint4.pos_ctrl(joint4_pos, 0.5) # set read pos
    # joint5.pos_ctrl(joint5_pos, 0.5) # set read pos
    # joint6.pos_ctrl(joint6_pos, 0.5) # set read pos

    while True:
        try:
            # print(screw1.read_motor_pid())
            print(screw1.read_motor_pid())
            print(screw2.read_motor_pid())
            print(screw3.read_motor_pid())
            print(screw4.read_motor_pid())
            Kp = 255
            Ki = 50
            screw1.override_PI_values(100, 100, Kp, Ki, 50, 50)
            screw2.override_PI_values(100, 100, Kp, Ki, 50, 50)
            screw3.override_PI_values(100, 100, Kp, Ki, 50, 50)
            screw4.override_PI_values(100, 100, Kp, Ki, 50, 50)
            print(screw1.read_motor_pid())
            print(screw2.read_motor_pid())
            print(screw3.read_motor_pid())
            print(screw4.read_motor_pid())
            break
        except TimeoutError:
            print('Timeout Error')
            continue
    # while True:
    #     try:
    #         print(screw1.read_motor_pid())
    #         Kp = 255
    #         Ki = 50
    #         screw1.override_PI_values(100, 100, Kp, Ki, 50, 50)
    #         print(screw1.read_motor_pid())
    #         break
    #     except TimeoutError:
    #         print('Timeout Error')
    #         continue
    # screwMotor.override_PI_values(100, 100, Kp, Ki, 50, 50)
    # print(screw1.read_motor_pid())
    # print(screw2.read_motor_pid())
    # print(screw3.read_motor_pid())
    # print(screw4.read_motor_pid())
    # print(screw5.read_motor_pid())

    input('Press Enter to spin screw motors')

    # screw2_pos = screw2.read_multiturn_position()
    # screw2.pos_ctrl(screw2_pos) # set read pos
    # print(screw2.read_multiturn_position())
    # time.sleep(0.1)
    # screw2.pos_ctrl(screw2_pos + 0.1) # set read pos
    # print(screw2.read_multiturn_position())
    # screw2.pos_ctrl(screw2_pos + 0.2) # set read pos
    # print(screw2.read_multiturn_position())

    # screw2.speed_ctrl(5) 
    # screw2.speed_ctrl(5) 
    


    # Roll
    command_speed = 10
    screw1.speed_ctrl(-command_speed)
    screw2.speed_ctrl(-command_speed)
    screw3.speed_ctrl(command_speed)
    screw4.speed_ctrl(-command_speed)
    
    # time.sleep(0.1)
    # print(screw1.speed_ctrl(5))
    # screw2.speed_ctrl(-10)
    # time.sleep(0.1)
    # print(screw2.speed_ctrl(-10))
    # screw3.speed_ctrl(5)

    # # # Torpedo
    # factor = 50
    
    # motor_voltages = []
    # number_of_steps = 1000
    # with open(f'{folder}/{test_name}.csv', 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile, delimiter=',')

    #     for i in range(number_of_steps):
    #         voltages = []
    #         # screw1.speed_ctrl((factor*1)*i/number_of_steps)
    #         # screw1.clear_error_flag()
    #         screw2.speed_ctrl((factor*1)*i/number_of_steps)
    #         # screw2.clear_error_flag()
    #         screw3.speed_ctrl((factor*1)*i/number_of_steps)
    #         # screw3.clear_error_flag()
    #         screw4.speed_ctrl((factor*1)*i/number_of_steps)
    #         # screw4.clear_error_flag()
    #         # screw5.speed_ctrl((factor*1)*i/number_of_steps)
    #         # screw5.clear_error_flag()

    #         # joint1.pos_ctrl(joint1_pos) # set read pos
    #         # joint2.pos_ctrl(joint2_pos) # set read pos
    #         # joint3.pos_ctrl(joint3_pos) # set read pos
    #         # joint4.pos_ctrl(joint4_pos) # set read pos

    #         # screw1_error = screw1.read_motor_err_and_voltage()
    #         screw2_error = screw2.read_motor_err_and_voltage()
    #         screw3_error = screw3.read_motor_err_and_voltage()
    #         screw4_error = screw4.read_motor_err_and_voltage()
    #         # screw5_error = screw5.read_motor_err_and_voltage()

    #         # joint1_error = joint1.read_motor_err_and_voltage()
    #         # joint2_error = joint2.read_motor_err_and_voltage()
    #         # joint3_error = joint3.read_motor_err_and_voltage()
    #         # joint4_error = joint4.read_motor_err_and_voltage()

    #         # voltages.append(screw1_error[1])
    #         voltages.append(screw2_error[1])
    #         voltages.append(screw3_error[1])
    #         voltages.append(screw4_error[1])
    #         # voltages.append(screw5_error[1])

    #         motor_voltages.append(voltages)

    #         # motor_voltages.append([screw1_error[1],
    #         #                     screw2_error[1]]) #, 
    #                             # screw3_error[1], 
    #                             # joint1_error[1],
    #                             # joint2_error[1],
    #                             # joint3_error[1],
    #                             # joint4_error[1]])

    #         writer.writerow(voltages)
            
    #         # writer.writerow([screw1_error[1], 
    #         #                     screw2_error[1]]) #, 
    #                             # screw3_error[1], 
    #                             # joint1_error[1],
    #                             # joint2_error[1],
    #                             # joint3_error[1],
    #                             # joint4_error[1]])

    #         # if screw1_error[2] != 0 or screw2_error[2] != 0 or screw3_error[2] != 0 or joint1_error[2] != 0 or joint2_error[2] != 0 or joint3_error[2] != 0 or joint4_error[2] != 0:
    #         print("Iteration: ", i)
    #         # print("Screw 1 error: ", screw1_error)
    #         print("Screw 2 error: ", screw2_error)
    #         print("Screw 3 error: ", screw3_error)
    #         print("Screw 4 error: ", screw4_error)
    #         # print("Screw 5 error: ", screw4_error)
    #         print('')
    #         # print("Joint 1 error: ", joint1_error)
    #         # print("Joint 2 error: ", joint2_error)
    #         # print("Joint 3 error: ", joint3_error)
    #         # print("Joint 4 error: ", joint4_error)
    #             # break


    #         time.sleep(0.01)

    # print(screw1.get_error_flag())
    # print(screw2.get_error_flag())
    # print(screw3.get_error_flag())
    # print(screw4.get_error_flag())
    # print(screw5.get_error_flag())

    # # --- TESTING READ FUNCTIONS ---
    # # print error messages
    # print("Error messages:\n")
    # print("Screw 1:", screw1.read_motor_err_and_voltage(), "\n")
    # print("Screw 2:", screw2.read_motor_err_and_voltage(), "\n")
    # print("Screw 3:", screw3.read_motor_err_and_voltage(), "\n")
    # # print three-phase currents
    # print("Reading phase currents:\n")
    # print("Screw 1:", screw1.read_phase_current_data(), "\n")
    # print("Screw 2:", screw2.read_phase_current_data(), "\n")
    # print("Screw 3:", screw3.read_phase_current_data(), "\n")

    # input("Run sine waves on joint motors")

    # amp_multiplier = 0.3
    # amp = amp_multiplier * math.pi
    # num_periods = 2
    # t_period = 10 # s
    # loop_rate = 100 # Hz
    # t_loop = (1/loop_rate) # expected time between loops
    # num_samples = int(loop_rate * t_period) # per period

    # err_count = 0
    # t0 = time.time()
    # for i in range(num_periods * num_samples + 1):

    #     cur_t = time.time() - t0


    #     joint1.pos_ctrl(-1*(amp*math.sin(2*math.pi*i/num_samples)) + joint1_pos)
    #     time.sleep(0.000001)
    #     # joint2.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint2_pos)
    #     # time.sleep(0.000001)
    #     # joint3.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint3_pos)
    #     # time.sleep(0.000001)
    #     # joint4.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint4_pos)
    #     # time.sleep(0.000001)
    #     # joint5.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint5_pos)
    #     # time.sleep(0.000001)
    #     # joint6.pos_ctrl(amp*math.sin(2*math.pi*i/num_samples) + joint6_pos)

    #     # (cur_torque, cur_speed, cur_s_pos) = joint1.read_motor_status()
    #     # joint2.pos_ctrl(amp*math.sin(2*math.pi*i/500 + math.pi/2) + joint2_zero_pos)
    #     # cur_m_pos = joint1.read_DIY_multiturn_position()
    #     # log_data.append([cur_t, cur_s_pos, cur_m_pos, cur_speed, cur_torque])
    #     print(f"Period {int(i/num_samples)}, Iteration {i}\n-------------")
    #     # print(f"Time = {cur_t}, s_pos = {cur_s_pos}, m_pos = {cur_m_pos}, speed = {cur_speed}, torque = {cur_torque}\n")

    #     # Ensures proper loop rate
    #     t_execute = time.time() - cur_t - t0 # Time to execute code inside loop
    #     if t_execute >= t_loop: # If loop takes longer to execute than expected loop time, loop rate is too fast
    #         err_count = err_count + 1
    #         print('Loop rate too fast.')
    #     elif t_execute < t_loop: # Otherwise wait difference between executed time and expected loop time
    #         time.sleep(t_loop - t_execute)

    input('Press Enter to stop motors')
    joint1.motor_off()
    joint2.motor_off()
    joint3.motor_off()
    joint4.motor_off()
    joint5.motor_off()
    joint6.motor_off()
    screw1.motor_off()
    screw2.motor_off()
    screw3.motor_off()
    screw4.motor_off()
    # screw5.motor_off()

    print('Done')

    printAllImuData(sensor_tail)
    printAllImuData(sensor_head)

    # plt.plot(motor_voltages)
    # plt.legend(["joint 3", "joint 5", "screw 9", "screw8", "screw6"]) #, "screw 3", "joint 1", "joint 2", "joint 3", "joint 4"])
    # plt.title(test_name)
    # plt.xlabel("time")
    # plt.ylabel("Motor Voltage")
    # plt.savefig(f"{folder}/{test_name}.png")
    # plt.show()

    core.CANHelper.cleanup("can0")

