import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor
import time
import numpy as np
from core.timeout import TimeoutError

import os
from os.path import dirname, realpath
import sys
from core.CanMotor import CanMotor
import csv

import matplotlib.pyplot as plt

arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)  


def get_time(t0):
    return time.time() - t0

if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')
    while True:
        try:
            print("Trying to initialize motors")
            screwMotor = CanMotor(can0, motor_id=5, gear_ratio=1)
            break
        except TimeoutError:
            print('Timeout Error')
            continue
        # screwMotor = CanUJoint(can0, 5, 1, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    # encoderMotor = CanUJoint(can0, 2, 1)

    sampling_rate = 200 # in Hz
    ### Change these as needed
    run_time = 20 # in second
    set_num = 4
    test_num = 7
    command_speed = 10 # in radians per second
    Kp = 40
    Ki = 30
    test_name = f'speed_rampup_Kp{Kp}_Ki{Ki}'
    data_folder = "/home/myeoh/Documents/3-26-24_motor_debugging/motor4_newscrewdrivetrain"
    os.makedirs(data_folder, exist_ok=True)

    time_data   = []
    torque_data = []
    angular_speed_data = []
    linear_speed_data = []

    # while True:
    #     try:
    #         print(screwMotor.read_motor_pid())
    #         screwMotor.override_PI_values(100, 100, Kp, Ki, 50, 50)
    #         print(screwMotor.read_motor_pid())
    #         break
    #     except TimeoutError:
    #         print('Timeout Error')
    #         continue
    print(screwMotor.read_motor_pid())
    input("Enter to start")
    # screwMotor.speed_ctrl(command_speed)
    try:
        
        t0 = time.time()
        with open(os.path.join(data_folder, test_name + "_csv.csv"), mode='w') as test_data:
            test_writer = csv.writer(test_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            test_writer.writerow(['time', 'angular speed', 'torque', 'linear speed'])

            t1 = time.time()
            while True:
                cur_t = get_time(t1)
                step = np.floor(cur_t/2)+1
                screwMotor.speed_ctrl(command_speed*step)
            #     while True:
            #         try:
                row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), 0]
                    #     break
                    # except TimeoutError:
                    #     print('Timeout Error')
                    #     continue
                print(row)
                test_writer.writerow(row)

                time_data.append(row[0])
                angular_speed_data.append(row[1])
                torque_data.append(row[2])
                linear_speed_data.append(row[3])


                time.sleep(1/sampling_rate)
                if get_time(t1) > run_time:
                    break
    except(KeyboardInterrupt) as e:
        print(e)

    # while True:
    #     try:
    screwMotor.motor_stop()
        #     break
        # except TimeoutError:
        #     print('Timeout Error')
        #     continue
    # encoderMotor.motor_stop()

    print('Done, stop sensor log')

    core.CANHelper.cleanup("can0")


    plt.figure()
    plt.plot(time_data, torque_data)
    plt.plot(time_data, angular_speed_data)
    plt.title(f"{test_name}\nAvg Torque: {np.mean(torque_data)}, Avg Speed: {np.mean(angular_speed_data)}")
    plt.legend(['Torque', 'Angular Speed'])
    # plt.ylim([0, 20])
    # plt.yticks(np.linspace(0, 20, 11))
    plt.savefig(os.path.join(data_folder, test_name + "_plot.png"))
    plt.show()