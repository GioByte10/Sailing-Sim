import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor
import time
import numpy as np
from core.timeout import TimeoutError


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
    gear_ratio = 1
    motor_id = 4

    while True:
        try:
            print("Trying to initialize motors")
            screwMotor = CanMotor(can0, motor_id, 1)
            # joint2 = CanMotor(can0, 4, gear_ratio)
            # joint1 = CanMotor(can0, 1, gear_ratio)
            # joint3 = CanMotor(can0, 3, gear_ratio)
            # joint4 = CanMotor(can0, 6, gear_ratio)
            # joint1_pos = joint1.read_multiturn_position()
            # joint2_pos = joint2.read_multiturn_position()
            # joint3_pos = joint3.read_multiturn_position()
            # joint4_pos = joint4.read_multiturn_position()
            # joint1.pos_ctrl(joint1_pos) # set read pos
            # joint2.pos_ctrl(joint2_pos) # set read pos
            # joint3.pos_ctrl(joint3_pos) # set read pos
            # joint4.pos_ctrl(joint4_pos) # set read pos
            break
        except TimeoutError:
            print('Timeout Error')
            continue
        # screwMotor = CanUJoint(can0, 5, 1, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    # encoderMotor = CanUJoint(can0, 2, 1)
    input("Motors initialized. Enter to continue")


    sampling_rate = 200 # in Hz

    ### Change these as needed
    
    run_time = 10 # in second
    set_num = 4
    test_num = 7
    command_speed = 10 # in radians per second
    command_torque = 10
    Kp = 255
    Ki = 30
    TC = 2000
    test_name = f'assembled_screwblock{motor_id}_v{command_speed}_Kp{Kp}_Ki{Ki}_fullshellOLD_BESTCASE_errorflagging'
    data_fname = f'tests/System Tests/ScrewShellSpinTests/{test_name}'

    time_data   = []
    torque_data = []
    voltage_data = []

    angular_speed_data = []
    linear_speed_data = []

    while True:
        try:
            print(screwMotor.read_motor_pid())
            screwMotor.override_PI_values(100, 100, Kp, Ki, 50, 50)
            print(screwMotor.read_motor_pid())
            break
        except TimeoutError:
            print('Timeout Error')
            continue
    # screwMotor.override_PI_values(100, 100, Kp, Ki, 50, 50)
    # print(screwMotor.read_motor_pid())
    input("Enter to start spinning")
    time.sleep(5)
    
    try:
        # screwMotor.torque_ctrl(command_torque)
        screwMotor.speed_ctrl(command_speed)
        t0 = time.time()
        with open(f"{data_fname}.csv", mode='w') as test_data:
            test_writer = csv.writer(test_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            test_writer.writerow(['time', 'angular speed', 'torque', 'temp', 'voltage', 'error_byte', 'error_flag'])

            # # synchronization procedure
            # print('Sensor should be in free hang. UNBIAS SENSOR')
            # print("Smooth the surface!")
            # input('Press enter to set zero position')
            # screwMotor.pos_ctrl(0, 8)
            # print("Set sensor back down.")
            # print("Wait a few seconds, then Bias the sensor")
            # input("Press enter to continue")

            t1 = time.time()
            # while True:
            #     try:
            #         screwMotor.speed_ctrl(command_speed)
            #         break
            #     except(TimeoutError):
            #         print('Timeout Error')
            #         continue
            while True:
                while True:
                    try:
                        torque, speed, _ = screwMotor.read_motor_status()
                        temp, voltage, err_byte, error_flag = screwMotor.read_motor_err_and_voltage()
                        row = [get_time(t0), speed, torque, temp, voltage, err_byte, error_flag]
                        break
                    except TimeoutError:
                        print('Timeout Error')
                        continue

                screwMotor.speed_ctrl(command_speed)
                
                print(row)
                test_writer.writerow(row)

                time_data.append(row[0])
                angular_speed_data.append(row[1])
                torque_data.append(row[2])
                # linear_speed_data.append(row[3])
                voltage_data.append(row[4])


                time.sleep(1/sampling_rate)
                if get_time(t1) > run_time:
                    break
    except(KeyboardInterrupt) as e:
        print(e)

    while True:
        try:
            screwMotor.motor_stop()
            # joint1.motor_stop()
            # joint2.motor_stop()
            # joint3.motor_stop()
            # joint4.motor_stop()
            break
        except TimeoutError:
            print('Timeout Error')
            continue
    # encoderMotor.motor_stop()

    print('Done, stop sensor log')

    core.CANHelper.cleanup("can0")


    plt.figure()
    plt.plot(time_data, torque_data)
    plt.plot(time_data, angular_speed_data)
    plt.plot(time_data, voltage_data)
    plt.title(f"Kp = {Kp}, Ki = {Ki},Set Vel = {command_speed}. Mean Torque = {np.mean(torque_data)}")
    plt.legend(['Torque', 'Angular Speed', 'Voltage'])
    # plt.ylim([0, 20])
    # plt.yticks(np.linspace(0, 20, 11))
    plt.savefig(f"{data_fname}.png")
    plt.show()