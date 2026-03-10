import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor
import time

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

    screwMotor = CanUJoint(can0, 0, 1, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    encoderMotor = CanUJoint(can0, 2, 1)

    sampling_rate = 200 # in Hz

    ### Change these as needed
    run_time = 5 # in second
    set_num = 1
    test_num = 1
    command_speed = -5 # in radians per second
    data_fname = 'tests/test.csv'.format(set_num, test_num)

    time_data   = []
    torque_data = []
    angular_speed_data = []
    linear_speed_data = []

    try:
        t0 = time.time()
        with open(data_fname, mode='w') as test_data:
            test_writer = csv.writer(test_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            test_writer.writerow(['time', 'angular speed', 'torque', 'linear speed'])

            # synchronization procedure
            input('Sensor should be in free hang, unbias sensor then press enter.')
            test_writer.writerow([get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed()])
            input("Lower screw into water, then press enter.")
            test_writer.writerow([get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed()])
            input("Bias sensor, then press enter to start trial.")

            t1 = time.time()
            screwMotor.speed_ctrl(command_speed)
            while True:
                row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed()]
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

    screwMotor.motor_stop()
    encoderMotor.motor_stop()

    print('Done, stop sensor log')

    core.CANHelper.cleanup("can0")


    plt.figure()
    plt.plot(time_data, torque_data)
    plt.title("Torque")

    plt.figure()
    plt.plot(time_data, angular_speed_data) 
    plt.title("Angular Speed")

    plt.figure()
    plt.plot(time_data, linear_speed_data) 
    plt.title("Linear Speed")

    plt.show()