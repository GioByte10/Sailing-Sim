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


if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # screwMotor = CanUJoint(can0, 1, 6, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    encoderMotor = CanUJoint(can0, 2, 1)
    sampling_rate = 200 # in Hz
    run_time = 10 # in second
    data_fname = 'tests/ScrewTestScripts/data_files/axial_load_tests/test1.csv'
    command_speed = 0.5 # in radians per second
    command_torque = -6
    
    time_data   = []
    torque_data = []
    angular_speed_data = []

    try:
        time.sleep(10)
        with open(data_fname, mode='w') as test_data:
            test_writer = csv.writer(test_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            test_writer.writerow(['time', 'torque', 'angular speed'])
            encoderMotor.torque_ctrl(command_torque)
            start_time = time.time()
            while True:
                cur_time = time.time() - start_time
                row = [cur_time, encoderMotor.read_torque(), encoderMotor.read_speed()]
                print(row)
                test_writer.writerow(row)

                time_data.append(row[0])
                torque_data.append(row[1])
                angular_speed_data.append(row[2])

                time.sleep(1/sampling_rate)
                if cur_time > run_time:
                    break

        
    except(KeyboardInterrupt) as e:
        print(e)

    # screwMotor.motor_stop() 
    encoderMotor.motor_stop()

    print('Done')

    core.CANHelper.cleanup("can0")

    plt.figure()
    plt.plot(time_data, torque_data)

    plt.figure()
    plt.plot(time_data, angular_speed_data) 

    plt.show()