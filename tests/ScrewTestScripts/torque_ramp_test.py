import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor
import time
import numpy as np
from matplotlib import pyplot as plt

from os.path import dirname, realpath  
import sys
from core.CanMotor import CanMotor
import csv
arcsnake_v2_path = dirname(dirname(realpath(__file__)))  
sys.path.append(arcsnake_v2_path)


def get_time(t0):
    return time.time() - t0


if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    screwMotor = CanUJoint(can0, 1, 6, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    encoderMotor = CanUJoint(can0, 0, 1)
    run_time = 6 # in second
    sampling_rate = 200 # in Hz

    # data_fname = '/screw_test_data_files/peak_force_tests/set1/test1.csv'
    data_fname = 'tests/ScrewTestScripts/data_files/torque_ramp_tests/set2/test3.csv'
    initial_torque = -0.01
    final_torque = -1.5
    torque_step = -0.005
    time_step = 0.05
    num_steps = int((final_torque - initial_torque) / torque_step) + 1

    try:
        t0 = time.time()
        with open(data_fname, mode='w') as test_data:
            test_writer = csv.writer(test_data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            test_writer.writerow(['time', 'angular speed', 'torque', 'linear speed', 'angular position', 'interval'])

            time.sleep(3)
            # synchronization procedure
            

            screwMotor.pos_ctrl(0, 6.0)
            row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed(), screwMotor.read_multiturn_position(), -1]
            print(row)
            test_writer.writerow(row)
            time.sleep(2)

            screwMotor.pos_ctrl(1.6, 6.0)
            row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed(), screwMotor.read_multiturn_position(), -2]
            print(row)
            test_writer.writerow(row)
            time.sleep(2)

            screwMotor.pos_ctrl(0, 6.0)
            row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed(), screwMotor.read_multiturn_position(), -3]
            print(row)
            test_writer.writerow(row)
            # screwMotor.motor_stop()

            time.sleep(15)

            # Start main trial loop
            t1 = time.time()
            for i in range(num_steps):

                screwMotor.torque_ctrl(initial_torque + i*torque_step)

                while True:
                    trial_time = get_time(t1)
                    row = [get_time(t0), screwMotor.read_speed(), screwMotor.read_torque(), encoderMotor.read_speed(), screwMotor.read_multiturn_position(), i+1]
                    print(row)
                    test_writer.writerow(row)
                    time.sleep(1/sampling_rate)
                    if trial_time > time_step*(i+1):
                        break



            
    except(KeyboardInterrupt) as e: 
        print(e)

    screwMotor.motor_stop() 
    encoderMotor.motor_stop()

    print('Done')

    core.CANHelper.cleanup("can0")

    ### plot motor data
    with open(data_fname) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        csv_list = list(csv_reader)
        data = np.empty((len(csv_list)-1, 6))
        line_count = 0
        for row in csv_list:
            if line_count != 0:
                data[line_count-1, :] = row
            line_count += 1
    
    plt.plot(data[:,0], data[:,1])
    plt.plot(data[:,0], data[:,2])
    plt.plot(data[:,0], data[:,4])
    plt.legend(['ang vel', 'torque', 'ang pos'])
    plt.show()