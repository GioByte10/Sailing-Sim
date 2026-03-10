import can
import core.CANHelper
from core.CanUJoint import CanUJoint
from core.CanScrewMotor import CanScrewMotor
import time

from core.CanMotor import CanMotor
import csv

import matplotlib.pyplot as plt

def get_time(t0):
    return time.time() - t0

if __name__ == "__main__":
    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    screwMotor = CanUJoint(can0, 1, 6, MIN_POS = 0 * 2 * 3.14, MAX_POS = 10 * 2 * 3.14)
    encoderMotor = CanUJoint(can0, 2, 1)

    print(screwMotor.read_motor_pid())
    

    sampling_rate = 200 # in Hz

    ### Change these as needed
    run_time = 10 # in second
    type_num = 1 # [screw sand  ,wheel sand, wheel concrete]
    set_num = 4 # [prop, middle, short middle, screw]
    dir_num = 1 # [forward]
    test_num = 3 # trial number [1 2 3]
    command_speed = -10 # in radians per second (1:1 gearbox); neg for forw and pos for back
    data_fname = 'tests/ScrewTestScripts/sand_tests/test{0}{1}{2}{3}.csv'.format(type_num,set_num, dir_num, test_num)

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