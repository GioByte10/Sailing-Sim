import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
import numpy as np
import math
import scipy.io


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=300, motor_id=7, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=300, motor_id=0, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.write_acceleration(0x00, np.uint32(60000))
        motor.write_acceleration(0x01, np.uint32(60000))

        motor.initialize_motor()


    mat_file = scipy.io.loadmat('cycles/cycle_1.mat')
    a_positions = mat_file['qA'][0]
    d_positions = mat_file['qD'][0]
    print(a_positions)
    print(d_positions)


    a_positions = -a_positions
    d_positions = -d_positions
    print(a_positions)
    print(d_positions)

    a_positions = a_positions + 5.68
    d_positions = d_positions + 1.73
    print(a_positions)
    print(d_positions)

    time.sleep(1)
    input("Continue")

    for motor in motors:
        motor.initialize_control_command()
        # motor.set_control_mode("position", 3)
        # motor.control()

    step = 0
    m_A.set_control_mode("position", a_positions[step])
    m_D.set_control_mode("position", d_positions[step])

    m_A.control()
    m_D.control()

    time.sleep(1)
    try:
        while True:
            # for motor in motors:
            #     motor.read_status_once()
            #     time.sleep(0.02)
            #     motor.read_multiturn_once()
            #     time.sleep(0.02)
            #     motor.read_motor_state_once()
            #     time.sleep(0.02)

            # for motor in motors:
            #     motor.datadump()
            #     time.sleep(0.02)

            m_A.set_control_mode("position", a_positions[step])
            m_D.set_control_mode("position", d_positions[step])
            m_A.control()
            m_D.control()
            time.sleep(0.005)

            step += 1


    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
