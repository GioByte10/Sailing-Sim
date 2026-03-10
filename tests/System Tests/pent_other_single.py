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


def restart_motors():
    for motor in motors:
        motor.write_acceleration(0x00, np.uint32(60000))
        motor.write_acceleration(0x01, np.uint32(60000))

        motor.write_acceleration(0x02, np.uint32(60000))
        motor.write_acceleration(0x03, np.uint32(60000))

        motor.initialize_motor()
        motor.initialize_control_command()



if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    # motor = CanMotor(can0, motor_id=7, gear_ratio=1) #m_A
    m_broken = CanMotor(can0, MAX_SPEED=300, motor_id=0, gear_ratio=1, name="Broken")
    motors = [m_broken]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    # m_broken.write_pid(0x01, np.float32(0))
    # m_broken.write_pid(0x02, np.float32(0))
    #
    # m_broken.write_pid(0x04, np.float32(0))
    # m_broken.write_pid(0x05, np.float32(0))
    #
    # m_broken.write_pid(0x07, np.float32(1)) # KP
    # m_broken.write_pid(0x08, np.float32(10))
    # m_broken.write_pid(0x09, np.float32(100))
    #
    msg_data = [0x31, 0x01, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x31, 0x02, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x31, 0x04, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x31, 0x05, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x32, 0x07, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x32, 0x08, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    msg_data = [0x32, 0x09, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
    m_broken.raw_send(msg_data)
    time.sleep(0.02)

    m_broken.write_control_command(0x02, np.uint32(1))

    restart_motors()

    time.sleep(1)
    input("Continue")


    t = 0

    try:
        while True:
            for motor in motors:
                motor.read_status_once()
                time.sleep(0.02)

                motor.read_multiturn_once()
                time.sleep(0.02)

                motor.read_motor_state_once()
                time.sleep(0.02)

            for motor in motors:
                time.sleep(0.02)
                motor.datadump()


            m_broken.set_control_mode("speed", 6)
            m_broken.control()
            time.sleep(0.07)

            m_broken.read_pid_once()
            time.sleep(0.02)

            t += 0.3

    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
