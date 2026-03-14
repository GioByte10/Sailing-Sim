import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import numpy as np
import can
import time

if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    chunky = CanMotor(can0, motor_id=0, gear_ratio=6)

    motors = [chunky]  # Fix: add m_A to list
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    chunky.read_status_once()
    chunky.read_multiturn_once()
    chunky.read_motor_state_once()
    chunky.datadump()

    step = float(sys.argv[1])
    chunky.set_control_mode("speed", step)
    chunky.control()

    t = 0

    try:
        while True:
            chunky.read_status_once()
            time.sleep(0.02)
            chunky.read_multiturn_once()
            time.sleep(0.02)
            chunky.read_motor_state_once()
            time.sleep(0.02)
            chunky.datadump()
            time.sleep(0.02)

            chunky.set_control_mode("speed", step)
            chunky.control()
            time.sleep(0.02)

            t += 0.005
            pass

    except KeyboardInterrupt:
        for motor in motors:
            motor.set_control_mode("torque", 0)
            motor.control()

        time.sleep(3)

        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
