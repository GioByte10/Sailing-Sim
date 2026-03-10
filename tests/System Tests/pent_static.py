import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time

if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_D = CanMotor(can0, motor_id=0, gear_ratio=1)

    motors = [m_D]  # Fix: add m_A to list
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    m_D.read_status_once()
    m_D.read_multiturn_once()
    m_D.read_motor_state_once()
    m_D.datadump()

    step = float(sys.argv[1])
    m_D.set_control_mode("torque", -step)

    try:
        while True:
            m_D.control()
            time.sleep(0.02)
            m_D.read_status_once()
            time.sleep(0.02)
            m_D.read_multiturn_once()
            time.sleep(0.02)
            m_D.read_motor_state_once()
            time.sleep(0.02)
            m_D.datadump()

            time.sleep(0.07)
            pass

    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
