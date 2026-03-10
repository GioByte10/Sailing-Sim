import math
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
import scipy.io


def end():
    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_D = CanMotor(can0, motor_id=0, gear_ratio=1)
    m_A = CanMotor(can0, motor_id=7, gear_ratio=1)

    motors = [m_D, m_A]
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

    m_A.read_status_once()
    m_A.read_multiturn_once()
    m_A.read_motor_state_once()
    m_A.datadump()

    offset_D=3.3
    offset_A=.92
    qA_list = [3.32744002066448,
                   3.39068692916100,
                   3.45424782480727,
                   3.51828799630229,
                   3.58396283625519,
                   3.65099743700569,
                   3.71970565714629,
                   3.79045614779722,
                   3.86369165524282,
                   3.94035517952573]

    qD_list = [3.94014533314375,
                   3.86400827599321,
                   3.79073968041954,
                   3.71958156903273,
                   3.65091004777809,
                   3.58390753773637,
                   3.51825631880380,
                   3.45366669928847,
                   3.38982789443853,
                   3.32782357833707]
    m_D.set_control_mode("speed", 4)
    m_A.set_control_mode("speed", 4)
    m_A.control()
    m_D.control()
    try:
        while True:
            print("=============================dynamic=============================")
            m_D.read_status_once()
            m_D.read_multiturn_once()
            m_D.read_motor_state_once()
            m_D.datadump()

            print("=============================static=============================")
            m_A.read_status_once()
            m_A.read_multiturn_once()
            m_A.read_motor_state_once()
            m_A.datadump()

            time.sleep(0.07)
            pass

        end()

    except KeyboardInterrupt:
        for motor in motors:
            motor.stop_all_tasks()
            motor.motor_off()

        notifier.stop()
        core.CANHelper.cleanup("can0")
        can0.shutdown()
        print("Exiting")
        exit(0)
