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

    m_A = CanMotor(can0, motor_id=7, gear_ratio=1)
    m_D = CanMotor(can0, motor_id=0, gear_ratio=1)
    motors = [m_A, m_D]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")


    # m_A.set_control_mode("position", 4.3)
    # m_A.control()
    #
    # m_D.set_control_mode("position", 1.1)
    # m_D.control()

    m_A.read_status_once()
    m_A.read_multiturn_once()
    m_A.read_motor_state_once()
    m_A.datadump()

    m_D.read_status_once()
    m_D.read_multiturn_once()
    m_D.read_motor_state_once()
    m_D.datadump()

    m_A.set_control_mode("torque", -7)
    m_D.set_control_mode("torque", -7)

    m_A.control()
    m_D.control()

    try:
        while True:
            print("=============================m_A=============================")
            m_A.read_status_once()
            m_A.read_multiturn_once()
            m_A.read_motor_state_once()
            m_A.datadump()

            p_A = m_A.motor_data.singleturn_position + 2 * math.pi - 5.5
            if m_A.motor_data.singleturn_position + 2 * math.pi - 5.4 > 2 * math.pi:
                p_A -= 2 * math.pi

            t_A = p_A - math.pi
            t_A = math.sin(t_A)

            # m_A.set_control_mode("torque", 3 * t_A)
            # m_A.control()

            print(p_A)
            print(t_A)
            print()

            print("=============================D=============================")
            m_D.read_status_once()
            m_D.read_multiturn_once()
            m_D.read_motor_state_once()
            m_D.datadump()

            p_D = m_D.motor_data.singleturn_position + 2 * math.pi - 2.1
            if m_D.motor_data.singleturn_position + 2 * math.pi - 2.03 > 2 * math.pi:
                p_D -= 2 * math.pi

            print(p_D)


            t_D = p_D - math.pi
            t_D = math.sin(t_D)

            # m_D.set_control_mode("torque", 3 * t_D)
            # m_D.control()

            print(t_D)

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
