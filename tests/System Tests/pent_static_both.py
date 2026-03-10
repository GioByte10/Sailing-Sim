import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time

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

    m_A.read_status_once()
    m_A.read_multiturn_once()
    m_A.read_motor_state_once()
    m_A.datadump()

    m_D.read_status_once()
    m_D.read_multiturn_once()
    m_D.read_motor_state_once()
    m_D.datadump()

    step = float(sys.argv[1])

    if len(sys.argv) == 3:
        s = int(sys.argv[2])

    else:
        s = 1

    # m_D.set_control_mode("torque", -t)
    # m_A.set_control_mode("torque", -t)
    
    m_A.set_control_mode("speed", step)
    m_D.set_control_mode("speed", step)

    m_A.control()
    m_D.control()

    start_time = time.time()

    try:
        while time.time() - start_time < s:
            print("=============================m_D=============================")
            m_D.read_status_once()
            time.sleep(0.02)
            m_D.read_multiturn_once()
            time.sleep(0.02)
            m_D.read_motor_state_once()
            time.sleep(0.02)
            m_D.datadump()
            time.sleep(0.02)

            m_D.control()

            print("=============================m_A=============================")
            m_A.read_status_once()
            time.sleep(0.02)
            m_A.read_multiturn_once()
            time.sleep(0.02)
            m_A.read_motor_state_once()
            time.sleep(0.02)
            m_A.datadump()
            time.sleep(0.02)
            
            m_A.control()

            time.sleep(0.07)
            pass
        end()

    except KeyboardInterrupt:
      end()
