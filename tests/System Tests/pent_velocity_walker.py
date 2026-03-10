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


def end():
    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


def load_cycle(filename):
    a = 0

    mat_file = scipy.io.loadmat(f'cycles/{filename}')
    a_positions = mat_file['qA'][0]
    d_positions = mat_file['qD'][0]
    print(a_positions)
    print(d_positions)

    a_positions = -a_positions
    d_positions = -d_positions
    print(a_positions)
    print(d_positions)

    a_positions = a_positions + 5.68
    d_positions = d_positions + 1.73 - 2 * math.pi
    print(a_positions)
    print(d_positions)
    # Load Velocities
    a_velocities = mat_file['dqA'][0]
    d_velocities = mat_file['dqD'][0]
    # Flip to motor direction
    a_velocities = -a_velocities
    d_velocities = -d_velocities

    l = len(a_positions)

    if l != len(d_positions):
        print("ERROR: dqA and dqD are not the same size")
        exit(2)

    return a_positions, d_positions, a_velocities, d_velocities, l


if __name__ == "__main__":

    if len(sys.argv) >= 2:
        filename = f'cycle_vel_{sys.argv[1]}.mat'

    else:
        print("ERROR: Filename not provided")
        exit(1)

    if len(sys.argv) >= 3:
        it = int(sys.argv[2])

    else:
        it = 0

    a_positions, d_positions, a_velocities, d_velocities, l = load_cycle(filename)

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=400, motor_id=7, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=400, motor_id=0, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.write_acceleration(0x02, np.uint32(60000))
        motor.write_acceleration(0x03, np.uint32(60000))

        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")
    # Send motor to start position
    step = 0
    m_A.set_control_mode("position", a_positions[step])
    m_D.set_control_mode("position", d_positions[step])

    print(a_positions[step])
    print(d_positions[step])

    m_A.control()
    m_D.control()
    # Hold start pose
    time.sleep(1)
    # Start at initial velocity
    m_A.set_control_mode("speed", a_velocities[step])
    m_D.set_control_mode("speed", d_velocities[step])
    m_A.control()
    m_D.control()

    time.sleep(.01)

    i = 0

    try:
        while it == 0 or i < it:
            for motor in motors:
                motor.read_status_once()
                time.sleep(0.02)
            #     motor.read_multiturn_once()
            #     time.sleep(0.02)
            #     motor.read_motor_state_once()
            #     time.sleep(0.02)
            #
            # for motor in motors:
            #     motor.datadump()
            #     time.sleep(0.02)

            step += 1

            m_A.set_control_mode("speed", a_velocities[step])
            m_D.set_control_mode("speed", d_velocities[step])
            m_A.control()
            m_D.control()

            time.sleep(0.01)

            if step == l - 1:
                step = 0
                i += 1

        end()

    except KeyboardInterrupt:
        end()
