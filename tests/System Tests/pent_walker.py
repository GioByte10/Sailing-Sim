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

A_OFFSET = 2.536
D_OFFSET = 2.373
RANGE = 10
TOP_N = 3

control_modes = {
    "p": "position",
    "v": "speed",
    "s": "shape"
}

def noop(*args, **kwargs):
    pass


def no_control():
    print("No control mode")


def end():
    print("Ending...")
    set_initial_position(A_OFFSET, D_OFFSET)

    for motor in motors:
        motor.stop_all_tasks()
        motor.motor_off()

    notifier.stop()
    core.CANHelper.cleanup("can0")
    can0.shutdown()
    print("Exiting")
    exit(0)


def restart_motors():
    for motor in motors:
        motor.write_acceleration(0x00, np.uint32(60000))
        motor.write_acceleration(0x01, np.uint32(60000))

        motor.write_acceleration(0x02, np.uint32(60000))
        motor.write_acceleration(0x03, np.uint32(60000))

        motor.initialize_motor()
        motor.initialize_control_command()

    if debug:
        datadump()


def parse_arguments():
    debug = False
    it = 0

    if len(sys.argv) >= 3:
        filename = f'cycle_{sys.argv[1]}.mat'
        control_mode = control_modes[sys.argv[2]]

    else:
        print("ERROR: Filename or control mode not provided")
        exit(1)

    if len(sys.argv) >= 4:
        debug = "debug" in sys.argv

        for i in range(3, len(sys.argv)):
            try:
                it = int(sys.argv[i])
            except ValueError:
                it = 0

    return filename, control_mode, debug, it


def datadump():
    for motor in motors:
        motor.read_status_once()
        time.sleep(0.02)
        motor.read_multiturn_once()
        time.sleep(0.02)
        motor.read_motor_state_once()
        time.sleep(0.02)

    for motor in motors:
        motor.datadump()
        time.sleep(0.02)


def set_initial_position(p_A, p_D):
    m_A.set_control_mode("position", p_A)
    m_D.set_control_mode("position", p_D)

    m_A.control()
    m_D.control()

    time.sleep(0.2)

    print(f'p_A = {p_A}')
    print(f'p_D = {p_D}')
    print("Positioning...")

    while True:
        m_A.read_motor_state_once()
        time.sleep(0.02)
        m_A.read_multiturn_once()
        time.sleep(0.02)

        m_D.read_motor_state_once()
        time.sleep(0.02)
        m_D.read_multiturn_once()
        time.sleep(0.02)

        # print(abs(m_A.motor_data.multiturn_position - p_A))
        # print(abs(m_D.motor_data.multiturn_position - p_D))

        if (m_A.motor_data.speed == 0 and m_D.motor_data.speed == 0 and
                abs(m_A.motor_data.multiturn_position - p_A) < 0.1  and
                abs(m_D.motor_data.multiturn_position - p_D) < 0.1):
            print("Positioned")
            break


# noinspection PyUnresolvedReferences
def load_cycle(filename):
    mat_file = scipy.io.loadmat(f'cycles/{filename}')

    qA = mat_file['qA'][0]
    qD = mat_file['qD'][0]

    dqA = mat_file['dqA'][0]
    dqD = mat_file['dqD'][0]

    print(qA)
    print(qD)

    qA = -qA * 5
    qD = -qD * 5

    qA = qA + A_OFFSET
    qD = qD + D_OFFSET

    print(qA)
    print(qD)

    dqA = -dqA * 5
    dqD = -dqD * 5

    l = len(qA)
    print(l)

    if l != len(qD):
        print("ERROR: qA and qD are not the same size")
        exit(2)

    return qA, qD, dqA, dqD, l


def get_control_mode():
    if control_mode == "position":
        return position_control

    elif control_mode == "speed":
        return speed_control

    elif control_mode == "shape":
        return shape_control

    return no_control()


def position_control():
    m_A.set_control_mode("position", qA[step])
    m_D.set_control_mode("position", qD[step])
    m_A.control()
    m_D.control()

    time.sleep(0.005)


def speed_control():
    m_A.set_control_mode("speed", dqA[step])
    m_D.set_control_mode("speed", dqD[step])
    m_A.control()
    m_D.control()

    time.sleep(0.01)


def shape_control():
    global step

    m_A.read_multiturn_once()
    current_positionA = m_A.motor_data.multiturn_position

    start = t - int(RANGE / 2)
    print(start)

    distances = []
    distances_indexes = []

    for i in range(RANGE):
        index = start + i

        if index < 0:
            index = l + index

        elif index >= l:
            index -= l

        print(index)
        print(abs(current_positionA - qA[index]))
        distances.append(abs(current_positionA - qA[index]))
        distances_indexes.append(index)

    sorted_lists = sorted(zip(distances, distances_indexes))
    distances, distances_indexes = zip(*sorted_lists)

    closest = float('inf')
    closest_index = 0

    m_A.read_multiturn_once()
    current_positionD = m_D.motor_data.multiturn_position

    for i in range(TOP_N):
        index = distances_indexes[i]

        if abs(current_positionD - qD[index]) < closest:
            closest_index = index

    t = closest_index
    print(f'This is t: {t}')

    m_A.set_control_mode("speed", dqA[t])
    m_D.set_control_mode("speed", dqD[t])

    m_A.control()
    m_D.control()


if __name__ == "__main__":

    filename, control_mode, debug, it = parse_arguments()
    qA, qD, dqA, dqD, l = load_cycle(filename)

    log = datadump if debug else noop

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=2000, motor_id=7, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=2000, motor_id=0, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)
    notifier = can.Notifier(can0, [motor_listener])

    restart_motors()

    time.sleep(1)
    input("Hey! I'm walking here!")

    set_initial_position(qA[0], qD[0])
    control = get_control_mode()

    step = 0
    i = 0

    while it == 0 or i < it:
        try:

            if step == l - 1:
                step = 0
                i += 1

            log()
            control()

            step += 1

        except (OSError, can.CanOperationError) as e:
            print(f"No crashing allowed {e}")
            time.sleep(0.3)
            m_A.clear_error_flag()
            m_D.clear_error_flag()
            time.sleep(1)

        except KeyboardInterrupt:
            end()

    end()
