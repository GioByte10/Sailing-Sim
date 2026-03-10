import sys
import os
import threading
from evdev import InputDevice, categorize, ecodes, list_devices

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
import numpy as np
import math
import scipy.io

A_OFFSET = 3.286 - 2 * math.pi
D_OFFSET = 0.6
RANGE = 10
TOP_N = 3

changed_file = False
changed_control_mode = False
restart = False
paused = False
kill_it = False


thread_lock = threading.Lock()

control_indexes = {
    "p": 1,
    "v": 2,
    "s": 3
}

control_modes = {
    1: "position",
    2: "speed",
    3: "shape"
}

cycle_indexes = {

    "short": 1,
    "LN": 2,
    "BC": 3,
    "AD": 4,
    "1": 5,
    "2": 6,
    "3": 7,
    "4": 8,
}

cycle_names = {
    1: "short",
    2: "LN",
    3: "BC",
    4: "AD",
    5: "1",
    6: "2",
    7: "3",
    8: "4",
}

def restart_pid():
    for motor in motors:
        msg_data = [0x31, 0x01, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x31, 0x02, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x31, 0x04, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x31, 0x05, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x32, 0x07, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x32, 0x08, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        msg_data = [0x32, 0x09, 0x64, 0x64, 0x28, 0x1e, 0x32, 0x32]
        motor.raw_send(msg_data)
        time.sleep(0.02)

        motor.write_control_command(0x02, np.uint32(1))

def noop(*args, **kwargs):
    pass


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


def find_controller():
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        print(f"Found: {device.name} at {device.path}")
        if 'Xbox' in device.name or 'xbox' in device.name:
            print("Using:", device.path)
            return device

    print("Controllerless setup")
    return None


def read_controller_inputs(device):

    def change_cycle_index():
        global cycle_index, changed_file

        with thread_lock:
            changed_file = True
            cycle_index -= event.value

            if cycle_index < 1:
                cycle_index += len(cycle_names)

            elif cycle_index > len(cycle_names):
                cycle_index -= len(cycle_names)

    def change_control_index():
        global control_index, changed_control_mode

        with thread_lock:
            changed_control_mode = True
            control_index += event.value

            if control_index < 1:
                control_index += len(control_indexes)

            elif control_index > len(control_indexes):
                control_index -= len(control_indexes)

    global restart, paused, kill_it

    for event in device.read_loop():
        if event.type == ecodes.EV_ABS and event.value:

            if event.code == ecodes.ABS_HAT0Y:
                change_cycle_index()

            elif event.code == ecodes.ABS_HAT0X:
                change_control_index()


        elif event.type == ecodes.EV_KEY:
            key_event = categorize(event)
            if key_event.keystate == key_event.key_down:
                if 'BTN_Y' in key_event.keycode:
                    with thread_lock:
                        restart = True

                elif 'BTN_A' in key_event.keycode:
                    with thread_lock:
                        paused = not paused

                elif 'BTN_B':
                    with thread_lock:
                        kill_it = True


def graceful_end():
    print("Ending")
    set_initial_position(A_OFFSET, D_OFFSET)
    end()


def end():
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
        motor.write_acceleration(0x00, np.uint32(600))
        motor.write_acceleration(0x01, np.uint32(600))

        motor.write_acceleration(0x02, np.uint32(600))
        motor.write_acceleration(0x03, np.uint32(600))

        motor.initialize_motor()
        motor.initialize_control_command()

    if debug:
        datadump()


def parse_arguments():
    debug = False
    slow = False
    loop_target = 0

    if len(sys.argv) >= 3:
        cycle_index = cycle_indexes[sys.argv[1]]
        control_index = control_indexes[sys.argv[2]]

    else:
        print("ERROR: Filename or control mode not provided")
        exit(1)

    if len(sys.argv) >= 4:
        debug = "debug" in sys.argv
        slow = "slow" in sys.argv

        for i in range(3, len(sys.argv)):
            try:
                loop_target = int(sys.argv[i])

            except ValueError:
                loop_target = 0

    return cycle_index, control_index, debug, slow, loop_target


def set_initial_position(p_A, p_D, stop=False):
    m_A.set_control_mode("position", p_A)
    m_D.set_control_mode("position", p_D)

    m_A.control()
    time.sleep(0.02)

    m_D.control()
    time.sleep(0.02)

    print()
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

        print(f'p_A = {p_A} m_A = {m_A.motor_data.multiturn_position}')
        print(f'p_D = {p_D} m_D = {m_D.motor_data.multiturn_position}')

        if kill_it:
            end()

        if (not stop or (m_A.motor_data.speed == 0 and m_D.motor_data.speed == 0) and
                abs(m_A.motor_data.multiturn_position - p_A) < 0.1  and
                abs(m_D.motor_data.multiturn_position - p_D) < 0.1):

            print(abs(m_A.motor_data.multiturn_position - p_A) < 0.1)
            print(abs(m_D.motor_data.multiturn_position - p_D) < 0.1)
            print("Positioned")
            print()
            break

        m_A.set_control_mode("position", p_A)
        m_D.set_control_mode("position", p_D)

        m_A.control()
        time.sleep(0.02)

        m_D.control()
        time.sleep(0.02)


# noinspection PyUnresolvedReferences
def load_cycle():
    mat_file = scipy.io.loadmat(f'cycles/cycle_{cycle_names[cycle_index]}.mat')

    period = mat_file['T'][0][0]

    qA = mat_file['qA'][0]
    qD = mat_file['qD'][0]

    dqA = mat_file['dqA'][0]
    dqD = mat_file['dqD'][0]

    print("Original vectors")
    print(f"qA: {qA}")
    print(f"qD: {qD}")

    qA = -qA * 5
    qD = -qD * 5

    qA = qA + A_OFFSET + 5 * math.pi / 2
    qD = qD + D_OFFSET + 5 * math.pi / 2

    print("Final Vectors")
    print(f"qA: {qA}\n")
    print(f"qD: {qD}\n")

    dqA = -dqA * 5
    dqD = -dqD * 5

    print("Final Vectors")
    print(f"dqA: {dqA}\n")
    print(f"dqD: {dqD}\n")

    path_length = len(qA)
    print()
    print(f"Using cycles/cycle_{cycle_names[cycle_index]}.mat")
    print(f"Path contains {path_length} points")

    if path_length != len(qD):
        print("ERROR: qA and qD are not the same size")
        exit(2)

    return qA, qD, dqA, dqD, path_length, period


def get_control_mode():

    def no_control():
        print("No control mode")


    def position_control():
        m_A.set_control_mode("position", qA[step])
        m_D.set_control_mode("position", qD[step])

        m_A.control()
        time.sleep(0.005)

        m_D.control()
        time.sleep(0.005)


    def speed_control():

        if step == 0:
            set_initial_position(qA[0], qD[0], True)

        m_A.set_control_mode("speed", dqA[step])
        m_D.set_control_mode("speed", dqD[step])

        m_A.control()
        time.sleep(period / (2.0 * path_length))

        m_D.control()
        time.sleep(period / (2.0 * path_length))


    def shape_control():
        global step

        m_A.read_motor_state_once()
        time.sleep(0.005)
        m_A.read_multiturn_once()
        time.sleep(0.005)

        m_D.read_motor_state_once()
        time.sleep(0.005)
        m_D.read_multiturn_once()
        time.sleep(0.005)

        if m_A.motor_data.speed != 0:
            current_position_1 = m_A.motor_data.multiturn_position
            current_position_2 = m_D.motor_data.multiturn_position
            q1 = qA
            q2 = qD

        elif m_D.motor_data.speed:
            current_position_1 = m_D.motor_data.multiturn_position
            current_position_2 = m_A.motor_data.multiturn_position
            q1 = qD
            q2 = qA

        else:
            print("early exit")
            step += 1
            return

        start = step - int(RANGE / 2)

        distances = []
        distances_indexes = []

        for i in range(RANGE):
            index = start + i

            if index < 0:
                index = path_length + index

            elif index >= path_length:
                index -= path_length

            # print(f"{index}")
            # print(abs(current_positionA - qA[index]))
            # input()
            distances.append(abs(current_position_1 - q1[index]))
            distances_indexes.append(index)

        sorted_lists = sorted(zip(distances, distances_indexes))
        distances, distances_indexes = zip(*sorted_lists)

        closest = float('inf')
        closest_index = 0

        for i in range(TOP_N):
            index = distances_indexes[i]
            # print(index)
            # print(abs(current_positionD - qD[index]))

            if abs(current_position_2 - q2[index]) < closest:
                closest = abs(current_position_2 - q2[index])
                closest_index = index

        step = closest_index
        print(f'This is t: {step}')
        # #input()

        m_A.set_control_mode("speed", dqA[step])
        m_D.set_control_mode("speed", dqD[step])

        m_A.control()
        time.sleep(period / (2 * path_length))

        m_D.control()
        time.sleep(period / (2 * path_length))


    if control_modes[control_index] == "position":
        return position_control

    elif control_modes[control_index] == "speed":
        return speed_control

    elif control_modes[control_index] == "shape":
        return shape_control

    return no_control()


def fake_main():
    global changed_control_mode, changed_file, restart
    while True:
        print(f"Cycle name is {cycle_names[cycle_index]}")
        print(f"Control is {control_modes[control_index]}")
        time.sleep(0.5)

        if changed_file:
            print("___________________________CYCLE")
            with thread_lock:
                changed_file = False

        if changed_control_mode:
            print("____________________________CONTROL")
            with thread_lock:
                changed_control_mode = False

        if restart:
            print("restart")
            restart = False


def handle_restart():
    global step, restart

    print("Restarting...")
    set_initial_position(A_OFFSET, D_OFFSET)
    time.sleep(1)
    set_initial_position(qA[0], qD[0])

    step = 0
    with thread_lock:
        restart = False

    print("Running path")


def handle_changed_file():
    global step, changed_file

    print("Changing .mat file...")
    qA, qD, dqA, dqD, path_length = load_cycle
    set_initial_position(A_OFFSET, D_OFFSET)
    time.sleep(1)
    set_initial_position(qA[0], qD[0])

    step = 0
    with thread_lock:
        changed_file = False

    print("Running path")


def handle_changed_control_mode():
    global step, control, changed_control_mode

    print(f"Changing control mode to {control_modes[control_index]}")
    set_initial_position(A_OFFSET, D_OFFSET)
    time.sleep(1)
    set_initial_position(qA[0], qD[0])
    control = get_control_mode()

    step = 0
    with thread_lock:
        changed_control_mode = False

    print("Running path")


if __name__ == "__main__":

    cycle_index, control_index, debug, slow, loop_target = parse_arguments()
    qA, qD, dqA, dqD, path_length, period = load_cycle()

    log = datadump if debug else noop

    controller = find_controller()

    if controller is not None:
        controller_thread = threading.Thread(target=read_controller_inputs, args=(controller,), daemon=True)
        controller_thread.start()

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    m_A = CanMotor(can0, MAX_SPEED=700 if slow else 1000, motor_id=8, gear_ratio=1, name="A")  # m_A
    m_D = CanMotor(can0, MAX_SPEED=700 if slow else 1000, motor_id=2, gear_ratio=1, name="D")  # m_D
    motors = [m_A, m_D]

    motor_listener = MotorListener(motor_list=motors)
    notifier = can.Notifier(can0, [motor_listener])

    restart_pid()
    restart_motors()

    time.sleep(1)
    print()
    input("Hey! I'm walking here!")

    set_initial_position(A_OFFSET, D_OFFSET, True)
    time.sleep(0.3)
    set_initial_position(qA[0], qD[0], True)
    control = get_control_mode()

    step = 0
    loop_count = 0

    print("Running path")

    while loop_target == 0 or loop_count < loop_target:
        try:
            if paused:
                continue

            if step == path_length - 1:
                step = 0
                loop_count += 1

            log()
            control()

            if restart:
                handle_restart()

            if changed_file:
                handle_changed_file()

            if changed_control_mode:
                handle_changed_control_mode()

            if kill_it:
                end()

            step += 1

        except (OSError, can.CanOperationError) as e:
            print(f"No crashing allowed {e}")
            time.sleep(0.3)
            m_A.clear_error_flag()
            m_D.clear_error_flag()
            time.sleep(1)

            if kill_it:
                end()

        except KeyboardInterrupt:
            graceful_end()

    graceful_end()