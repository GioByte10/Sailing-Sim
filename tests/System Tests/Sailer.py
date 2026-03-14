import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
sys.path.insert(1, '../../Physics')

from boat_state import BoatState
from environment_state import Environment
from haptic_state import HapticState
from motor_command_state import MotorCommand
from params import Params
from simulate import run_simulation

import arcade
import math

WIDTH = 1200
HEIGHT = 800


def initialize_physics():
    params = Params()
    boat_state = BoatState()
    haptic_state = HapticState()
    motor_command = MotorCommand()
    env = Environment()

    dt = params.dt
    t = params.t_start
    t_end = params.t_end

    log_state = []
    log_haptic = []
    log_forces = []
    log_torque = []

    return params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque


if __name__ == "__main__":

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    wheel = CanMotor(can0, motor_id=0, gear_ratio=6, name='wheel')
    #winch = CanMotor(can0, motor_id=1, gear_ratio=6)

    motors = [wheel]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    for i in range(3):
        for motor in motors:
            wheel.read_status_once()
            time.sleep(0.02)
            wheel.read_multiturn_once()
            time.sleep(0.02)
            wheel.read_motor_state_once()
            time.sleep(0.02)

    wheel_offset = wheel.motor_data.multiturn_position

    params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque = initialize_physics()


    try:
        while True:
            wheel.read_status_once()
            time.sleep(0.02)
            wheel.read_multiturn_once()
            time.sleep(0.02)
            wheel.read_motor_state_once()
            time.sleep(0.02)

            # print("WHEEL")
            # print(wheel_offset)
            # print(wheel.motor_data.multiturn_position)

            haptic_state.wh[0] = wheel.motor_data.multiturn_position - wheel_offset
            haptic_state.wh[1] = 0
            haptic_state.wh[2] = 0

            haptic_state.wi[0] = 0.5
            haptic_state.wi[1] = 0.0
            haptic_state.wi[2] = 0.0

            boat_state, tau_total, motor_command = run_simulation(boat_state, haptic_state, env, params)

            # Send haptic torques to motors
            wheel_torque = motor_command.wh_torque  # tau_total[5] / params.steering_ratio
            winch_torque = motor_command.wi_torque  # motor_command.wi_torque

            wheel_torque = wheel_torque / 6
            wheel_torque = max(-10, min(wheel_torque, 10))

            print(f"Wheel Torque:  {wheel_torque}")
            print(f"Winch Torque: {winch_torque}")
            print(f"Vx: {boat_state.v[0]}")
            print(f"Vy: {boat_state.v[1]}")

            print(f"x_pos: {boat_state.nu[0]}")
            print(f"y_pos: {boat_state.nu[1]}")

            print(f"Rudder Angle: {(haptic_state.wh[0] / params.steering_ratio) * 180 / math.pi}")
            print(f"Angle: {boat_state.nu[5] * 180 / math.pi}")
            print(f"Omega: {boat_state.v[5]}")

            wheel.set_control_mode("torque", wheel_torque)
            wheel.control()
            time.sleep(0.02)

            wheel.datadump()
            time.sleep(0.02)

            t += dt

            log_state.append(boat_state.as_vector())
            log_haptic.append(haptic_state.as_vector())
            log_forces.append(tau_total)
            log_torque.append([wheel_torque, winch_torque])

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
