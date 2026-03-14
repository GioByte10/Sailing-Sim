import math
import sys
import os

import arcade

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper
import can
import time
sys.path.insert(1, '../../Physics')

import numpy as np
from boat_state import BoatState
from environment_state import Environment
from haptic_state import HapticState
from motor_command_state import MotorCommand
from params import Params
from simulate import run_simulation

sys.path.insert(1, '../../Simulation')

import threading
import math

WIDTH = 1200
HEIGHT = 800
SCREEN_TITLE = "Sailing Simulation"

X = 0
Y = 1

STOP = 0

params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def rotate(matrix, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s],
                  [s, c]])

    return matrix @ R.T


class VectorField:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

        self.translations = np.zeros((self.rows * self.cols, 2), dtype=float)
        self.vectors = np.zeros((self.rows * self.cols, 2), dtype=float)

        # Position of vectors' tails
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = (WIDTH / (self.cols + 1)) * (col + 1)
                self.translations[i, Y] = (HEIGHT / (self.rows + 1)) * (row + 1)

        # Vectors
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col

                r = 30
                theta = i * math.pi / 100

                self.vectors[i, X] = r * math.cos(theta)
                self.vectors[i, Y] = r * math.sin(theta)

    def point_to(self, theta):
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col

                r = 30
                self.vectors[i, X] = r * math.cos(theta)
                self.vectors[i, Y] = r * math.sin(theta)

    def update(self, offset):
        c = np.cos(offset)
        s = np.sin(offset)
        R = np.array([[c, -s],
                      [s, c]])

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col

                self.vectors[i] = R @ self.vectors[i]


class Canvas(arcade.Window):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, SCREEN_TITLE)

        arcade.set_background_color((0, 119, 190, 0))
        self.currentField = VectorField(12, 12)
        self.t = 0

        arcade.schedule(self.on_update, 1/60)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.drawBoat()


    def drawPolygon(self, points, tx, ty):
        n = points.shape[0]

        for i in range(n):
            arcade.draw_line(tx + points[i][0], ty + points[i][1], tx + points[(i + 1)  % n][0], ty + points[(i + 1) % n][1], color=arcade.color.BLACK)


    def drawBoat(self):
        x = boat_state.nu[0] * 100 + WIDTH / 2
        y = boat_state.nu[1] * 100 + HEIGHT / 2

        yaw = boat_state.nu[5]


        body = np.array([[0, -14], [-10, -24], [-10, 0], [0, 10], [10, 0], [10, -24]])
        body_rot = rotate(body, yaw)
        self.drawPolygon(body_rot, x, y)


    def drawCurrentField(self):
        for i in range(self.currentField.rows * self.currentField.cols):
            tx = self.currentField.translations[i, X]
            ty = self.currentField.translations[i, Y]
            x = self.currentField.vectors[i, X]
            y = self.currentField.vectors[i, Y]

            arcade.draw_line(tx, ty, tx + x, ty + y, arcade.color.BLACK)

            r = np.sqrt(np.square(x) + np.square(y))
            theta = np.atan2(y, x)

            tip = np.array([[r - 5, 5], [r, 0], [r - 5, -5]])
            rot_tip = rotate(tip, theta)

            arcade.draw_line(tx + rot_tip[0][0], ty + rot_tip[0][1], tx + rot_tip[1][0], ty + rot_tip[1][1], arcade.color.BLACK)
            arcade.draw_line(tx + rot_tip[1][0], ty + rot_tip[1][1], tx + rot_tip[2][0], ty + rot_tip[2][1], arcade.color.BLACK)

    def on_update(self, delta_time):
        theta = env.wind_field[1] * 180 / math.pi
        self.currentField.point_to(theta)



def initialize_physics():
    global params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque

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


def end_motors(motors, notifier, can0):
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


def run_motors():
    global params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque

    core.CANHelper.init("can0")
    can0 = can.ThreadSafeBus(channel='can0', bustype='socketcan')

    wheel = CanMotor(can0, motor_id=2, gear_ratio=6, name='wheel')
    winch = CanMotor(can0, motor_id=0, gear_ratio=6, name='winch')

    motors = [wheel, winch]
    motor_listener = MotorListener(motor_list=motors)

    notifier = can.Notifier(can0, [motor_listener])

    for motor in motors:
        motor.initialize_motor()
        motor.initialize_control_command()

    time.sleep(1)
    input("Continue")

    for i in range(3):
        for motor in motors:
            motor.read_status_once()
            time.sleep(0.02)
            motor.read_multiturn_once()
            time.sleep(0.02)
            motor.read_motor_state_once()
            time.sleep(0.02)

    wheel_offset = wheel.motor_data.multiturn_position
    winch_offset = winch.motor_data.multiturn_position


    try:
        while True:
            wheel.read_status_once()
            time.sleep(0.02)
            wheel.read_multiturn_once()
            time.sleep(0.02)
            wheel.read_motor_state_once()
            time.sleep(0.02)

            winch.read_status_once()
            time.sleep(0.02)
            winch.read_multiturn_once()
            time.sleep(0.02)
            winch.read_motor_state_once()
            time.sleep(0.02)

            haptic_state.wh[0] = wheel.motor_data.multiturn_position - wheel_offset
            haptic_state.wh[1] = 0
            haptic_state.wh[2] = 0

            haptic_state.wi[0] = winch.motor_data.multiturn_position - winch_offset
            haptic_state.wi[1] = 0.0
            haptic_state.wi[2] = 0.0

            boat_state, tau_total, motor_command = run_simulation(boat_state, haptic_state, env, params)

            # Send haptic torques to motors
            wheel_torque = motor_command.wh_torque  # tau_total[5] / params.steering_ratio
            winch_torque = 0  # motor_command.wi_torque

            wheel_torque = wheel_torque / 6
            wheel_torque = max(-10, min(wheel_torque, 10))

            # print(f"Wheel Torque:  {wheel_torque}")
            # print(f"Winch Torque: {winch_torque}")
            # print(f"Vx: {boat_state.v[0]}")
            # print(f"Vy: {boat_state.v[1]}")
            #
            # print(f"x_pos: {boat_state.nu[0]}")
            # print(f"y_pos: {boat_state.nu[1]}")
            #
            # print(f"Rudder Angle: {(haptic_state.wh[0] / params.steering_ratio) * 180 / math.pi}")
            # print(f"Angle: {boat_state.nu[5] * 180 / math.pi}")
            # print(f"Omega: {boat_state.v[5]}")

            wheel.set_control_mode("torque", wheel_torque)
            wheel.control()
            time.sleep(0.02)

            # wheel.datadump()
            # time.sleep(0.02)

            winch.set_control_mode("torque", winch_torque)
            winch.control()
            time.sleep(0.02)

            winch.datadump()
            time.sleep(0.02)

            t += dt

            log_state.append(boat_state.as_vector())
            log_haptic.append(haptic_state.as_vector())
            log_forces.append(tau_total)
            log_torque.append([wheel_torque, winch_torque])

            if STOP:
                end_motors(motors, notifier, can0)

            pass

    except KeyboardInterrupt:
        end_motors(motors, notifier, can0)


if __name__ == "__main__":
    initialize_physics()

    run_motors_thread = threading.Thread(target=run_motors)
    run_motors_thread.start()

    window = Canvas()

    try:
        arcade.run()

    except KeyboardInterrupt:
        STOP = 1
