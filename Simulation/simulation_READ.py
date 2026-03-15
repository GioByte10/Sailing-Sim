import math
import sys
import os

import arcade
sys.path.insert(1, '../../Physics')

import numpy as np
from boat_state import BoatState
from environment_state import Environment
from haptic_state import HapticState
from motor_command_state import MotorCommand
from params import Params
from simulate import run_simulation


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
    boat_state
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


if __name__ == "__main__":
    initialize_physics()

    window = Canvas()
    arcade.run()