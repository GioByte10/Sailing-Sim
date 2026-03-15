import math
import sys
import os

import arcade
sys.path.insert(1, '../Physics')

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

        self.spacingR = HEIGHT // self.rows
        self.spacingC = WIDTH // self.cols

        self.rows += 1
        self.cols += 1

        self.translations = np.zeros((self.rows * self.cols, 2), dtype=float)
        self.vectors = np.zeros((self.rows * self.cols, 2), dtype=float)

        # Position of vectors' tails
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = 0.5 * self.spacingC + col * self.spacingC
                self.translations[i, Y] = 0.5 * self.spacingR + row * self.spacingR

        self.initialTranslations = self.translations.copy()

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

    def update(self, offsetX, offsetY):
        offsetX = offsetX % self.spacingC
        offsetY = offsetY % self.spacingR

        print(offsetX, offsetY)

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                print("Before")
                print(self.translations[i, X], self.spacingC)
                self.translations[i, X] = self.initialTranslations[i, X] - offsetX
                self.translations[i, Y] = self.initialTranslations[i, Y] - offsetY
                print("After")
                print(self.translations[i, X], self.spacingC)

                if self.translations[i, X] < -self.spacingC:
                    print("HERE-")
                    print(self.translations[i, X], self.spacingC)
                    self.translations[i, X] += WIDTH

                elif self.translations[i, X] > WIDTH + self.spacingC:
                    print("HERE+")
                    print(self.translations[i, X], self.spacingC)
                    self.translations[i, X] -= WIDTH

                if self.translations[i, Y] < -self.spacingR / 2:
                    self.translations[i, Y] += HEIGHT

                elif self.translations[i, Y] > HEIGHT + self.spacingR / 2:
                    self.translations[i, Y] -= HEIGHT



class Canvas(arcade.Window):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, SCREEN_TITLE)

        arcade.set_background_color((0, 100, 170, 0))
        self.currentField = VectorField(8, 8)

        self.boat_sprite = arcade.Sprite('assets/boat.png', scale=0.6)
        self.compass_sprite = arcade.Sprite('assets/compass4.png', scale=0.15)

        self.compass_sprite.center_x = 70
        self.compass_sprite.center_y = HEIGHT - 70

        self.sprites = arcade.SpriteList()
        self.sprites.append(self.boat_sprite)
        self.sprites.append(self.compass_sprite)

        arcade.schedule(self.on_update, 1/60)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.updateBoat()
        self.sprites.draw()


    def on_update(self, delta_time):
        theta = env.wind_field[1] * 180 / math.pi
        self.currentField.point_to(theta)
        update_physics()



    @staticmethod
    def drawPolygon(points, tx, ty):
        n = points.shape[0]

        for i in range(n):
            arcade.draw_line(tx + points[i][0], ty + points[i][1], tx + points[(i + 1)  % n][0], ty + points[(i + 1) % n][1], color=arcade.color.BLACK)


    def updateBoat(self):
        x = boat_state.nu[0] * 100 + WIDTH / 2
        y = boat_state.nu[1] * 100 + HEIGHT / 2

        yaw = boat_state.nu[5]

        self.boat_sprite.center_x = WIDTH / 2
        self.boat_sprite.center_y = HEIGHT / 2
        self.boat_sprite.angle = yaw




    def drawCurrentField(self):
        offsetX = boat_state.nu[0] * 100
        offsetY = boat_state.nu[1] * 100
        self.currentField.update(offsetX, offsetY)

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


def update_physics():
    boat_state.nu[0] += 0.01
    boat_state.nu[1] += 0.01
    boat_state.nu[5] += 0.1
    pass


def initialize_physics():
    global params, boat_state, haptic_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque

    params = Params()
    boat_state = BoatState()
    boat_state.nu[0] = 0
    boat_state.nu[1] = 0
    boat_state.nu[1] = 0

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