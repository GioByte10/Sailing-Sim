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
from control_state import ControlState



WIDTH = 1200
HEIGHT = 800
SCREEN_TITLE = "Sailing Simulation"

X = 0
Y = 1

STOP = 0

params, boat_state, haptic_state, control_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = self.initialTranslations[i, X] - offsetX
                self.translations[i, Y] = self.initialTranslations[i, Y] - offsetY

                if self.translations[i, X] < -self.spacingC:
                    self.translations[i, X] += WIDTH

                elif self.translations[i, X] > WIDTH + self.spacingC:
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
        self.compass_sprite = arcade.Sprite('assets/compass4.png', scale=0.2)
        self.rudder_sprite = arcade.Sprite('assets/rudder__.png', scale=0.2)

        self.compass_sprite.center_x = 70
        self.compass_sprite.center_y = HEIGHT - 70

        self.rudder_sprite.center_x = WIDTH - 120
        self.rudder_sprite.center_y = 120

        self.sprites = arcade.SpriteList()
        self.sprites.append(self.boat_sprite)
        self.sprites.append(self.compass_sprite)
        self.sprites.append(self.rudder_sprite)

        self.boat_path = []

        arcade.schedule(self.on_update, 1/60)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.drawFrame()
        self.drawPath()
        self.sprites.draw()
        self.drawSail()
        self.drawRudderNeedle()


    def on_update(self, delta_time):
        theta = env.wind_field[1] * 180 / math.pi
        self.currentField.point_to(theta)

        update_physics()

        offsetX = boat_state.nu[0] * boat_state.nu[0] * 25
        offsetY = boat_state.nu[1] * 25

        self.boat_path.append([WIDTH / 2 - offsetY, HEIGHT / 2 - offsetX])
        self.currentField.update(offsetY, offsetX)
        self.updateBoat()


    @staticmethod
    def drawPolygon(points, tx, ty):
        n = points.shape[0]

        for i in range(n):
            arcade.draw_line(tx + points[i][0], ty + points[i][1], tx + points[(i + 1)  % n][0], ty + points[(i + 1) % n][1], color=arcade.color.BLACK)


    def updateBoat(self):
        yaw = boat_state.nu[5] * 180 / math.pi

        self.boat_sprite.center_x = WIDTH / 2
        self.boat_sprite.center_y = HEIGHT / 2
        self.boat_sprite.angle = yaw


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


    def drawSail(self):
        sail_angle = -boat_state.nu[5] * 180 / math.pi + 90

        arcade.draw_text("Sail angle: ", WIDTH -70, HEIGHT -20, arcade.color.BLACK, font_size=13, anchor_x="right", anchor_y="top")
        arcade.draw_text(f"{(sail_angle - 90) % 360:.2f}°", WIDTH -78, HEIGHT -40, arcade.color.BLACK, font_size=13, anchor_x="right", anchor_y="top")

        points = []
        start_angle = 30
        end_angle = 150

        width = 60
        height = 34
        center_x = WIDTH / 2
        center_y = HEIGHT / 2

        segments = 120

        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        tilt_rad = math.radians(sail_angle)

        for i in range(segments + 1):
            t = start_rad + (end_rad - start_rad) * i / segments

            x = (width / 2) * math.cos(t)
            y = (height / 2) * math.sin(t)

            rx = x * math.cos(tilt_rad) - y * math.sin(tilt_rad)
            ry = x * math.sin(tilt_rad) + y * math.cos(tilt_rad)

            points.append((center_x + rx, center_y + ry))

        arcade.draw_polygon_filled(points, arcade.color.WHITE)


    def drawFrame(self):
        arcade.draw_line(40, 40, 100, 40, arcade.color.WHITE, 2)
        arcade.draw_line(40, 40, 40, 100, arcade.color.WHITE, 2)

        arcade.draw_text("x", 40, 105, arcade.color.WHITE, font_size=16, anchor_x="center", anchor_y="bottom")
        arcade.draw_text("y", 110, 40, arcade.color.WHITE, font_size=16, anchor_x="left", anchor_y="center")


    def drawPath(self):
        dash_length = 10
        gap_length = 5
        width = 2
        draw = True
        remaining = dash_length

        for j in range(len(self.boat_path) - 1):
            i = len(self.boat_path) - 1 - j
            x1, y1 = self.boat_path[i]
            x2, y2 = self.boat_path[i - 1]

            dx = x2 - x1
            dy = y2 - y1
            segment_length = math.hypot(dx, dy)

            if segment_length == 0:
                continue

            dir_x = dx / segment_length
            dir_y = dy / segment_length

            dist = 0
            cx, cy = x1, y1

            while dist < segment_length:
                step = min(remaining, segment_length - dist)
                nx = cx + dir_x * step
                ny = cy + dir_y * step

                if draw:
                    arcade.draw_line(cx, cy, nx, ny, arcade.color.WHITE, width)

                cx, cy = nx, ny
                dist += step
                remaining -= step

                if remaining <= 0:
                    draw = not draw
                    remaining = dash_length if draw else gap_length

    def drawRudderNeedle(self):
        x = self.rudder_sprite.center_x
        y = self.rudder_sprite.center_y

        r = 40
        theta = control_state.rudder_angle * 2


        arcade.draw_line(x, y, x + r * np.sin(theta), y - r * np.cos(theta), arcade.color.WHITE, 2)


def update_physics():
    boat_state.nu[0] += 0.01
    boat_state.nu[1] += 0.005
    boat_state.nu[5] += 0.005

    control_state.sail_angle += 0.01
    control_state.rudder_angle += 0.005


def initialize_physics():
    global params, boat_state, haptic_state, control_state, motor_command, env, dt, t, t_end, log_state, log_haptic, log_forces, log_torque

    params = Params()
    boat_state = BoatState(params)
    boat_state.nu[0] = 0
    boat_state.nu[1] = 0
    boat_state.nu[1] = 0

    haptic_state = HapticState()
    control_state = ControlState()
    control_state.sail_angle = 0

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