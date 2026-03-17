import math
import sys
import os
import time
import threading

import arcade
import numpy as np
import can

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from core.CanMotorNew import CanMotor
from core.MotorListener import MotorListener
import core.CANHelper

WIDTH = 1200
HEIGHT = 800
SCREEN_TITLE = "Dynamic Haptic Sailing Simulation"

X = 0
Y = 1
STOP = 0

# --- THREAD-SAFE STATE BRIDGE ---
# This dictionary shares the physics data with the arcade graphics thread
sim_state = {
    'heading': 0.0,
    'pos_x': 0.0,
    'pos_y': 0.0,
    'wind_angle_deg': 0.0,
    'boat_speed_kts': 0.0
}

yaw = math.pi / 2
sail_angle = 0
rudder_angle = 0
rudder_boat_angle = 0

offset_sail_boat_angle = 0

acceleration = np.array([0, 0], dtype=np.float64)
velocity = np.array([10, 0], dtype=np.float64)
position = np.array([0, 0], dtype=np.float64)
wind = np.array([10, 0], dtype=np.float64)

def rotate(matrix, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s], [s, c]])
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

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                self.translations[i, X] = 0.5 * self.spacingC + col * self.spacingC
                self.translations[i, Y] = 0.5 * self.spacingR + row * self.spacingR

        self.initialTranslations = self.translations.copy()

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                r = 30
                theta = i * math.pi / 100
                self.vectors[i, X] = r * math.cos(theta)
                self.vectors[i, Y] = r * math.sin(theta)

    def point_to(self, theta_rad):
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                r = 30
                self.vectors[i, X] = r * math.cos(theta_rad)
                self.vectors[i, Y] = r * math.sin(theta_rad)

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

        arcade.set_background_color((0, 119, 190, 0))
        self.currentField = VectorField(8, 8)

        self.boat_sprite = arcade.Sprite('assets/boat.png', scale=0.6)
        self.compass_sprite = arcade.Sprite('assets/compass4.png', scale=0.15)
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
        self.t = 0

        arcade.schedule(self.on_update, 1 / 60)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.drawFrame()
        self.drawPath()
        self.sprites.draw()
        self.drawSail()
        self.drawRudderNeedle()

        # Overlay Speed HUD
        arcade.draw_text(f"Speed: {sim_state['boat_speed_kts']:.1f} kts", 10, HEIGHT - 30, arcade.color.WHITE, 16)
        arcade.draw_text(f"Heading: {sim_state['heading']:.0f}°", 10, HEIGHT - 55, arcade.color.WHITE, 16)

    def on_update(self, delta_time):
        offsetX = position[X] * 10
        offsetY = position[Y] * 10

        self.boat_path.append([WIDTH / 2 - offsetX, HEIGHT / 2 - offsetY])
        self.currentField.update(offsetX, offsetY)
        self.updateBoat()

        theta_rad = np.arctan2(wind[Y], wind[X])
        self.currentField.point_to(theta_rad)
        self.t += 0.1

    @staticmethod
    def drawPolygon(points, tx, ty):
        n = points.shape[0]
        for i in range(n):
            arcade.draw_line(tx + points[i][0], ty + points[i][1], tx + points[(i + 1) % n][0],
                             ty + points[(i + 1) % n][1], color=arcade.color.BLACK)

    def updateBoat(self):
        self.boat_sprite.center_x = WIDTH / 2
        self.boat_sprite.center_y = HEIGHT / 2
        self.boat_sprite.angle = -(yaw * 180 / math.pi - 90)

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

            arcade.draw_line(tx + rot_tip[0][0], ty + rot_tip[0][1], tx + rot_tip[1][0], ty + rot_tip[1][1],
                             arcade.color.BLACK)
            arcade.draw_line(tx + rot_tip[1][0], ty + rot_tip[1][1], tx + rot_tip[2][0], ty + rot_tip[2][1],
                             arcade.color.BLACK)

    def drawSail(self):
        arcade.draw_text("Sail angle: ", WIDTH - 70, HEIGHT - 20, arcade.color.BLACK, font_size=13, anchor_x="right",
                         anchor_y="top")
        arcade.draw_text(f"{(sail_angle * 180 / math.pi) % 360:.2f}°", WIDTH - 78, HEIGHT - 40, arcade.color.BLACK,
                         font_size=13, anchor_x="right", anchor_y="top")

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
        tilt_rad = sail_angle

        for i in range(segments + 1):
            t = start_rad + (end_rad - start_rad) * i / segments

            x = (width / 2) * math.cos(t)
            y = (height / 2) * math.sin(t)

            rx = x * math.cos(tilt_rad) - y * math.sin(tilt_rad)
            ry = x * math.sin(tilt_rad) + y * math.cos(tilt_rad)

            points.append([center_x + rx, center_y + ry])

        # tx = -30 * math.cos(tilt_rad)
        # ty = -30 * math.sin(tilt_rad)
        #
        # for i in range(segments + 1):
        #     points[i][0] += tx
        #     points[i][1] += ty

        arcade.draw_polygon_filled(points, arcade.color.WHITE)
        arcade.draw_point(points[0][X], points[0][Y], arcade.color.GREEN, 5)

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
        theta = rudder_boat_angle * 2
        arcade.draw_line(x, y, x + r * np.sin(theta), y - r * np.cos(theta), arcade.color.WHITE, 2)

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
    print("Exiting safely.")
    exit(0)


def run_motors():
    global STOP, yaw, sail_angle, rudder_angle, rudder_boat_angle, position, velocity

    # Initialize Hardware
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

    # Pull initial offsets
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

    yaw_omega = 0

    rudder_boat_angle = 0

    k_yaw = 0.5
    k_rudder = 0.5
    b = 0.2

    CL = 1.2
    CD = 0.12

    mass = 3100
    A = 150

    rho = 1030
    cdhx = 0.1
    cdhy = 0.1

    Ax = 2.5
    Ay = 7


    try:
        while True:
            wheel.read_status_once()
            time.sleep(0.01)
            wheel.read_multiturn_once()
            time.sleep(0.01)
            wheel.read_motor_state_once()
            time.sleep(0.01)

            winch.read_status_once()
            time.sleep(0.01)
            winch.read_multiturn_once()
            time.sleep(0.01)
            winch.read_motor_state_once()
            time.sleep(0.01)

            wheel_angle = (wheel.motor_data.multiturn_position - wheel_offset)
            winch_angle = (winch.motor_data.multiturn_position - winch_offset)

            rudder_boat_angle = wheel_angle / 10
            sail_boat_angle = winch_angle / 10 + offset_sail_boat_angle

            rudder_angle = rudder_boat_angle + yaw
            sail_angle = sail_boat_angle + yaw

            wind_angle = np.arctan2(wind[Y], wind[X])
            wind_mag = np.linalg.norm(wind)
            velocity_angle = np.arctan2(velocity[Y], velocity[X])

            dt = 0.08

            wheel_torque = k_yaw * np.sin(yaw - wind_angle)

            if np.linalg.norm(velocity) > 0.001:
                wheel_torque +=  k_rudder * np.sin(rudder_angle - velocity_angle)

            wheel_torque *= 10
            wheel_torque = np.clip(wheel_torque, -6, 6)

            yaw_moment = k_yaw * np.sin(yaw - wind_angle) - b * yaw_omega
            yaw_omega += yaw_moment * dt
            yaw += yaw_omega * dt

            print(f"Wind mag: {wind_mag}")
            force_lift = 0.5 * A * CL * np.square((np.sin(sail_angle - wind_angle) * wind_mag)) * np.sign(np.cos(sail_angle - wind_angle))
            # force_drag = 0.5 * A * CD * (np.cos(sail_angle - wind_angle) * wind_mag) ^ 2

            x_drag_body = -0.5 * rho * cdhx * Ax * (np.cos(-yaw) * velocity[X]) * abs(np.cos(-yaw) * velocity[X])
            y_drag_body = -0.5 * rho * cdhy * Ay * (np.cos(-yaw) * velocity[Y]) * abs(np.cos(-yaw) * velocity[Y])

            acceleration[X] = (force_lift * np.cos(sail_angle - math.pi / 2) + x_drag_body * np.cos(-yaw) + y_drag_body * np.sin(-yaw)) / mass
            acceleration[Y] = (force_lift * np.sin(sail_angle - math.pi / 2) + x_drag_body * np.sin(-yaw) + y_drag_body * np.cos(-yaw)) / mass

            print()
            print(f"Sail angle: {sail_angle * 180 / math.pi}")
            print(f"Sin: {np.sin(sail_angle - math.pi / 2)}")
            print(f"Lift force: {force_lift}")
            print("Acceleration:", acceleration)
            print("Velocity:", velocity)
            print("Position:", position)
            print()

            print(f"Sail angle: {sail_angle * 180 / math.pi}")
            print(f"Wind angle: {wind_angle* 180 / math.pi}")
            print(f"yaw moment: {yaw_moment * 180 / math.pi}")
            print(f"yaw omega: {yaw_omega * 180 / math.pi}")
            print(f"yaw angle: {yaw * 180 / math.pi}")

            print()
            print(f"Yaw contribution: {k_yaw * np.sin(yaw - wind_angle)}")
            print(f"Rudder angle: {rudder_angle * 180 / math.pi}")
            print(f"Velocity angle: {velocity_angle * 180 / math.pi}")
            print(f"Delta: {(rudder_angle - velocity_angle) * 180 / math.pi}")
            print(f"Sin: {np.sin(rudder_angle - velocity_angle)}")
            print(f"Rudder contribution: {k_rudder * np.sin(rudder_angle - velocity_angle)}")
            print(f"wheel torque {wheel_torque}")

            wheel.set_control_mode("torque", wheel_torque)
            wheel.control()
            time.sleep(0.01)

            winch.set_control_mode("torque", 0)
            winch.control()
            time.sleep(0.01)

            velocity += acceleration * dt
            position += velocity * dt

            if STOP:
                end_motors(motors, notifier, can0)

    except KeyboardInterrupt:
        end_motors(motors, notifier, can0)


if __name__ == "__main__":
    run_motors_thread = threading.Thread(target=run_motors, daemon=True)
    run_motors_thread.start()

    window = Canvas()

    try:
        arcade.run()
    except KeyboardInterrupt:
        STOP = 1