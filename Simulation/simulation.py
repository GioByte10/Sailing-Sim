import arcade
import math
import numpy as np
import random

WIDTH = 1200
HEIGHT = 800
SCREEN_TITLE = "Sailing Simulation"

X = 0
Y = 1


def rotate(matrix, theta):
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s],
                  [s, c]])

    return R @ matrix

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

    def point_to(self, x, y):
        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col

                r = 30
                theta = np.atan2(y - HEIGHT / 2, x - WIDTH / 2)

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

    def on_mouse_motion(self, x, y, dx, dy):
        self.currentField.point_to(x, y)

    def on_draw(self):
        self.clear()
        self.drawCurrentField()
        self.drawBoat()

    def drawBoat(self):
        pass

    def drawCurrentField(self):
        for i in range(self.currentField.rows * self.currentField.cols):
            tx = self.currentField.translations[i, X]
            ty = self.currentField.translations[i, Y]
            x = self.currentField.vectors[i, X]
            y = self.currentField.vectors[i, Y]

            arcade.draw_line(tx, ty, tx + x, ty + y, arcade.color.BLACK)

            r = np.sqrt(np.square(x) + np.square(y))
            theta = np.atan2(y, x)

            tip = np.array([[r - 5, r, r - 5],[5, 0, -5]])
            rot_tip = rotate(tip, theta)

            arcade.draw_line(tx + rot_tip[0][0], ty + rot_tip[1][0], tx + rot_tip[0][1], ty + rot_tip[1][1], arcade.color.BLACK)
            arcade.draw_line(tx + rot_tip[0][1], ty + rot_tip[1][1], tx + rot_tip[0][2], ty + rot_tip[1][2], arcade.color.BLACK)

    def on_update(self, delta_time):
        self.t += 1


if __name__ == "__main__":
    window = Canvas()
    arcade.run()