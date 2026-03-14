import arcade
import math

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600


class Game(arcade.Window):

    def __init__(self):
        super().__init__(SCREEN_WIDTH, SCREEN_HEIGHT, "Boat Grid")

        self.world_x = 0
        self.world_y = 0

        self.velocity = 2
        self.angle = 0

    def on_update(self, delta_time):

        vel_x = math.cos(self.angle) * self.velocity
        vel_y = math.sin(self.angle) * self.velocity

        self.world_x -= vel_x
        self.world_y -= vel_y

    def on_draw(self):

        arcade.start_render()

        spacing = 50

        for x in range(-30, 30):
            for y in range(-30, 30):

                draw_x = x * spacing + self.world_x
                draw_y = y * spacing + self.world_y

                arcade.draw_circle_filled(draw_x, draw_y, 2, arcade.color.WHITE)

        # boat stays centered
        arcade.draw_triangle_filled(
            SCREEN_WIDTH//2 - 10,
            SCREEN_HEIGHT//2 - 10,
            SCREEN_WIDTH//2 + 10,
            SCREEN_HEIGHT//2 - 10,
            SCREEN_WIDTH//2,
            SCREEN_HEIGHT//2 + 15,
            arcade.color.BLUE
        )


Game()
arcade.run()