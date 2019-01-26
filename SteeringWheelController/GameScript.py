import pygame
from pygame.locals import *
import ControllerConstants
import CarConstants
from ValueConverter import linear_converter
import math

class App:
    def __init__(self):
        pygame.init()

        pygame.display.set_caption("Space Agency")
        self.screen = pygame.display.set_mode((300, 270))
        self.font = pygame.font.SysFont("Courier", 20)

        # Set up the joystick
        pygame.joystick.init()

        self.my_joystick = pygame.joystick.Joystick(0)
        self.my_joystick.init()

    def draw_text(self, text, x, y, color, align_right=False):
        surface = self.font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey((0, 0, 0))

        self.screen.blit(surface, (x, y))

    def get_speed(self):
        axis = ControllerConstants.SPEED_AXIS
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_SPEED, CarConstants.MAX_SPEED, value, invert=True)

        return value, converted_value

    def get_wheel_angle(self):
        axis = ControllerConstants.DIRECTION_AXIS
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_WHEEL_ANGLE, CarConstants.MAX_WHEEL_ANGLE, value)

        return value, converted_value

    def get_speed_degree(self):
        axis = ControllerConstants.SPEED_AXIS
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_SPEED_DEGREE, CarConstants.MAX_SPEED_DEGREE, value, invert=True)

        return converted_value

    def get_wheel_degree(self):
        axis = ControllerConstants.DIRECTION_AXIS
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_WHEEL_DEGREE, CarConstants.MAX_WHEEL_DEGREE, value)

        return converted_value

    def print_controller_state(self):
        in_speed, converted_speed = self.get_speed()
        in_angle, converted_wheel_angle = self.get_wheel_angle()

        wheel_degree = self.get_wheel_degree()
        speed_degree = self.get_speed_degree()

        self.screen.fill(0)
        self.draw_text("IN Speed: {}".format(in_speed),
                       5, 0, (0, 255, 0))
        self.draw_text("Converted Speed: {}".format(converted_speed),
                       5, 20, (0, 255, 0))
        self.draw_text("Speed Degree: " + str(speed_degree+90),
                       5, 40, (0, 255, 0))
        self.draw_text("IN Angle: {}".format(in_angle),
                       5, 60, (0, 0, 255))
        self.draw_text("Converted Angle: {}".format(converted_wheel_angle),
                       5, 80, (0, 0, 255))
        self.draw_text("Wheel Degree: " + str(wheel_degree+90),
                       5, 100, (0, 0, 255))

        pygame.draw.circle(self.screen, (150, 100, 0), [140, 270], 125)
        pygame.draw.rect(self.screen, (0, 0, 0), [15, 270, 250, 145])

        spd_radar = (140, 270)
        spd_radar_len = 125
        spd_x = spd_radar[0] + math.cos(math.radians(speed_degree)) * spd_radar_len
        spd_y = spd_radar[1] + math.sin(math.radians(speed_degree)) * spd_radar_len
        pygame.draw.line(self.screen, (0, 255, 0), [140, 270], [spd_x, spd_y], 5)

        wd_radar = (140, 270)
        wd_radar_len = 125
        wd_x = wd_radar[0] + math.cos(math.radians(wheel_degree)) * wd_radar_len
        wd_y = wd_radar[1] + math.sin(math.radians(wheel_degree)) * wd_radar_len
        pygame.draw.line(self.screen, (0, 0 ,255), [140, 270], [wd_x, wd_y], 5)

        pygame.display.flip()

    def quit(self):
        pygame.display.quit()


if __name__ == '__main__':
    app = App()

    while True:
        g_keys = pygame.event.get()

        for event in g_keys:
            if (event.type == KEYDOWN and event.key == K_ESCAPE) or (event.type == QUIT):
                app.quit()
                exit(0)
        app.print_controller_state()
