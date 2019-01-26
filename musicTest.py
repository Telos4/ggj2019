import pygame
from pygame.locals import *
import time


class App:
    def __init__(self):
        pygame.mixer.pre_init()
        pygame.mixer.init()
        pygame.init()

        pygame.display.set_caption("Space Agency")
        self.screen = pygame.display.set_mode((300, 400))
        self.font = pygame.font.SysFont("Courier", 20)

        background_sound = pygame.mixer.Sound("Music/background.ogg")
        pygame.mixer.Channel(1).play(background_sound)

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
        time.sleep(5)
