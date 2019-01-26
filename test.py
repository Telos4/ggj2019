import pygame
import time

pygame.mixer.pre_init(44100,16,2,4096)
pygame.mixer.init()
pygame.init()
pygame.mixer.music.load("Music/Hintergrundmusikfertig.ogg")
pygame.mixer.music.play(-1)
time.sleep(5)

