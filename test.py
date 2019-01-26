import pygame
import time

pygame.mixer.pre_init(44100,16,2,4096)
pygame.mixer.init()
pygame.init()
pygame.mixer.music.load("Music/Hintergrundmusikfertig.ogg")
img = pygame.image.load("Art/generic_item_small.png")
print(img.get_rect().size)
pygame.mixer.music.play()
time.sleep(5)

