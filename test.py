import pygame
import time

pygame.mixer.pre_init(44100,16,2,4096)
pygame.mixer.init()
pygame.init()

screenheight = 800
screenwidth = 4 * screenheight // 3
screen = pygame.display.set_mode([screenwidth, screenheight])
screen.fill([0,0,0])

overlay = pygame.image.load("Art/Overlay.png")
overlay = pygame.transform.scale(overlay, (screenwidth, screenheight) )

screen.blit(overlay , (0,0))
pygame.display.update()

img = pygame.image.load("Art/generic_item_small.png")
print(img.get_rect().size)
time.sleep(5)

