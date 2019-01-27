import pygame

from datetime import datetime
from Item_ids import *

class Marker:
    def __init__(self, id, type, screen):
        self.id = id
        self.uptime = 0
        self.found = False
        self.type = type
        self.screen = screen
        if self.type == "memory":
            self.sound_length = None

    # item aufheben, erinnerung abspielen
    def event(self,pos):
        self.found = True
        self.time_found = datetime.now()

    def augment(self,pos,markerwidth,car):
        text = None
        img = None
        # ueberblende mit spezifischem Erinnerungsbild
        if self.found:
            if self.type == "memory":
                transition_time = music_transition_dict[self.id]
                delta = (datetime.now()-self.time_found).seconds
                dummy = 1. * (delta % self.sound_length) / self.sound_length
                index = 0 if dummy < 1. * music_transition_dict[self.id] / self.sound_length else 1
                img = pygame.image.load(pic_dict[self.id][index])
            elif self.type == "item":
                img = pygame.image.load("Art/generic_item_small.png")
        # ueberblende mit allgemeinem Erinnerungsicon
        else:
            if self.type == "memory":
                img = pygame.image.load("Art/generic_memory.png")
            elif self.type == "item":
                img = pygame.image.load(pic_dict[self.id])
            elif self.type == "home":
                if len(car.found) == len(item_dict) and markerwidth > 100:
                    img = pygame.image.load("Art/home_won.png")
                else:
                    img = pygame.image.load("Art/home_not_won.png")

        if markerwidth > 100:
            x,y = img.get_rect().size
            xscaling = int(4*x/y)
            yscaling = 4
            if self.type == "item":
                img = pygame.image.load(pic_dict[self.id])

        else:
            xscaling = yscaling = 1.5

        img = pygame.transform.scale(img, (int(xscaling*markerwidth), int(yscaling*markerwidth)))

        self.screen.blit(img, pygame.rect.Rect(pos[0]-img.get_width()/2, pos[1]-img.get_height()/2, img.get_width(), img.get_height()))
