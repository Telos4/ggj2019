import pygame

from datetime import datetime,timedelta
import random
import CarConstants

class RfidEvent:
    def __init__(self,id,duration,img,screen):
        self.id = id
        self.triggered = True
        self.start_time = datetime.now()
        self.time_duration = timedelta(seconds=duration)
        self.img = pygame.image.load(img)
        self.screen=screen
    def start(self):
        if self.id == 224:
            CarConstants.MAX_SPEED*=CarConstants.RFID_SLOWDOWN
            CarConstants.MIN_SPEED*=(1-CarConstants.RFID_SLOWDOWN)
    def stop(self):
        if self.id == 224:
            CarConstants.MAX_SPEED/=CarConstants.RFID_SLOWDOWN
            CarConstants.MIN_SPEED/=(1-CarConstants.RFID_SLOWDOWN)
    def event(self):
        if self.id == 224:
            self.img = pygame.transform.scale(self.img, (int(1.5*self.screen.get_width()), int(1.5*self.screen.get_height())))
            randomx = random.randint(-20,20)
            randomy = random.randint(-20,20)
            self.screen.blit(self.img, (-20+randomx, -20+randomy))
