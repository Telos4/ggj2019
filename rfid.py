import pygame

from datetime import datetime,timedelta
import random
import CarConstants

sandstorm_id = 224
emp_id = 59

class RfidEvent:
    def __init__(self,id,duration,img,screen):
        self.id = id
        self.triggered = True
        self.start_time = datetime.now()
        self.time_duration = timedelta(seconds=duration)
        if img is not None:
            self.img = pygame.image.load(img)
        self.screen=screen
        self.emp_active = False

    def start(self):
        if self.id == sandstorm_id:
            CarConstants.MAX_SPEED*=CarConstants.RFID_SLOWDOWN
            CarConstants.MIN_SPEED*=(1-CarConstants.RFID_SLOWDOWN)
        if self.id == emp_id:
            self.emp_active = True
    def stop(self):
        if self.id == sandstorm_id:
            CarConstants.MAX_SPEED/=CarConstants.RFID_SLOWDOWN
            CarConstants.MIN_SPEED/=(1-CarConstants.RFID_SLOWDOWN)
        if self.id == emp_id:
            self.emp_active = False
    def event(self):
        if self.id == sandstorm_id:
            self.img = pygame.transform.scale(self.img, (int(1.5*self.screen.get_width()), int(1.5*self.screen.get_height())))
            randomx = random.randint(-20,20)
            randomy = random.randint(-20,20)
            self.screen.blit(self.img, (-20+randomx, -20+randomy))
        if self.id == emp_id:
            # nothing to do
            pass
