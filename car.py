from Item_ids import *
import pygame


class Car:
    def __init__(self,cap = 10000):
        self.battery_capacity = cap
        self.battery_charge = cap
        self.charging_possible = False
        self.found = []

    def event_handler(self,mem,pos):
        self.found.append(mem)
        if mem.type == "item":
            if mem.id == Battery_id:
                self.battery_capacity += 10000
                self.battery_charge += 10000
                print("increased capacity")
            if mem.id == Solar_Pan_id:
                self.charging_possible = True
                print("charging now possible")
        elif mem.type == "memory":
            try:
                sound_file = music_dict[mem.id]
            except KeyError:
                sound_file = None
            if sound_file:
                sound_object = pygame.mixer.Sound(sound_file)
                mem.sound_length = sound_object.get_length()
                pygame.mixer.Channel(2).play(sound_object)

        mem.event(pos)
