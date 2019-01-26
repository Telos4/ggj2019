from Item_ids import *

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

        mem.event(pos)