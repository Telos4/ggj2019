import rospy
import pygame

import cv2
import numpy as np

from ros_comm import ROS_communicator
from barc.msg import ECU

import CarConstants
import ControllerConstants
from ValueConverter import linear_converter

from Item_ids import *
from marker import Marker
import rfid
from datetime import datetime,timedelta

import time

def two_norm(dx_vec,dy_vec):
    return np.sqrt(dx_vec**2 + dy_vec**2)

class Game:
    def __init__(self, car):
        pygame.mixer.pre_init()
        pygame.mixer.init()
        pygame.init()
        pygame.font.init()
        pygame.joystick.init()
        pygame.display.set_caption("ROS camera stream on Pygame")

        background_sound = pygame.mixer.Sound("Music/background.ogg")
        pygame.mixer.Channel(1).play(background_sound, loops=-1)

        self.myfont = pygame.font.SysFont('Comic Sans MS', 30)
        self.screenheight = 800
        self.screenwidth = 4 * self.screenheight // 3
        self.screen = pygame.display.set_mode([self.screenwidth, self.screenheight])
        red = (255, 0, 0)
        teal = (0, 255, 255)


        self.overlay = pygame.image.load("Art/Overlay_neu.png")
        self.overlay = pygame.transform.scale(self.overlay, (self.screenwidth, self.screenheight) )
        self.car = car
        self.markerlist = []
        self.current_rfid_events = []
        for i in type_list:
            self.markerlist.append(Marker(i[0],i[1], self.screen))
        #self.markerlist = [Marker(1,"memory"), Marker(2,"memory"), Marker(64,"memory"), Marker(320,"item")]
        self.ic = ROS_communicator((self.screenwidth, self.screenheight))
        time.sleep(0.5) # wait for hardware
        self.travel_dist = self.ic.encoder

        # TODO maybe move to ros_comm
        self.commands_pub = rospy.Publisher("gamepad_commands", ECU)

        try:
            self.my_joystick = pygame.joystick.Joystick(0)
            self.my_joystick.init()
        except pygame.error:
            print("warning: no gamepad found!")

        img = pygame.image.load("Art/Titelbild.png")
        img = pygame.transform.scale(img, (self.screenwidth,self.screenheight))
        self.screen.blit(img,(0,0))
        myfontbig = pygame.font.SysFont('Comic Sans MS', 80)
        self.draw_text("Press throttle to begin",int(self.screenwidth/2)-400,int(self.screenheight/2)+10, (255, 0, 0),myfontbig)

        pygame.display.update()
        while abs(self.get_speed()[1]-1500) <= 20:
            pygame.event.get()
        self.game_start_time = datetime.now()

        self.debug_output = False

    def colorize(self, image, newColor):
        """
        Colorize image with the specified color
        """
        image = image.copy()
        image.fill(newColor[0:3] + (0,), None, pygame.BLEND_RGBA_ADD)

        return image

    def loop(self):
        # get recent image
        cv_image = self.ic.cv_image
        marker_found = self.ic.marker_found
        corners = self.ic.corners
        ids = self.ic.ids
        time_passed = (datetime.now()-self.game_start_time)
        h = 1.
        # jetzt < start + 5 sek
        if time_passed.total_seconds() < 5:
            h = time_passed.total_seconds()/5*np.sin(2.5/5*np.pi*time_passed.total_seconds())**2
            pygame.draw.rect(self.screen, (0, 0, 0), [0, 0, self.screenwidth, int((1 -h)/2*self.screenheight)])
            pygame.draw.rect(self.screen, (0, 0, 0), [0, int((1+h)/2*self.screenheight), self.screenwidth, self.screenheight])
            pygame.display.flip()

        # output of camera image in pygame screen
        self.screen.fill([0, 0, 0])

        # read pygame events ( = gamepad input )
        g_keys = pygame.event.get()

        #get traveled distance and handle charge
        delta_dist = abs(self.travel_dist - self.ic.encoder)
        self.travel_dist = self.ic.encoder
        self.car.battery_charge -= delta_dist
        if self.car.battery_charge < 0:
            print (" no charge left! ")
            self.commands_pub.publish(ECU(1500, 1500))
            gameover_img =pygame.image.load("Art/Game_Over_Screen.png")
            gameover_img = pygame.transform.scale(gameover_img,(self.screenwidth,self.screenheight))
            self.screen.blit(gameover_img, (0,0))
            pygame.display.update()
            time.sleep(5)
            pygame.quit()
            quit()

        # output of camera image in pygame screen
        self.screen.fill([0, 0, 0])
        # frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # (interesting colors)
        #oldheight, oldwidth, dum = cv_image.shape
        #cv_image = cv2.resize(cv_image,(screenwidth,screenheight))
        #newheight, newwidth, dum2 = cv_image.shape

        if self.emp_event_active():
            frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)

        if not self.emp_event_active():
            # color filter for Mars feeling
            frame = self.colorize(frame, (128, 0, 0))

        self.screen.blit(frame, (0, 0))


        #self.screen.blit(self.overlay , (0,0))

        overlay = pygame.image.load("Art/Overlay_neu.png")
        overlay = pygame.transform.scale(overlay, (self.screenwidth, self.screenheight) )

        self.screen.blit(overlay , (0,0))

        #print all collected items
        posy = self.screenheight - 100
        posx = 50
        for it in self.car.found:
            if it.type == "item":
                img = it.img_small
                x,y = img.get_rect().size
                self.screen.blit(it.img_small, (posx,posy))
                posx += x

        if marker_found:
            pos = np.mean(corners[0][0], axis=0)

            side1_width = two_norm(corners[0][0][0][0] - corners[0][0][1][0], corners[0][0][0][1] - corners[0][0][1][1])
            side2_width = two_norm(corners[0][0][1][0] - corners[0][0][2][0], corners[0][0][1][1] - corners[0][0][2][1])
            pos_flipped = (cv_image.shape[1] - int(pos[0]), int(pos[1]))

            ml=[mar for mar in self.markerlist if mar.id == ids[0]]
            for m in ml:
                m.augment(pos_flipped, side1_width,self.car)

            if np.abs(side1_width * side2_width) >= 10000:

                # draw id
                for m in ml:
                    if (not m.found):
                        self.car.event_handler(m ,pos_flipped)

        # handle controller input
        in_speed, converted_speed = self.get_speed()
        in_angle, converted_wheel_angle = self.get_wheel_angle()

        wheel_degree = self.get_wheel_degree()
        speed_degree = self.get_speed_degree()

        if self.debug_output:
            radius_tacho = 125
            center_tacho = [1066-radius_tacho, 800]
            pygame.draw.circle(self.screen, (150, 200, 0), center_tacho, radius_tacho)
            #pygame.draw.rect(screen, (0, 0, 0), [15, 270, 250, 145])

            spd_radar = center_tacho
            spd_radar_len = radius_tacho
            spd_x = spd_radar[0] + np.cos(np.radians(speed_degree)) * spd_radar_len
            spd_y = spd_radar[1] + np.sin(np.radians(speed_degree)) * spd_radar_len
            pygame.draw.line(self.screen, (255, 0, 0), center_tacho, [spd_x, spd_y], 5)

            wd_radar = center_tacho
            wd_radar_len = radius_tacho
            wd_x = wd_radar[0] + np.cos(np.radians(wheel_degree)) * wd_radar_len
            wd_y = wd_radar[1] + np.sin(np.radians(wheel_degree)) * wd_radar_len
            pygame.draw.line(self.screen, (0, 0 ,255), center_tacho, [wd_x, wd_y], 5)

            self.draw_text("Speed: {}".format(-int(in_speed * 100)),
                          center_tacho[0]- 75, center_tacho[1] - radius_tacho * 1.3, (255, 0, 0))
            self.draw_text("Debug Speed: {}".format(int(converted_speed)), center_tacho[0]- 75, center_tacho[1] - radius_tacho * 1.3 -25, (255, 0, 0))
            self.draw_text("Steering: {}".format(int(wheel_degree)+90),
                          center_tacho[0] - radius_tacho * 2 - 60, center_tacho[1] - 50, (0, 0, 255))


        battery_position = [800, 35]
        battery_size = [int(self.car.battery_capacity/100)+20, 70]
        battery_rect = (battery_position[0], battery_position[1], battery_size[0], battery_size[1])

        if self.car.battery_charge < self.car.battery_capacity * 0.25:
            battery_color = (255, 0, 0)
        elif self.car.battery_charge < self.car.battery_capacity * 0.5:
            battery_color = (255, 255, 0)
        else:
            battery_color = (0, 255, 0)

        nub_size = [15, int(battery_size[1] / 2)]
        nub_rect = (battery_position[0] + battery_size[0], battery_position[1] + int(battery_size[1] / 4), nub_size[0], nub_size[1])

        charge_position = [battery_position[0] + 10, battery_position[1] + 10]
        charge_size = [int(self.car.battery_charge/100), battery_size[1]-20]
        charge_rect = (charge_position[0], charge_position[1], charge_size[0], charge_size[1])

        pygame.draw.rect(self.screen, (255, 255, 255), battery_rect, 1)
        pygame.draw.rect(self.screen, (255, 255, 255), nub_rect)
        pygame.draw.rect(self.screen, battery_color, charge_rect)

        # publish command for ros
        self.commands_pub.publish(ECU(converted_speed, converted_wheel_angle))

        if self.debug_output:
            # demo how to use sensor values
            if self.ic.encoder is not None:
                encoder_text = self.myfont.render("Sum Enc = {}".format(self.ic.encoder), False, (0,0,0))
                self.screen.blit(encoder_text, (260, 410))

            if self.ic.echo is not None:
                echo_text = self.myfont.render("Echo = {}".format(self.ic.echo), False, (0, 0, 0))
                self.screen.blit(echo_text, (260, 500))

            if self.ic.light is not None:
                light_text = self.myfont.render("Light = {}".format(self.ic.light), False, (0, 0, 0))
                self.screen.blit(light_text, (260, 600))
                if self.car.charging_possible and self.ic.light > CarConstants.Light_threshold:
                    self.car.battery_charge = min (self.car.battery_charge + 10,self.car.battery_capacity)
        if self.ic.rfid is not None:
            if self.debug_output:
                rfid_text = self.myfont.render("RFID = {}".format(self.ic.rfid), False, (0, 0, 0))
                self.screen.blit(rfid_text, (260, 700))
            if self.ic.rfid > 0:
                if self.ic.rfid == rfid.sandstorm_id:
                    rfide = rfid.RfidEvent(self.ic.rfid,duration=5,img="Art/Sandstorm.png",screen = self.screen)
                    self.current_rfid_events.append(rfide)
                    rfide.start()
                elif self.ic.rfid == rfid.emp_id:
                    rfide = rfid.RfidEvent(self.ic.rfid, duration=5, img=None, screen=None)
                    self.current_rfid_events.append(rfide)
                    rfide.start()
                else:
                    print("unrecognized rfid")
        t = datetime.now()
        for r in self.current_rfid_events:
            if t > r.start_time + r.time_duration:
                r.stop()
                self.current_rfid_events.remove(r)
            else:
                r.event()

        pygame.display.update()

    def emp_event_active(self):
        # check if an EMP is currently active
        for event in self.current_rfid_events:
            if event.emp_active:
                return True
        return False

    def draw_text(self, text, x, y, color,  font = None, align_right=False,):
        if font is None:
            font = self.myfont
        surface = font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey((0, 0, 0))

        self.screen.blit(surface, (x, y))

    def get_speed(self):
        axis = ControllerConstants.SPEED_AXIS
        try:
            value = self.my_joystick.get_axis(axis)
        except AttributeError:
            #print("warning: no gamepad found!")
            value = 0
        converted_value = linear_converter(CarConstants.MIN_SPEED, CarConstants.MAX_SPEED, value, invert=True)

        return value, converted_value

    def get_wheel_angle(self):
        axis = ControllerConstants.DIRECTION_AXIS
        try:
            value = self.my_joystick.get_axis(axis)
        except AttributeError:
            #print("warning: no gamepad found!")
            value = 0
        converted_value = linear_converter(CarConstants.MIN_WHEEL_ANGLE, CarConstants.MAX_WHEEL_ANGLE, value)

        return value, converted_value

    def get_speed_degree(self):
        axis = ControllerConstants.SPEED_AXIS
        try:
            value = self.my_joystick.get_axis(axis)
        except AttributeError:
            # print("warning: no gamepad found!")
            value = 0
        converted_value = linear_converter(CarConstants.MIN_SPEED_DEGREE, CarConstants.MAX_SPEED_DEGREE, value, invert=True)

        return converted_value

    def get_wheel_degree(self):
        axis = ControllerConstants.DIRECTION_AXIS
        try:
            value = self.my_joystick.get_axis(axis)
        except AttributeError:
            # print("warning: no gamepad found!")
            value = 0
        converted_value = linear_converter(CarConstants.MIN_WHEEL_DEGREE, CarConstants.MAX_WHEEL_DEGREE, value)

        return converted_value
