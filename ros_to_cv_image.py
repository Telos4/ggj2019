# see: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# !/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from barc.msg import Encoder
from barc.msg import ECU
from barc.msg import Light
from barc.msg import Echo
from barc.msg import RFID
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import pygame
from datetime import datetime

from Item_ids import *

import CarConstants
import ControllerConstants
from ValueConverter import linear_converter




# pygame setup
pygame.mixer.pre_init()
pygame.mixer.init(frequency=22050,size=-16,channels = 2,buffer=512)
pygame.init()
pygame.font.init()
pygame.joystick.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)
pygame.display.set_caption("ROS camera stream on Pygame")
screenheight = 800
screenwidth = 4*screenheight//3
screen = pygame.display.set_mode([screenwidth, screenheight])
red = (255, 0, 0)
teal = (0, 255, 255)

# ros setup
# camera_stream = "/cv_camera/image_raw"
camera_stream = "/image_raw"
compression = True


def two_norm(dx_vec,dy_vec):
    return np.sqrt(dx_vec**2 + dy_vec**2)


class ImageConverter:
    def __init__(self):
        self.image_pub = rospy.Publisher("pygame_image", Image)

        self.bridge = CvBridge()

        if compression:
            self.image_sub = rospy.Subscriber(camera_stream + "/compressed", CompressedImage, self.callback)
        else:
            self.image_sub = rospy.Subscriber(camera_stream, Image, self.callback)

        self.encoder = 0
        self.echo = 0
        self.light = 0
        self.rfid = 0

        self.cv_image = np.zeros((1, 1, 3), np.uint8)
        self.marker_found = False
        self.corners = np.full((0, 0, 0, 0), 1)
        self.ids = []

        self.encoder_sub = rospy.Subscriber("/encoder", Encoder, self.encoder_callback)
        self.echo_sub = rospy.Subscriber("/echo", Echo, self.echo_callback)
        self.light_sub = rospy.Subscriber("/light", Light, self.light_callback)
        self.rfid_sub = rospy.Subscriber("/rfid", RFID, self.rfid_callback)

    def callback(self, data):
        try:
            if compression:
                self.cv_image_small = self.bridge.compressed_imgmsg_to_cv2(data)
            else:
                self.cv_image_small = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.cv_image_small = np.fliplr(self.cv_image_small) # why is this necessary?

        self.cv_image = cv2.resize(self.cv_image_small,(screenwidth,screenheight))
        # marker detection
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.marker_found = len(self.corners) > 0
        if self.marker_found:
            print(self.ids[0])

    def encoder_callback(self, data):
        self.encoder = sum([data.FL, data.FR, data.BL, data.BR])

    def echo_callback(self, data):
        self.echo = data.distance
        # print(data)

    def light_callback(self, data):
        self.light = data.light
        # print(data)

    def rfid_callback(self, data):
        self.rfid = data.id

class Marker:
    def __init__(self, id, type):
        self.id = id
        self.uptime = 0
        self.found = False
        self.type = type

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
                dt = (datetime.now()-self.time_found).seconds
                dummy = ( dt % (2*time_dict[self.id]))
                index =  dummy / time_dict[self.id]
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
            xscaling = int(4*16./9)
            yscaling = 4
            if self.type == "item":
                img = pygame.image.load(pic_dict[self.id])

        else:
            xscaling = yscaling = 1.5

        img = pygame.transform.scale(img, (int(xscaling*markerwidth), int(yscaling*markerwidth)))

        screen.blit(img, pygame.rect.Rect(pos[0]-img.get_width()/2, pos[1]-img.get_height()/2, img.get_width(), img.get_height()))



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

class Game:
    def __init__(self, car):
        self.car = car
        self.markerlist = []
        for i in type_list:
            self.markerlist.append(Marker(i[0],i[1]))
        #self.markerlist = [Marker(1,"memory"), Marker(2,"memory"), Marker(64,"memory"), Marker(320,"item")]
        self.ic = ImageConverter()
        time.sleep(0.5) # wait for hardware
        self.travel_dist = self.ic.encoder

        self.commands_pub = rospy.Publisher("gamepad_commands", ECU)

        try:
            self.my_joystick = pygame.joystick.Joystick(0)
            self.my_joystick.init()
        except pygame.error:
            print("warning: no gamepad found!")

    def loop(self):
        # get recent image
        cv_image = self.ic.cv_image
        marker_found = self.ic.marker_found
        corners = self.ic.corners
        ids = self.ic.ids

        # output of camera image in pygame screen
        screen.fill([0, 0, 0])

        # read pygame events ( = gamepad input )
        g_keys = pygame.event.get()

        #get traveled distance and handle charge
        delta_dist = abs(self.travel_dist - self.ic.encoder)
        self.travel_dist = self.ic.encoder
        self.car.battery_charge -= delta_dist
        if self.car.battery_charge < 0 :
            print (" no charge left! ")

        # output of camera image in pygame screen
        screen.fill([0, 0, 0])
        # frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # (interesting colors)
        #oldheight, oldwidth, dum = cv_image.shape
        #cv_image = cv2.resize(cv_image,(screenwidth,screenheight))
        #newheight, newwidth, dum2 = cv_image.shape

        frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)

        screen.blit(frame, (0, 0))

        if marker_found:
            # find center of the marker
            #for i in range(4):
            #    corners[0][0][i][0] *= int(newwidth*1. / oldwidth);
            #    corners[0][0][i][1] *= int(newheight*1. / oldheight);

            pos = np.mean(corners[0][0], axis=0)

            # [][][cornerid][dim]
            side1_width = two_norm(corners[0][0][0][0] - corners[0][0][1][0], corners[0][0][0][1] - corners[0][0][1][1])
            side2_width = two_norm(corners[0][0][1][0] - corners[0][0][2][0], corners[0][0][1][1] - corners[0][0][2][1])
            pos_flipped = (cv_image.shape[1] - int(pos[0]), int(pos[1]))

            ml=[mar for mar in self.markerlist if mar.id == ids[0]]
            for m in ml:
                m.augment(pos_flipped, side1_width,self.car)

            if np.abs(side1_width * side2_width) >= 10000:

                # draw id
                for m in ml:
                    if (not m.found) and (m.id != Home_id):
                        self.car.event_handler(m ,pos_flipped)

        # handle controller input
        in_speed, converted_speed = self.get_speed()
        in_angle, converted_wheel_angle = self.get_wheel_angle()

        wheel_degree = self.get_wheel_degree()
        speed_degree = self.get_speed_degree()

        radius_tacho = 125
        center_tacho = [1066-radius_tacho, 800]
        pygame.draw.circle(screen, (150, 200, 0), center_tacho, radius_tacho)
        #pygame.draw.rect(screen, (0, 0, 0), [15, 270, 250, 145])

        spd_radar = center_tacho
        spd_radar_len = radius_tacho
        spd_x = spd_radar[0] + np.cos(np.radians(speed_degree)) * spd_radar_len
        spd_y = spd_radar[1] + np.sin(np.radians(speed_degree)) * spd_radar_len
        pygame.draw.line(screen, (255, 0, 0), center_tacho, [spd_x, spd_y], 5)

        wd_radar = center_tacho
        wd_radar_len = radius_tacho
        wd_x = wd_radar[0] + np.cos(np.radians(wheel_degree)) * wd_radar_len
        wd_y = wd_radar[1] + np.sin(np.radians(wheel_degree)) * wd_radar_len
        pygame.draw.line(screen, (0, 0 ,255), center_tacho, [wd_x, wd_y], 5)

        self.draw_text("Speed: {}".format(-int(in_speed * 100)),
                       center_tacho[0]- 75, center_tacho[1] - radius_tacho * 1.3, (255, 0, 0))
        self.draw_text("Steering: {}".format(int(wheel_degree)+90),
                       center_tacho[0] - radius_tacho * 2 - 60, center_tacho[1] - 50, (0, 0, 255))


        battery_position = [825, 15]
        battery_size = [int(self.car.battery_capacity/100)+20, 70]
        battery_rect = (battery_position[0], battery_position[1], battery_size[0], battery_size[1])

        nub_size = [15, int(battery_size[1] / 2)]
        nub_rect = (battery_position[0] + battery_size[0], battery_position[1] + int(battery_size[1] / 4), nub_size[0], nub_size[1])

        charge_position = [battery_position[0] + 10, battery_position[1] + 10]
        charge_size = [int(self.car.battery_charge/100), battery_size[1]-20]
        charge_rect = (charge_position[0], charge_position[1], charge_size[0], charge_size[1])

        pygame.draw.rect(screen, (255, 255, 255), battery_rect, 1)
        pygame.draw.rect(screen, (255, 255, 255), nub_rect)
        pygame.draw.rect(screen, (0, 255, 0), charge_rect)

        # publish command for ros
        self.commands_pub.publish(ECU(converted_speed, converted_wheel_angle))

        # demo how to use sensor values
        if self.ic.encoder is not None:
            encoder_text = myfont.render("Sum Enc = {}".format(self.ic.encoder), False, (0,0,0))
            screen.blit(encoder_text, (260, 410))

        if self.ic.echo is not None:
            echo_text = myfont.render("Echo = {}".format(self.ic.echo), False, (0, 0, 0))
            screen.blit(echo_text, (260, 500))

        if self.ic.light is not None:
            light_text = myfont.render("Light = {}".format(self.ic.light), False, (0, 0, 0))
            screen.blit(light_text, (260, 600))
            if self.car.charging_possible and self.ic.light > CarConstants.Light_threshold:
                self.car.battery_charge = min (self.car.battery_charge + 1,self.car.battery_capacity)

        if self.ic.rfid is not None:
            rfid_text = myfont.render("RFID = {}".format(self.ic.rfid), False, (0, 0, 0))
            screen.blit(rfid_text, (260, 700))

        pygame.display.update()

    def draw_text(self, text, x, y, color, align_right=False):
        surface = myfont.render(text, True, color, (0, 0, 0))
        surface.set_colorkey((0, 0, 0))

        screen.blit(surface, (x, y))

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

def main(args):
    rospy.init_node('game_node', anonymous=True)
    car = Car()
    game = Game(car)
    backsoundbool = True

    while True:
        if backsoundbool:
            backsoundbool = False
            #background_sound = pygame.mixer.Sound("Music/Hintergrundmusik.wav")
            #pygame.mixer.Channel(1).play(background_sound, loops = -1)


        game.loop()

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
