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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import pygame

from Item_ids import *

import CarConstants
import ControllerConstants
from ValueConverter import linear_converter

# pygame setup
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

        self.cv_image = np.zeros((1, 1, 3), np.uint8)
        self.marker_found = False
        self.corners = np.full((0, 0, 0, 0), 1)
        self.ids = []

        self.encoder_pub = rospy.Subscriber("/encoder", Encoder, self.encoder_callback)

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

        if self.encoder is not None:
            encoder_text = myfont.render("Sum Enc = {}".format(self.encoder), False, (0,0,0))
            screen.blit(encoder_text, (260, 410))

    def encoder_callback(self, data):
        self.encoder = sum([data.FL, data.FR, data.BL, data.BR])
        #print(data)


class Marker:
    def __init__(self, id, type):
        self.id = id
        self.uptime = 0
        self.found = False
        self.type = type

    # item aufheben, erinnerung abspielen
    def event(self,pos):
        self.found = True

    def augment(self,pos,markerwidth):
        text = None
        img = None
        # ueberblende mit spezifischem Erinnerungsbild
        if self.found:
            if self.type == "memory":
                img = pygame.image.load(pic_dict[self.id])
            elif self.type == "item":
                img = pygame.image.load("Art/generic_memory.jpg")
        # ueberblende mit allgemeinem Erinnerungsicon
        else:
            if self.type == "memory":
                img = pygame.image.load("Art/generic_memory.jpg")
            elif self.type == "item":
                img = pygame.image.load(pic_dict[self.id])

        if markerwidth > 100:
            xscaling = int(3*16./9)
            yscaling = 3
        else:
            xscaling = yscaling = 1.2

        img = pygame.transform.scale(img, (int(xscaling*markerwidth), int(yscaling*markerwidth)))

        screen.blit(img, pygame.rect.Rect(pos[0]-img.get_width()/2, pos[1]-img.get_height()/2, img.get_width(), img.get_height()))


class Item:
    name = ""
    obtained = False
    id = 0

    def __init__(self, name,id):
        self.name = name
        self.id = id
        self.obatined=False

class Car:
    items_found = []
    battery_capacity = 0
    battery_charge = 0
    def __init__(self,cap = 10000):
        self.battery_capacity = cap
        self.battery_charge = cap

class Game:


    def __init__(self, car):
        self.car = car
        self.markerlist = [Marker(1,"memory"), Marker(2,"memory"), Marker(64,"memory"), Marker(320,"item")]
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
        print("Charge: " + str(self.car.battery_charge))

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
                m.augment(pos_flipped, side1_width)

            if np.abs(side1_width * side2_width) >= 10000:

                # draw id
                for m in ml:
                    if not m.found:
                        m.event(pos_flipped)

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
        #self.draw_text("Converted Speed: {}".format(converted_speed),
        #               5, 20, (255, 255, 255))
        self.draw_text("Steering: {}".format(int(wheel_degree)+90),
                       center_tacho[0] - radius_tacho * 2 - 60, center_tacho[1] - 50, (0, 0, 255))
        #self.draw_text("Converted Angle: {}".format(converted_wheel_angle),
        #               5, 60, (255, 255, 255))

        # publish command for ros
        self.commands_pub.publish(ECU(converted_speed, converted_wheel_angle))

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
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_SPEED_DEGREE, CarConstants.MAX_SPEED_DEGREE, value, invert=True)

        return converted_value

    def get_wheel_degree(self):
        axis = ControllerConstants.DIRECTION_AXIS
        value = self.my_joystick.get_axis(axis)
        converted_value = linear_converter(CarConstants.MIN_WHEEL_DEGREE, CarConstants.MAX_WHEEL_DEGREE, value)

        return converted_value

def main(args):
    rospy.init_node('game_node', anonymous=True)
    car = Car()
    game = Game(car)

    while True:
        game.loop()

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
