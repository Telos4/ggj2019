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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pygame

from Item_ids import *

# pygame setup
pygame.init()
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)
pygame.display.set_caption("ROS camera stream on Pygame")
screenheight = 1000
screenwidth = 16*screenheight//9
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

        self.encoder = None

        self.cv_image = np.zeros((1, 1, 3), np.uint8)
        self.marker_found = False
        self.corners = np.full((0, 0, 0, 0), 1)
        self.ids = []

        self.encoder_pub = rospy.Subscriber("/encoder", Encoder, self.encoder_callback)

    def callback(self, data):
        try:
            if compression:
                self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            else:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.cv_image = np.fliplr(self.cv_image) # why is this necessary?

        # self.cv_image = cv2.resize(self.cv_image,(screenwidth,screenheight))
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
    def __init__(self, id):
        self.id = id
        self.uptime = 0
        self.found = False

    # item aufheben, erinnerung abspielen
    def event(self,pos):
        if self.id == Landscape_id:
            self.found = True

    def augment(self,pos,markerwidth):
        text = None
        img = None
        if self.id == Landscape_id:
            # ueberblende mit spezifischem Erinnerungsbild
            if self.found:
                text = myfont.render("hatte ich schon", False, teal)
                img = pygame.image.load(pic_dict[self.id])
                print("hatteich scbon")
            # ueberblende mit allgemeinem Erinnerungsicon
            else:
                text = myfont.render("hatte ich noch nicht", False, teal)
                img = pygame.image.load("Art/generic_memory.jpg")

        screen.blit(text, pos)
        img = pygame.transform.scale(img, (int(markerwidth), int(markerwidth)))
        screen.blit(img, pygame.rect.Rect(pos[0], pos[1], markerwidth, markerwidth))


class Item:
    name = ""
    obtained = False
    id = 0

    def __init__(self, name,id):
        self.name = name
        self.id = id
        self.obatined=False;


class Car:
    items_found = []
    battery_capacity = 0
    battery_charge = 0

    def __init__(self,cap = 1000):
        self.battery_capacity = cap
        self.battery_charge = cap


class Game:

    def __init__(self):
        self.markerlist = [Marker(1), Marker(2), Marker(64)]
        self.ic = ImageConverter()

    def loop(self):
        # get recent image
        cv_image = self.ic.cv_image
        marker_found = self.ic.marker_found
        corners = self.ic.corners
        ids = self.ic.ids

        # output of camera image in pygame screen
        screen.fill([0, 0, 0])

        frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame, (0, 0))

        if marker_found:
            # find center of the marker
            pos = np.mean(corners[0][0], axis=0)

            # [][][cornerid][dim]
            side1_width = two_norm(corners[0][0][0][0] - corners[0][0][1][0], corners[0][0][0][1] - corners[0][0][1][1])
            side2_width = two_norm(corners[0][0][1][0] - corners[0][0][2][0], corners[0][0][1][1] - corners[0][0][2][1])

            ml=[mar for mar in self.markerlist if mar.id == ids[0]]
            for m in ml:
                m.augment(pos, side1_width)

            if np.abs(side1_width * side2_width) >= 10000:
                # pos = (cv_image.shape[1] - int(pos[0]), int(pos[1]))

                # draw id
                for m in ml:
                    if not m.found:
                        m.event(pos)

        pygame.display.update()


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    game = Game()
    while True:
        game.loop()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
