# see: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# !/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
#from barc.msg import Encoder
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pygame

# pygame setup
pygame.init()
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)
pygame.display.set_caption("ROS camera stream on Pygame")
screen = pygame.display.set_mode([1280, 720])
red = (255, 0, 0)
teal = (0, 255, 255)

# ros setup
# camera_stream = "/cv_camera/image_raw"
camera_stream = "/image_raw"
compression = True

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("pygame_image", Image)

        self.bridge = CvBridge()

        if compression:
            self.image_sub = rospy.Subscriber(camera_stream + "/compressed", CompressedImage, self.callback)
        else:
            self.image_sub = rospy.Subscriber(camera_stream, Image, self.callback)

        self.encoder = None
        #self.encoder_pub = rospy.Subscriber("/encoder", Encoder, self.encoder_callback)

    def callback(self, data):
        try:
            if compression:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = np.fliplr(cv_image) # why is this necessary?

        # marker detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        marker_found = len(corners) > 0

        # output of camera image in pygame screen
        screen.fill([0, 0, 0])
        #frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # (interesting colors)
        frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)
        pygame.draw.rect(frame, (0, 0, 255), (0, 0, 100, 200))
        pygame.draw.rect(frame, (255, 0, 0), (250, 400, 400, 100))

        if marker_found:
            # find center of the marker
            pos = np.mean(corners[0][0], axis=0)
            pos = (cv_image.shape[1] - int(pos[0]), int(pos[1]))

            # draw crosshair at marker center
            linewidth = 3
            pygame.draw.circle(frame, red, pos, 100, linewidth)
            pygame.draw.circle(frame, red, pos, 50, linewidth)
            pygame.draw.circle(frame, red, pos, 10, linewidth)
            pygame.draw.line(frame, red, (pos[0] - 150, pos[1]), (pos[0] + 150, pos[1]), linewidth)
            pygame.draw.line(frame, red, (pos[0], pos[1] - 150), (pos[0], pos[1] + 150), linewidth)

            # draw id
            id = ids[0]
            id_text = myfont.render("ID = {}".format(id), False, teal)

            screen.blit(frame, (0, 0))
            screen.blit(id_text, pos)


        else:
            # if no markers are found just draw the camera image
            screen.blit(frame, (0,0))

        if self.encoder is not None:
            encoder_text = myfont.render("Sum Enc = {}".format(self.encoder), False, (0,0,0))
            screen.blit(encoder_text, (260, 410))

        pygame.display.update()
        # try:
        #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #   print(e)

    def encoder_callback(self, data):
        self.encoder = sum([data.FL, data.FR, data.BL, data.BR])
        #print(data)

class item:
    name = ""
    obtained = False
    id = 0
    def __init__(self,name,id):
        self.name = name
        self.id = id
        self.obatined=False;


class car:
    items_found = []
    battery_capacity = 0
    battery_charge = 0
    def __init__(cap = 1000):
        self.battery_capacity = cap
        self.battery_charge = cap



def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
