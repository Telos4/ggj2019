import rospy
import cv2
import cv2.aruco as aruco

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from barc.msg import Encoder
from barc.msg import Light
from barc.msg import Echo
from barc.msg import RFID

import numpy as np

# ros setup
# camera_stream = "/cv_camera/image_raw"
camera_stream = "/image_raw"
compression = True

class ROS_communicator:
    def __init__(self, screen_sizes):
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

        self.screen_sizes = screen_sizes

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

        self.cv_image = cv2.resize(self.cv_image_small,self.screen_sizes)
        # marker detection
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.marker_found = len(self.corners) > 0

    def encoder_callback(self, data):
        self.encoder = 20.0 * sum([data.FL, data.FR, data.BL, data.BR])

    def echo_callback(self, data):
        self.echo = data.distance
        # print(data)

    def light_callback(self, data):
        self.light = data.light
        # print(data)

    def rfid_callback(self, data):
        self.rfid = data.id
