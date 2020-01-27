#!/usr/bin/env python
import tf
import cv2
import math
import tf2_ros
import rospy
import numpy as np
import image_geometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker

import math


########################################################################################################################

#  TODO:
# 1. need to divide the image into multiple squares.
# 2. get pixel values instead of draw line :D
# 3. then make histogram from the pixel values 
########################################################################################################################

class convert_to_topo_nav():
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2listener = tf2_ros.TransformListener(self.tf2Buffer)
        self.br = tf.TransformBroadcaster()
        self.camera_info = rospy.wait_for_message('/thorvald_002/kinect2_camera/hd/camera_info', CameraInfo)
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

    def import_image(self):
        image = rospy.wait_for_message("/thorvald_002/kinect2_camera/hd/image_color_rect", Image)
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        return cv_image

    def remove_soil(self, cv_image):  # ran once to segment the weeds and locate them
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        lower_green = np.array([20,0,0])
        upper_green = np.array([255,255,255])
        mask = cv2.inRange(hsv_img, lower_green, upper_green)

        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return opening

    def calculate_crop_rows(self, cv_image):
        # 1. divide image up into set of square images for histogram analysis on each.
        # crop_img = img[y:y+h, x:x+w]  # USE THIS TO CROP IMAGES INTO HOWEVER MANY WE WANT

        # 1. Find center of image.
        imgx = (cv_image.shape[1] / 2)
        imgy = (cv_image.shape[0] / 2)

        # 2. Calculate lines for each angle (0, 179) degrees and do for 
        for angle in range(0, 180):
            angle_rad = (angle * math.pi) / 180
            r = 10000  # R = LENGTH OF LINE DRAWN. JUST NEEDS TO BE LONGER THAN THE EDGES OF THE IMAGE
            for a in range(-r, r + 1, max(1, int((r * 2)))):
                xcirc = imgx + a * math.cos(angle_rad)
                ycirc = imgy + a * math.sin(angle_rad)
                cv2.line(cv_image, (int(imgx), int(imgy)), (int(xcirc), int(ycirc)), (100, 100, 100), thickness=1, lineType=8)

        # 3. buld histogram from sum of white pixels in line.

        cv2.imshow("cv_image", cv_image)
        k = cv2.waitKey(0)
        if k ==27:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    no_soil_image = convert.remove_soil(raw_image)
    convert.calculate_crop_rows(no_soil_image)


