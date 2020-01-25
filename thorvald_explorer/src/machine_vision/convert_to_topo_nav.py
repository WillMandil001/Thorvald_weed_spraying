#!/usr/bin/env python
import tf
import cv2
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
        # cv_image = cv2.resize(cv_image, None, fx=0.2, fy=0.2)  # Resize
        cv2.imshow("cv_image", cv_image)

        kernel = np.ones((10,10),np.uint8)
        dilation = cv2.dilate(cv_image,kernel,iterations = 1)
        cv2.imshow("dilationdilation", dilation)

        minLineLength = 100
        maxLineGap = 100
        lines = cv2.HoughLinesP(dilation,1,np.pi/180,100,minLineLength,maxLineGap)
        for i in range(0, 10):
            for x1,y1,x2,y2 in lines[i]:
                cv2.line(dilation,(x1,y1),(x2,y2),(100,100,10),2)


        # cv2.HoughLines(image, rho, theta, threshold[, lines[, srn[, stn]]])
        # edges = cv2.Canny(gray,50,150,apertureSize = 3)
        for i in range(0, 100):
            try:
                lines = cv2.HoughLines(dilation,1,np.pi/90,300)
                for rho,theta in lines[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 10000*(-b))
                    y1 = int(y0 + 10000*(a))
                    x2 = int(x0 - 10000*(-b))
                    y2 = int(y0 - 10000*(a))
                    cv2.line(cv_image,(x1,y1),(x2,y2),(50,50,100), 5)
                    cv2.line(dilation,(x1,y1),(x2,y2),(0,0,255), 50)
            except:
                break

        cv2.imshow("dilation", dilation)
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


