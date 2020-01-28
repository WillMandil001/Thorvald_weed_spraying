#!/usr/bin/env python
import tf
import cv2
import math
import tf2_ros
import rospy
import numpy as np
import image_geometry
from matplotlib import pyplot as plt

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker


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
        imgx_l = cv_image.shape[1]
        imgy_h = cv_image.shape[0]
        box_size = 200
        prev_point_l = 0
        prev_point_h = 0
        for i in range(0, imgx_l, box_size):
            for j in range(0, imgy_h, box_size):
                # print("i = ", i, "j = ", j)
                image_square = cv_image[i:(i+box_size), j:(j+box_size)]
                # 1. Find center of image.
                imgx = (image_square.shape[1] / 2)
                imgy = (image_square.shape[0] / 2)

                # 2. Calculate lines for each angle (0, 179) degrees and do for 
                histogram = []
                angles = []
                for angle in range(0, 180, 10):
                    histo = 0
                    angle_rad = (angle * math.pi) / 180
                    r = 100  # R = LENGTH OF LINE DRAWN. JUST NEEDS TO BE LONGER THAN THE EDGES OF THE image_square
                    for a in range(-r, r + 1, max(1, int((r * 2)))):
                        xcirc = imgx + a * math.cos(angle_rad)
                        ycirc = imgy + a * math.sin(angle_rad)
                        points = self.createLineIterator((int(imgx), int(imgy)), (int(xcirc), int(ycirc)), image_square)
                        histo += sum(points[:, 2])
                    histogram.append(histo)
                    angles.append(angle)

                plt.bar(angles, histogram, align='center', alpha=0.5)
                plt.xticks(angles, histogram)
                plt.ylabel('Usage')
                plt.title('Programming language usage')
                plt.show()

                cv2.imshow("image_square", image_square)
                k = cv2.waitKey(0)
                if k ==27:
                    break
                cv2.destroyAllWindows()
                # 3. buld histogram from sum of white pixels in line.


    def createLineIterator(self, P1, P2, image_square):
       imageH = image_square.shape[0]
       imageW = image_square.shape[1]
       P1X = P1[0]
       P1Y = P1[1]
       P2X = P2[0]
       P2Y = P2[1]

       #difference and absolute difference between points
       #used to calculate slope and relative location between points
       dX = P2X - P1X
       dY = P2Y - P1Y
       dXa = np.abs(dX)
       dYa = np.abs(dY)

       #predefine numpy array for output based on distance between points
       itbuffer = np.empty(shape=(np.maximum(dYa,dXa),3),dtype=np.float32)
       itbuffer.fill(np.nan)

       #Obtain coordinates along the line using a form of Bresenham's algorithm
       negY = P1Y > P2Y
       negX = P1X > P2X
       if P1X == P2X: #vertical line segment
           itbuffer[:,0] = P1X
           if negY:
               itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
           else:
               itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)              
       elif P1Y == P2Y: #horizontal line segment
           itbuffer[:,1] = P1Y
           if negX:
               itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
           else:
               itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
       else: #diagonal line segment
           steepSlope = dYa > dXa
           if steepSlope:
               slope = dX/dY
               if negY:
                   itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
               else:
                   itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
               itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int) + P1X
           else:
               slope = dY/dX
               if negX:
                   itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
               else:
                   itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
               itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int) + P1Y

       #Remove points outside of image
       colX = itbuffer[:,0]
       colY = itbuffer[:,1]
       itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]

       #Get intensities from image_square ndarray
       itbuffer[:,2] = image_square[itbuffer[:,1].astype(np.uint),itbuffer[:,0].astype(np.uint)]

       return itbuffer

if __name__ == '__main__':
    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    no_soil_image = convert.remove_soil(raw_image)
    convert.calculate_crop_rows(no_soil_image)


