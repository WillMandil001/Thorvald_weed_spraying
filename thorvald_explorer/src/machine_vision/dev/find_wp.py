#!/usr/bin/env python
import tf
import cv2
import tf2_ros
import rospy
import numpy as np
import image_geometry
import copy
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import bresenham

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

    def dilate(self, img):
        kernel = np.ones((10,10),np.uint8)
        dilation = cv2.dilate(img,kernel,iterations = 1)
        return dilation

    def find_lines_at_different_granularity(self, img):
        acc = np.zeros(img.shape) # initialise empty image to visualise the results

        box_sizes = [1000,500,120,100,80,70,60,50,40,30] # the levels of granilarity in pixels
        row_height = img.shape[0]
        column_length = img.shape[1]

        for size in box_sizes:
            # TODO: The threshold can be tuned and could also be adjusted to each granularity level
            threshold = 30 # number of 'votes' required for hough line detection
            for i in range(0, row_height, size): # iterate the window across the image
                for j in range(0, column_length, size):
                    corners1 = (i, j) # top left corners of squares, in row and column format
                    corners2 = (i+size//2, j+size//2) # a second square of the same size, shifted by half length down and right (overlapping grid)
                    square1 = img[corners1[0]:min(row_height-1, corners1[0]+size), corners1[1]:min(column_length-1, corners1[1]+size)]
                    square2 = img[corners2[0]:min(row_height-1, corners2[0]+size), corners2[1]:min(column_length-1, corners2[1]+size)]

                    legal_squares = [s for s in [square1,square2] if np.product(s.shape) > 0] # in case square 1 is beyond image dimensions, it is dropped
                    legal_corners = [corners1,corners2] if len(legal_squares) > 1 else [corners1]
                    for square,corner in zip(legal_squares,legal_corners):

                        if np.nonzero(square)[0].size > 0:
                            inf_lines, segments = self.find_straight_lines(square, threshold)
                            if inf_lines is not None:
                                for line in inf_lines:
                                    for rho,theta in line:
                                        a = np.cos(theta)
                                        b = np.sin(theta)
                                        x0 = a*rho
                                        y0 = b*rho
                                        x1 = int(x0 + size*2*(-b))# coordinates in cartesian
                                        y1 = int(y0 + size*2*(a))
                                        x2 = int(x0 - size*2*(-b))
                                        y2 = int(y0 - size*2*(a))

                                        line_endpoint = self.calculate_line_endpoints(x1,y1,x2,y2,size)
                                        # Resulting lines are added to the accumulation image
                                        # All pixels along that line are increased
                                        pixels = bresenham.bresenham(min(corner[1]+line_endpoint[0][0],column_length-1), min(corner[0]+line_endpoint[0][1], row_height-1), min(corner[1]+line_endpoint[1][0],column_length-1) , min(corner[0]+line_endpoint[1][1], row_height-1))
                                        for x,y in pixels:
                                            acc[y,x] += 1
        plt.imshow(acc)
        plt.show()
        return acc

    def find_straight_lines(self, img, threshold):
        edges = cv2.Canny(img,100,200)
        # Find hough lines from the edges
        inf_lines = cv2.HoughLines(edges,1,np.pi/90,threshold)
        line_segments = cv2.HoughLines(edges,1,np.pi/90,90,10,50)
        return inf_lines, line_segments

    def calculate_line_endpoints(self,x1,y1,x2,y2,size):
        inter_pixels = np.array(list(bresenham.bresenham(x1,y1,x2,y2)))
        loc1 =  inter_pixels[np.argwhere(inter_pixels[:,0] == 0),:]
        loc2 =  inter_pixels[np.argwhere(inter_pixels[:,1] == 0),:]
        loc3 =  inter_pixels[np.argwhere(inter_pixels[:,0] == size),:]
        loc4 =  inter_pixels[np.argwhere(inter_pixels[:,1] == size),:]
        locations = [(loc[0,0,0],loc[0,0,1]) for loc in [loc1,loc2,loc3,loc4] if loc.size > 0]
        endpoints = list(set(locations))
        endpoints = [point for point in endpoints if point[0] >= 0]
        endpoints = [point for point in endpoints if point[1] >= 0]
        endpoints = [point for point in endpoints if point[0] <= size]
        endpoints = [point for point in endpoints if point[1] <= size]
        cleaned_endpoints = sorted(endpoints)
        return cleaned_endpoints

if __name__ == '__main__':
    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    no_soil_image = convert.remove_soil(raw_image)
    dilation = convert.dilate(no_soil_image)

    acc = convert.find_lines_at_different_granularity(dilation)
