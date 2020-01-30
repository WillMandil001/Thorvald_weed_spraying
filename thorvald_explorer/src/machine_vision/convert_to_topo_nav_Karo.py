#!/usr/bin/env python
import tf
import cv2
import tf2_ros
import rospy
import numpy as np
import image_geometry
import copy
import scipy.misc
from matplotlib import pyplot as plt
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

    def canny_hough_rows(self,cv_image):
        # Pre-processing on the input image
        kernel = np.ones((10,10),np.uint8)
        dilation = cv2.dilate(cv_image,kernel,iterations = 1)
        cv2.imshow("Pre-processed image", dilation)
        # Compute canny edges on the processed image
        edges = cv2.Canny(dilation,100,200)
        cv2.imshow("Canny edges", edges)
        # Keep copies of the original images for later processing
        edges_working_copy = copy.deepcopy(edges)

        # Find the best hough lines from the edges
        for i in range(0, 100):
            try:
                lines = cv2.HoughLines(edges_working_copy,1,np.pi/90,90)
                for rho,theta in lines[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 10000*(-b))
                    y1 = int(y0 + 10000*(a))
                    x2 = int(x0 - 10000*(-b))
                    y2 = int(y0 - 10000*(a))
                    # Resulting lines are drawn onto the original image for visualisation
                    cv2.line(cv_image,(x1,y1),(x2,y2),(50,50,100), 5)
                    # And we draw over the 'found' edges in black
                    cv2.line(edges_working_copy,(x1,y1),(x2,y2),(0,0,255), 15)
            except:
                break

        cv2.imshow("Original image with hough lines", cv_image)
        k = cv2.waitKey(0)
        cv2.destroyAllWindows()
        return dilation, edges

    def pca_on_full_image(self, img, img_label):
        # pca performed on the full image (could take dilation or edges as input)
        y, x = np.nonzero(img)
        x = x - np.mean(x)
        y = y - np.mean(y)
        coords = np.vstack([x, y])
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
        x_v2, y_v2 = evecs[:, sort_indices[1]]

        # Plotting
        vector_scale = 20 # Only relevant for visualisation purposes
        plt.plot([x_v1*-vector_scale*2, x_v1*vector_scale*2],
                 [y_v1*-vector_scale*2, y_v1*vector_scale*2], color='red')
        plt.plot([x_v2*-vector_scale, x_v2*vector_scale],
                 [y_v2*-vector_scale, y_v2*vector_scale], color='blue')
        plt.plot(x, y, 'k.')
        plt.axis('equal')
        plt.gca().invert_yaxis()  # Match the image system with origin at top left
        plt.title("PCA on full image '%s'" % img_label)
        plt.show()

    def pca_on_kernels(self, img, img_label):
        # PCA performed on smaller sections of images
        pca_img = copy.deepcopy(img)

        imgx_l = img.shape[0]
        imgy_h = img.shape[1]
        box_size = 80
        prev_point_l = 0
        prev_point_h = 0
        # iterate kernel over the image
        for i in range(0, imgx_l, box_size):
            for j in range(0, imgy_h, box_size):
                image_square = img[i:(i+box_size), j:(j+box_size)]
                cv2.rectangle(pca_img, (j,i), (j+box_size, i+box_size), (50,50,100), 1)
                # find principal components in kernel
                y, x = np.nonzero(image_square)
                if y.size > 0:
                    # If there are any nonzero pixels, find principal component
                    x = x - np.mean(x)
                    y = y - np.mean(y)
                    coords = np.vstack([x, y])
                    cov = np.cov(coords)
                    try:
                        evals, evecs = np.linalg.eig(cov)
                        sort_indices = np.argsort(evals)[::-1]
                        x_v1, y_v1 = evecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
                        x_v2, y_v2 = evecs[:, sort_indices[1]]
                        # TODO: Rotate the vectors to point in positive direction...?

                        square_centre = (j + box_size/2, i + box_size/2)
                        vector_scale = 50 # Only relevant for visualisation purposes

                        projection_pca1 = (int(square_centre[0] + (vector_scale*x_v1)), int(square_centre[1] + (vector_scale*y_v1)))
                        projection_pca2 = (int(square_centre[0] + (vector_scale*x_v2)), int(square_centre[1] + vector_scale*y_v2))
                        cv2.line(pca_img, square_centre, projection_pca1, (50,50,100), 5)
                        cv2.line(pca_img, square_centre, projection_pca2, (50,50,100), 5)
                    except:
                        pass
        title = "PCA on smaller kernels across image '{}'"
        cv2.imshow(title.format(img_label), pca_img)
        k = cv2.waitKey(0)
        if k ==27:
            pass
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    no_soil_image = convert.remove_soil(raw_image)
    dilation, edges = convert.canny_hough_rows(no_soil_image)
    convert.pca_on_full_image(dilation, 'dilation')
    convert.pca_on_full_image(edges, 'edges')
    convert.pca_on_kernels(dilation, 'dilation')
    convert.pca_on_kernels(edges, 'edges')
