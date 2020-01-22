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


class segment_basil_class():
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2listener = tf2_ros.TransformListener(self.tf2Buffer)
        self.image_pub = rospy.Publisher("only_weeds_image",Image, queue_size=10)
        self.pose_pub = rospy.Publisher('weed_to_kill', PoseStamped, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.camera_info = rospy.wait_for_message('/thorvald_001/kinect2_camera/hd/camera_info', CameraInfo)
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

    def segment_once(self):  # ran once to segment the weeds and locate them
        image = rospy.wait_for_message("/thorvald_001/kinect2_camera/hd/image_color_rect", Image)
        self.last_ts = image.header.stamp
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        im_weed_and_soil = self.remove_crop(cv_image)  # Remove the crop from the image.
        im_weeds = self.remove_soil(cv_image, im_weed_and_soil)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_weeds, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def publish_weed_array(self, weed_pose_array):
        # print("weed Basil")
        for weed in weed_pose_array:
            try:
                uv = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint(weed))
                single_weed = PoseStamped()
                single_weed.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame" 

                single_weed.pose.position.x = (uv[0] * 0.493)
                single_weed.pose.position.y = (uv[1] * 0.493)
                single_weed.pose.position.z = 0.493

                single_weed.pose.orientation.x = 0
                single_weed.pose.orientation.y = 0
                single_weed.pose.orientation.z = 0
                single_weed.pose.orientation.w = 1

                # the following code transforms the weed centroids to the /map frame at the time step when the image was taken:
                trans = self.tf2Buffer.lookup_transform("map", "thorvald_001/kinect2_rgb_optical_frame", self.last_ts)
                single_weed_correct = PoseStamped()
                single_weed_correct.header.frame_id = "map"

                # A. t1 * rot_matrix:
                t1 = np.array([single_weed.pose.position.x,
                                single_weed.pose.position.y,
                                single_weed.pose.position.z])
                theta = tf.transformations.euler_from_quaternion([trans.transform.rotation.x, 
                                                trans.transform.rotation.y, 
                                                trans.transform.rotation.z, 
                                                trans.transform.rotation.w])
                yaw = theta[2]
                rotation_matrix = np.array([[(math.cos(yaw)), -(math.sin(yaw)), 0],
                                    [(math.sin(yaw)), (-math.cos(yaw)), 0],
                                    [0, 0, 1]])
                t1_A = np.matmul(t1, rotation_matrix)

                # B. t * output_of_a
                t_x = trans.transform.translation.x + t1_A[0]
                t_y = trans.transform.translation.y + t1_A[1]
                t_z = 0

                single_weed_correct.pose.position.x = t_x 
                single_weed_correct.pose.position.y = t_y
                single_weed_correct.pose.position.z = t_z

                single_weed_correct.pose.orientation.x = 0
                single_weed_correct.pose.orientation.y = 0
                single_weed_correct.pose.orientation.z = 0
                single_weed_correct.pose.orientation.w = 1

                self.pose_pub.publish(single_weed_correct)
            except:
                print("failed")


    def object_removal(self, image, kernal_a, kernal_b, min_size):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernal_a, kernal_b))  # remove small objects (this step will speed up the connected regions bit)
        img1 = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img1, connectivity=8) # remove larger objects still remaining
        sizes = stats[1:, -1]
        nb_components = nb_components - 1
        img2 = np.zeros((output.shape))
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                img2[output == i + 1] = 255
        return img2

    def remove_crop(self, cv_image):
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        lower_green = np.array([20,130,0])
        upper_green = np.array([255,255,255])
        mask = cv2.inRange(hsv_img, lower_green, upper_green)
        weeds_and_soil = self.object_removal(image=mask, kernal_a = 5, kernal_b = 5, min_size=1000) # SET TO min_size= 50 TO INCLUDE SAPLINGS
        return weeds_and_soil

    def remove_soil(self, cv_image, image_mask):
        weed_pose_array = []
        image_mask = (255 - image_mask)
        image_mask = image_mask.astype(np.uint8)
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=image_mask)

        hsv_img = cv2.cvtColor(result_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, np.array([25,0,0]), np.array([255,255,255]))
        weeds_only_mask = self.object_removal(image=mask, kernal_a = 10, kernal_b = 10, min_size=1000)
        weeds_only = cv2.bitwise_and(cv_image, cv_image, mask=weeds_only_mask.astype(np.uint8))

        # show centroid;
        gray = cv2.cvtColor(weeds_only, cv2.COLOR_BGR2GRAY)
        _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            weed_pose_array.append([cX, cY])
            cv2.circle(weeds_only, (cX, cY), 7, (255, 0, 0), -1)

        self.publish_weed_array(weed_pose_array)
        return weeds_only
