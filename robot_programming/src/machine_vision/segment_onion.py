#!/usr/bin/env python
import tf
import cv2
import math
import rospy
import tf2_ros
import numpy as np
import image_geometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker

## Class which is used by the vision controller to segment onions.
class segment_onion_class():
    def __init__(self):
        self.bridge = CvBridge()  ## Initialise all tf and relevant subs and pubs
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
        im_no_soil = self.remove_soil(cv_image)
        im_only_grass = self.remove_crop(im_no_soil, cv_image)
        weed_img = self.weed_or_grass(im_only_grass, cv_image, im_no_soil)
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(weed_img, "bgr8"))
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

    def remove_soil(self, cv_image):
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        lower_green = np.array([30,0,0])
        upper_green = np.array([255,255,255])
        mask_no_soil = cv2.inRange(hsv_img, lower_green, upper_green)
        masked_img = cv2.bitwise_and(cv_image, cv_image, mask=mask_no_soil)
        return masked_img

    def remove_crop(self, no_soil_image, cv_image):
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        lower_green = np.array([40,0,30])
        upper_green = np.array([255,90,255])
        mask_no_weeds = cv2.inRange(hsv_img, lower_green, upper_green)

        # Now clean up the noise:
        min_size = 200
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(mask_no_weeds, connectivity=4) # remove larger objects still remaining
        sizes = stats[1:, -1]
        nb_components = nb_components - 1
        mask_no_weeds_clean = np.zeros((output.shape))
        for i in range(0, nb_components):
            if sizes[i] > min_size:
                mask_no_weeds_clean[output == i + 1] = 255
        im_weeds_colour = cv2.bitwise_and(cv_image, cv_image, mask=mask_no_weeds_clean.astype(np.uint8))

        gray = cv2.cvtColor(im_weeds_colour, cv2.COLOR_BGR2GRAY)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)  # y
        ret, thresh1 = cv2.threshold(sobely, 0, 1, cv2.THRESH_BINARY)
        sobely_colour = cv2.bitwise_and(cv_image, cv_image, mask=thresh1.astype(np.uint8))

        gray = cv2.cvtColor(sobely_colour, cv2.COLOR_BGR2GRAY)
        min_size = 350
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(gray, connectivity=4) # remove larger objects still remaining
        sizes = stats[1:, -1]
        nb_components = nb_components - 1
        mask_no_weeds_clean = np.zeros((output.shape))
        for i in range(0, nb_components):
            if sizes[i] > min_size:
                mask_no_weeds_clean[output == i + 1] = 255
        return mask_no_weeds_clean

    def weed_or_grass(self, image, cv_image, im_no_soil):
        weed_pose_array = []
        im_weeds_colour = cv2.bitwise_and(cv_image, cv_image, mask=image.astype(np.uint8))
        gray = cv2.cvtColor(im_weeds_colour, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLines(gray, 1, np.pi / 180, 200)
        if lines is not None:  # locate the first line of onions and apply line mask over it.
            for rho, theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 10000 * (-b))
                y1 = int(y0 + 10000 * (a))
                x2 = int(x0 - 10000 * (-b))
                y2 = int(y0 - 10000 * (a))
                cv2.line(image, (x1, y1), (x2, y2), 0, 300)
                line_1 = [x1, y1, x2, y2]

            im_weeds_colour = cv2.bitwise_and(cv_image, cv_image, mask=image.astype(np.uint8))
            gray = cv2.cvtColor(im_weeds_colour, cv2.COLOR_BGR2GRAY)
            lines = cv2.HoughLines(gray, 1, np.pi / 180, 200)
            if lines is not None:  # locate the second line of onions and apply line mask over it..
                for rho, theta in lines[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 10000 * (-b))
                    y1 = int(y0 + 10000 * (a))
                    x2 = int(x0 - 10000 * (-b))
                    y2 = int(y0 - 10000 * (a))
                    cv2.line(image, (x1, y1), (x2, y2), 0, 300)
                    line_2 = [x1, y1, x2, y2]

                kernel = np.array(([[0, 0, 0], [1, 1, 1], [0, 0, 0]]), np.uint8)
                dilation = cv2.dilate(image, kernel, iterations=10)

                cv2.line(dilation, (line_1[0], line_1[1]), (line_1[2], line_1[3]), 255, 300)
                cv2.line(dilation, (line_2[0], line_2[1]), (line_2[2], line_2[3]), 255, 300)
                dilation = (dilation - 255)
                im_weeds_colour = cv2.bitwise_and(im_no_soil, im_no_soil, mask=dilation.astype(np.uint8))

        # Need to remove the final grasses from the image:
        # erode with respect y-axis to remove onion blodes sticking out from the thick lines
        kernel = np.array([[0, 0, 0, 0, 0], 
                        [0, 0, 0, 0, 0], 
                        [1, 1, 1, 1, 1], 
                        [0, 0, 0, 0, 0], 
                        [0, 0, 0, 0, 0]], dtype = np.uint8)
        im_weeds_colour_noiseless = cv2.erode(im_weeds_colour,kernel,iterations = 3)

        kernel = np.ones((3,3),np.uint8)
        im_weeds_colour_noiseless = cv2.erode(im_weeds_colour_noiseless, kernel, iterations = 2)
        im_weeds_colour_noiseless = cv2.dilate(im_weeds_colour_noiseless, kernel, iterations = 2)

        # show centroid
        gray_2 = cv2.cvtColor(im_weeds_colour_noiseless, cv2.COLOR_BGR2GRAY)
        _, contours, _ = cv2.findContours(gray_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                weed_pose_array.append([cX, cY])
                cv2.circle(im_weeds_colour_noiseless, (cX, cY), 7, (255, 0, 0), -1)

        self.publish_weed_array(weed_pose_array)
        return im_weeds_colour_noiseless
