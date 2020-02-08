#!/usr/bin/env python
import tf
import cv2
import sys
import copy
import math
import rospy
import tf2_ros
import operator
import itertools
import image_geometry
import pdb

import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from scipy.signal import find_peaks
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Point,Point32
from visualization_msgs.msg import MarkerArray, Marker


class convert_to_topo_nav():
    def __init__(self):
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2listener = tf2_ros.TransformListener(self.tf2Buffer)
        self.br = tf.TransformBroadcaster()
        self.camera_info = rospy.wait_for_message('/thorvald_002/kinect2_camera/hd/camera_info', CameraInfo)
        self.pub_weed_pointcloud = rospy.Publisher("/way_points/pointcloud", PointCloud, queue_size=1)
        self.wp_point_cloud = PointCloud()
        self.wp_point_cloud.header.frame_id = 'map'
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.camera_height = 20.5
        self.wp_interval = 10

    def import_image(self):
        image = rospy.wait_for_message("/thorvald_002/kinect2_camera/hd/image_color_rect", Image)
        self.last_ts = image.header.stamp
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        return cv_image

    def remove_soil(self, cv_image):  # ran once to segment the weeds and locate them
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        lower_green = np.array([20,0,0])
        upper_green = np.array([255,255,255])
        mask = cv2.inRange(hsv_img, lower_green, upper_green)

        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((10,10),np.uint8)
        dilate = cv2.dilate(opening, kernel, 1)

        return dilate

    def perpendicular_cumulative_sections(self, cv_image, color_image):
        # cv2.imshow("cv_image", cv_image)
        # k = cv2.waitKey(0)
        # if k ==27:
        #     pass
        imgx = (cv_image.shape[1] / 2)
        imgy = (cv_image.shape[0] / 2)

        # 2. Calculate lines for each angle (0, 179) degrees and do for
        perpendicular_cumulative_section = []
        angles = []
        point = []
        max_angle = 180
        step_size = 10
        for angle in range(0, max_angle, step_size):
            cv_image_copy_1 = np.zeros([cv_image.shape[0], cv_image.shape[1]]) + angle + 1
            angle_rad = (angle * math.pi) / 180
            r = min(imgx, imgy)
            for a in range(-r, r + 1, max(1, int((r * 2)))):
                xcirc = imgx + a * math.cos(angle_rad)
                ycirc = imgy + a * math.sin(angle_rad)
                cv_image_copy_1 = cv2.line(cv_image_copy_1, (int(imgx), int(imgy)), (int(xcirc), int(ycirc)), angle, 1)
                # cv_image_copy_2 = cv2.line(cv_image_copy_2, (int(imgx), int(imgy)), (int(xcirc), int(ycirc)), 200, 1)

            indices = np.where(cv_image_copy_1 == angle)
            n = 20  # How often to take perpendicular line segment
            cumulative_list = []
            for i in range(0, len(indices[0])):
                if i % n == 1:
                    cv_image_copy_2 = np.zeros([cv_image.shape[0], cv_image.shape[1]])
                    for b in range(-r, r + 1, max(1, int((r * 2)))):
                        xcirc_1 = indices[1][i] + b * math.cos(angle_rad - (math.pi / 2))
                        ycirc_1 = indices[0][i] + b * math.sin(angle_rad - (math.pi / 2))
                        cv_image_copy_2 = cv2.line(cv_image_copy_2, (int(indices[1][i]), int(indices[0][i])), (int(xcirc_1), int(ycirc_1)), 100, 1)
                        # cv_image_copy_2[indices[0][i], indices[1][i]] = 200
                    coordinates = np.where(cv_image_copy_2 == 100)
                    sum__ = 0
                    for coord in range(0, len(coordinates[0])):
                        if cv_image[coordinates[0][coord], coordinates[1][coord]] == 255:
                            # cv_image = cv2.circle(cv_image, (coordinates[1][coord], coordinates[0][coord]), 10, 100, 2)
                            # cumulative.append(coordinates[0][coord], coordinates[1][coord], angle)
                            sum__ += 1
                    cumulative_list.append(sum__)

            print("angle: ", angle, "cumulative_list: ", cumulative_list)
            perpendicular_cumulative_section.append(cumulative_list)

            # cv2.imshow("image_square", image_square)
            # k = cv2.waitKey(0)
            # if k == 27:
            #     break
            # cv2.destroyAllWindows()
        peaks_list = []
        for i in range(0, len(perpendicular_cumulative_section)):
            peaks, _ = find_peaks(perpendicular_cumulative_section[i], height=0)
            sum_peaks_list = []
            for j in peaks:
                sum_peaks_list.append(perpendicular_cumulative_section[i][j])
            peaks_list.append([peaks, sum_peaks_list])

        mean_list = []
        for line in peaks_list:
            mean = sum(line[1]) / len(line[1])
            mean_list.append(mean)

        index, value = max(enumerate(mean_list), key=operator.itemgetter(1))
        print index
        # plt.plot(perpendicular_cumulative_section[index])
        # plt.ylabel(('angle: ' + str(index)))
        # plt.ylim([0, 700])
        # plt.show()

        # Now get the correct pixel points for the waypoints:
        # 1. first get the angle correct:
        # index -> angle:
        angle = (index * step_size) - 90
        angle_rad = (angle * math.pi) / 180

        imgx = (cv_image.shape[1] / 2)
        imgy = (cv_image.shape[0] / 2)

        cv_image_copy_1 = np.zeros([cv_image.shape[0], cv_image.shape[1]])
        r = min(imgx, imgy)
        for a in range(-r, r + 1, max(1, int((r * 2)))):
            xcirc = imgx + a * math.cos(angle_rad)
            ycirc = imgy + a * math.sin(angle_rad)
            cv_image_copy_1 = cv2.line(cv_image_copy_1, (int(imgx), int(imgy)), (int(xcirc), int(ycirc)), 150, 1)

        indices = np.where(cv_image_copy_1 == 150)
        cumulative_list = []
        wp_pose_list = []
        wp_pixel_list = []
        for i in range(0, len(indices[0])):
            if i % self.wp_interval == 1:
                cv_image_copy_2 = np.zeros([cv_image.shape[0], cv_image.shape[1]])
                for b in range(-r, r + 1, max(1, int((r * 2)))):
                    xcirc_1 = indices[1][i] + b * math.cos(angle_rad - (math.pi / 2))
                    ycirc_1 = indices[0][i] + b * math.sin(angle_rad - (math.pi / 2))
                    cv_image_copy_2 = cv2.line(cv_image_copy_2, (int(indices[1][i]), int(indices[0][i])), (int(xcirc_1), int(ycirc_1)), 150, 1)
                coordinates = np.where(cv_image_copy_2 == 150)
                waypoint_graph = []
                waypoint_graph_coords = []
                for coord in range(0, len(coordinates[0])):
                    if cv_image[coordinates[0][coord], coordinates[1][coord]] == 255:
                        # cv_image = cv2.circle(cv_image, (coordinates[1][coord], coordinates[0][coord]), 10, 100, 2)
                        # cumulative.append(coordinates[0][coord], coordinates[1][coord], angle)
                        waypoint_graph_coords.append([1, coordinates[0][coord], coordinates[1][coord]])
                        waypoint_graph.append(1)
                    else:
                        waypoint_graph_coords.append([0, coordinates[0][coord], coordinates[1][coord]])
                        waypoint_graph.append(0)

                peaks, _ = find_peaks(waypoint_graph, height=0)
                if len(peaks) != 0:
                    mean_peak_list = self.cluster_analysis(peaks, 20)

                print("peaks: ", peaks)
                plt.plot(waypoint_graph)
                if len(peaks) != 0:
                    for p in mean_peak_list:
                        plt.plot(p, 1, "x")
                plt.ylabel(('state_' + str(i)))
                plt.ylim([0, 2])
                # plt.savefig('WAYPOINTPLOTSMEAN_state_' + str(i))
                plt.clf()

                # now plot peaks on cv_image:::
                if len(peaks) != 0:
                    for pk in mean_peak_list:
                        radius = 5
                        # color_image = cv2.circle(color_image, (waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]), radius, (150, 20, 20), 3)
                        wp_pose_list.append(self.convert_to_world_pose(waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]))
                        wp_pixel_list.append([waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]])

        rows = self.classify(wp_pixel_list, 2.1*self.wp_interval)
        sorted_rows = self.order_rows(rows, angle_rad)


        ''' Plotting the classification result:'''
        for row in sorted_rows:
            colour = np.random.rand(3,) * 150
            for point in row:
                color_image = cv2.circle(color_image, (point[0], point[1]), radius, colour, 3)
        cv2.imshow("color_image", color_image)
        k = cv2.waitKey(0)
        if k == 27:
            pass
        cv2.destroyAllWindows()

        ''' Publish the point cloud:'''
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.pub_weed_pointcloud.publish(self.wp_point_cloud)
        #     rate.sleep()
        '''More plotting '''
        # self.visualization_of_waypoints(wp_pose_list)

    def euclidean_dist(self,a,b,x,y):
        xdiff = b - y
        ydiff = a - x
        distance = np.sqrt((xdiff**2) + (ydiff**2))
        return distance

    def classify(self, waypoints, max_dist):
        # INPUT: a list of lists holding image coordinates (row, column)
        # OUPUT: Image pixels classified into lists for each crop row
        rows = [] # initialise an empty list of lists to store the single rows
        row_count = -1 # to keep track of the current class
        next_to_check = [] # to keep track of which other points within the same class need to be examinred before moving on

        while len(waypoints) > 0:
            if len(next_to_check) == 0:
                # Move on to a new starting point if the current thread is fully explored
                rows.append([]) # open a new class (or row)
                next_to_check.append(waypoints[0]) # pick a new point to examine
                row_count += 1
                rows[row_count].append(waypoints[0]) # add the new point to the new row
                del waypoints[0] # remove the point from the remaining

            (r,c) = next_to_check[0]

            distances = [self.euclidean_dist(r,c,waypoints[i][0],waypoints[i][1]) for i in range(len(waypoints))]
            ind = np.argwhere(np.array(distances) < max_dist)
            indeces = np.reshape(ind, [ind.shape[0],])
            if len(indeces) > 0:
                for element in indeces[::-1]:
                    rows[row_count].append(waypoints[element])
                    next_to_check.append(waypoints[element])
                    del waypoints[element]
            del(next_to_check[0])
        return rows

    def order_rows(self, rows, angle):
		# INPUT: A list of lists, where each single list contains waypoints of a seperate row
		# INPUT: The general angle of the rows in radians
		# OUPUT: returns the same waypoints, but in the correct order for navigating along the rows
		order = []
		row_count = 0
		# rotate the data set by the vector of maximal variance (i.e. the angle)
		# Dimensions reduced to x,y --> only y
		V = np.zeros([2,1])
		V[0] = np.cos(angle)
		V[1] = np.sin(angle)

		for row in rows:
			order.append([])
			points = []
			for point in row:
				wp_vector = np.reshape(np.array(point),[1,2])
				rotated_vector = np.dot(wp_vector,V)
				points.append(float(rotated_vector))
			order[row_count] = np.argsort(points)
			row_count += 1

		sorted_rows = []
		row_count = 0
		for i,row in enumerate(rows):
			sorted_rows.append([])
			for j in range(len(row)):
				index = order[i][j]
				sorted_rows[row_count].append(row[index])
			row_count += 1
		return sorted_rows

    def visualization_of_waypoints(self, wp_list):
        for wp_xy in wp_list:
            wp = Point()
            wp.x = wp_xy[0]
            wp.y = wp_xy[1]
            wp.z = 0
            self.wp_point_cloud.points.append(wp)

    def convert_to_world_pose(self, pose_x, pose_y):
        uv = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint([pose_x, pose_y]))
        way_point_world = [((uv[0] * self.camera_height) - 0.45), (uv[1] * self.camera_height)]
        return way_point_world

    def cluster_analysis(self, peaks, maxgap):
        mean_peak_list = []
        peaks.sort()
        groups = [[peaks[0]]]
        for x in peaks[1:]:
            if abs(x - groups[-1][-1]) <= maxgap:
                groups[-1].append(x)
            else:
                groups.append([x])
        for p in groups:
            mean_peak_list.append(sum(p) / len(p))

        return mean_peak_list

if __name__ == '__main__':
    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    no_soil_image = convert.remove_soil(raw_image)
    # convert.calculate_crop_rows(no_soil_image)
    convert.perpendicular_cumulative_sections(no_soil_image, raw_image)
