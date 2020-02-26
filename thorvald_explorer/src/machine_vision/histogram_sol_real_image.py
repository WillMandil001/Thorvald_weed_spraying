#!/usr/bin/env python
import tf
import pdb
import csv
import cv2
import sys
import copy
import math
import rospy
import tf2_ros
import operator
import itertools
import image_geometry

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
import pickle
import time

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
        self.wp_interval = 5
        self.extension_d = 12
        self.cluster_distance_scalar = 13
        self.max_deviation = 10 # max permitted deviation perpendicular to the angle of travel (used to make map sparse)
        self.pixel_peak_clustering_dist = 7
        
        self.camera_FoV = 26 # in pixels
        self.wheel_centre_distance = 30
        self.wheel_width = 9

        self.image_file = 'RealcropA_2.png'
        self.annotated_img_file = 'Evaluation/world4_annotated.png'

    def import_image(self):
        #image = rospy.wait_for_message("/thorvald_002/kinect2_camera/hd/image_color_rect", Image)
        #self.last_ts = image.header.stamp
        #cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv_image = cv2.imread(self.image_file)
        return cv_image

    def remove_soil(self, cv_image):  # ran once to segment the weeds and locate them
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # convert to hsv for better color segmentation:
        self.show_hsv_split(hsv_img)

        if self.image_file == 'RealcropA_2.png':

            low1 = np.array([0,0,0])
            upp1 = np.array([30,255,255])
            low2 = np.array([85,0,0])
            upp2 = np.array([255,255,255])
            low3 = np.array([0,0,0])
            upp3 = np.array([255,255,30])
            mask1 = cv2.bitwise_not(cv2.inRange(hsv_img, low1, upp1))
            mask2 = cv2.bitwise_not(cv2.inRange(hsv_img, low2, upp2))
            mask3 = cv2.bitwise_not(cv2.inRange(hsv_img, low3, upp3))
            overlaid = cv2.bitwise_and(mask1, mask2)
            overlaid = cv2.bitwise_and(overlaid, mask3)
            
            kernel = np.ones((3,3),np.uint8)
            erosion = cv2.erode(overlaid,kernel,iterations = 2)

            result_of_find = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            kj, contours, hierachy = result_of_find
            threshold = 250
            list_cnt = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > threshold:
                    list_cnt.append(cnt)
            cv2.drawContours(erosion, list_cnt, -1, 255, -1)

            cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('mask', 600,600)
            cv2.imshow('mask', erosion)

            k = cv2.waitKey(0)
            if k ==27:
                pass
            return erosion 


        elif self.image_file == 'RealcropD_2.png':

            low1 = np.array([0,0,0])
            upp1 = np.array([20,255,255])
            low2 = np.array([90,0,0])
            upp2 = np.array([110,255,255])
            low3 = np.array([0,0,80])
            upp3 = np.array([255,255,165])
            mask1 = cv2.bitwise_not(cv2.inRange(hsv_img, low1, upp1))
            mask2 = cv2.bitwise_not(cv2.inRange(hsv_img, low2, upp2))
            mask3 = cv2.bitwise_not(cv2.inRange(hsv_img, low3, upp3))
            overlaid = cv2.bitwise_and(mask1, mask2)
            overlaid = cv2.bitwise_and(overlaid, mask3)
            
            kernel = np.ones((3,3),np.uint8)
            opening = cv2.morphologyEx(overlaid, cv2.MORPH_OPEN, kernel)
            kernel = np.ones((10,10),np.uint8)
            dilate = cv2.dilate(opening, kernel, 1)

            cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('mask', 600,600)
            cv2.imshow('mask', overlaid)

            cv2.namedWindow('dilate',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('dilate', 600,600)
            cv2.imshow("dilate", dilate)
            k = cv2.waitKey(0)
            if k ==27:
                pass
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

            #print("angle: ", angle, "cumulative_list: ", cumulative_list)
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
        #print(index)
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
        r = 2 * max(imgx, imgy)
        for a in range(-r, r + 1, max(1, int((r * 2)))):
            xcirc = imgx + a * math.cos(angle_rad)
            ycirc = imgy + a * math.sin(angle_rad)
            cv_image_copy_1 = cv2.line(cv_image_copy_1, (int(imgx), int(imgy)), (int(xcirc), int(ycirc)), 150, 1)

        indices = np.where(cv_image_copy_1 == 150)
        cumulative_list = []
        wp_pose_list = []
        wp_pixel_list = []

        wp_pose_list_c = []
        wp_pixel_list_c = []

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

                peak_all = []
                peaks, properties = find_peaks(waypoint_graph, height=0)
                for index,point in enumerate(waypoint_graph):
                    if point == 1:
                        peak_all.append(waypoint_graph_coords[index])

                left_edge = None
                sparse_peaks = []
                for i,p in enumerate(peak_all):
                    if not left_edge:
                        left_edge = (p[1],p[2])
                        previous_point = p
                    elif i == len(peak_all) - 1:
                        right_edge = (p[1],p[2])
                        peak_coordinate = [(right_edge[1]+left_edge[1])/2,(right_edge[0]+left_edge[0])/2]
                        wp_pixel_list_c.append(peak_coordinate)
                        left_edge = None

                    else:
                        if self.euclidean_dist(previous_point[1], previous_point[2],p[1],p[2]) > self.pixel_peak_clustering_dist:
                            right_edge = (previous_point[1],previous_point[2])
                            peak_coordinate = [(right_edge[1]+left_edge[1])/2,(right_edge[0]+left_edge[0])/2]
                            wp_pixel_list_c.append(peak_coordinate)
                            previous_point = None
                            left_edge = None
                        else:
                            previous_point = p

                # wp_pixel_list_c.append(sparse_peaks)
                # #print("peaks: ", peaks)
                # plt.plot(waypoint_graph)
                # if len(peaks) != 0:
                #     # for p, prop in zip(peaks, properties):
                #         # plt.plot(p, 1, "o")
                #     for p in mean_peak_list:
                #         plt.plot(p, 1, "x")
                # plt.ylabel(('state_' + str(i)))
                # plt.ylim([0, 2])
                # # plt.savefig('WAYPOINTPLOTSMEAN_state_' + str(i))
                # plt.clf()

                # # now plot peaks on cv_image:::
                # if len(peaks) != 0:
                #     for pk in peaks:
                #         radius = 5
                #         # color_image = cv2.circle(color_image, (waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]), radius, (150, 20, 20), 3)
                #         wp_pose_list_c.append(self.convert_to_world_pose(waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]))
                #         wp_pixel_list_c.append([waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]])

                # now plot peaks on cv_image:::
                # if len(peaks) != 0:
                #     for pk in mean_peak_list:
                #         radius = 5
                #         # color_image = cv2.circle(color_image, (waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]), radius, (150, 20, 20), 3)
                #         wp_pose_list.append(self.convert_to_world_pose(waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]))
                #         wp_pixel_list.append([waypoint_graph_coords[pk][2], waypoint_graph_coords[pk][1]])

        # for point in wp_pixel_list_c:
        #     color_image = cv2.circle(color_image, (point[1], point[0]), 5, (150, 20, 20), 3)
        # cv2.imshow("sparse_peaks",color_image)
        # cv2.waitKey(0)

        #rows = self.classify(wp_pixel_list, self.cluster_distance_scalar *self.wp_interval)
        rows_c = self.classify(wp_pixel_list_c, self.cluster_distance_scalar)

        sorted_rows = self.order_rows(rows_c, angle_rad)
        extended_sorted_rows = self.extend_points(sorted_rows, angle_rad)
        sparse_waypoints = self.drop_redundant_wp(extended_sorted_rows)

        self.plot_wp(rows_c, "rows_c")
        pdb.set_trace()
        #self.plot_wp(rows, "rows")
        #pdb.set_trace()
        self.plot_wp(sorted_rows, "colour_im")
        pdb.set_trace()
        self.plot_wp(extended_sorted_rows, "colour_im")
        pdb.set_trace()
        self.plot_wp(sparse_waypoints, "colour_im")
        pdb.set_trace()
        path_img, overlap1, overlap2 = self.evaluate_camera(sparse_waypoints)
        wheel_img, overlap3, overlap4 = self.evaluate_wheels(sparse_waypoints)
        stats = self.compute_overlap(path_img,wheel_img)

        sparse_rows_world = self.convert_wplist_to_world(sparse_waypoints)

        with open("sparse_sorted_rows.csv","w") as f:
            wr = csv.writer(f)
            wr.writerows(sparse_rows_world)

        ''' Exort waypoint pixel locations for evaluation'''
        # with open('toponav.waypoints', 'wb') as wp_file:
        #   pickle.dump(extended_sorted_rows, wp_file)

        ''' Plotting the classification result:'''
        for row in sparse_waypoints:
            colour = np.random.rand(3,) * 150
            for point in row:
                color_image = cv2.circle(color_image, (point[0], point[1]), radius, colour, 3)
        #cv2.imshow("color_image", color_image)
        #k = cv2.waitKey(0)
        #if k == 27:
        #     pass
        #cv2.destroyAllWindows()

        ''' Publish the point cloud:'''
        # self.visualization_of_waypoints(extended_sorted_rows_world)
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #   self.pub_weed_pointcloud.publish(self.wp_point_cloud)
        #   rate.sleep()
        return stats, color_image, overlap1, overlap2, overlap3, overlap4

    def plot_wp(self, wp, name):
        color_image = cv2.imread(self.image_file)
        colours = [(245, 69, 66),(21, 21,214),(30,168,214),(23,194,37), (123,135,55),(100,13,105),(59,56,49),(135,110,212),(245, 69, 66),(21, 21,214),(30,168,214),(23,194,37), (123,135,55),(100,13,105)]
        for i,row in enumerate(wp):
            colour = np.random.rand(3,) * 150
            for point in row:
                color_image = cv2.circle(color_image, (point[0], point[1]), 5, colour, 3)
        cv2.imshow(name, color_image)
        k = cv2.waitKey(0)
        if k == 27:
            pass
        #cv2.destroyAllWindows()

    def convert_wplist_to_world(self, extended_sorted_rows):
        wp_list_list = []
        for row in extended_sorted_rows:
            wp_list = []
            for wp in row:
                wp_list.append(self.convert_to_world_pose(wp[0], wp[1]))
            wp_list_list.append(wp_list)

        return wp_list_list

    def extend_points(self, sorted_rows, angle_rad):
        # pdb.set_trace()
        for i in range(0, len(sorted_rows)):
            if len(sorted_rows[i]) > 10:
                fx= sorted_rows[i][0][0] + (-self.extension_d * math.cos(angle_rad))
                fy= sorted_rows[i][0][1] + (-self.extension_d * math.sin(angle_rad))
                first_new_wp = [int(fx), int(fy)]
                sorted_rows[i].insert(0, first_new_wp)
                lx= sorted_rows[i][-1][0] + (self.extension_d * math.cos(angle_rad))
                ly= sorted_rows[i][-1][1] + (self.extension_d * math.sin(angle_rad))
                last_new_wp = [int(lx), int(ly)]
                sorted_rows[i].append(last_new_wp)
                #print(first_new_wp, last_new_wp)

        return sorted_rows

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

        pdb.set_trace()
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
        W = np.zeros([2,1])
        V[0] = np.cos(angle)
        V[1] = np.sin(angle)
        W[0] = np.sin(angle)
        W[1] = np.cos(angle)

        mean_height = []
        for row in rows:
            heights = []
            order.append([])
            points = []
            for point in row:
                wp_vector = np.reshape(np.array(point),[1,2])
                rotated_vector_dist = np.dot(wp_vector,V)
                rotated_vector_height = np.dot(wp_vector,W)
                points.append(float(rotated_vector_dist))
                heights.append(float(rotated_vector_height))
            order[row_count] = np.argsort(points)
            mean_height.append(np.mean(heights))
            row_count += 1

        sorted_rows = []
        row_count = 0
        for i,row in enumerate(rows):
            sorted_rows.append([])
            for j in range(len(row)):
                index = order[i][j]
                sorted_rows[row_count].append(row[index])
            row_count += 1

        double_sorted = []
        row_order = np.argsort(mean_height)
        for num in row_order:
            double_sorted.append(sorted_rows[num])
        return double_sorted

    def drop_redundant_wp(self, wp):
        sparse_wp = []
        row_count = -1
        for row in wp:
            row_count += 1
            sparse_wp.append([])
            anchor_point = []
            for point in row:
                if not anchor_point:
                    previous_angle = []
                    anchor_point = tuple(point)
                    sparse_wp[row_count].append(point)
                else:
                    dx = point[0]-anchor_point[0]
                    dy = point[1]-anchor_point[1]
                    angle = math.atan2(dy, dx)
                    if angle < 0:
                        sign = -1
                    else:
                        sign = 1
                    angle = abs(angle)
                    while angle > (math.pi/2):
                        angle -= (math.pi/2)

                    if not previous_angle:
                        # Previous angle is always from the current anchor point
                        # to the immediate next point
                        previous_angle = sign * angle
                        prev_point = tuple(point)
                    else:
                        length = math.sqrt(dx**2 + dy**2)
                        delta_angle = (sign * angle) - previous_angle
                        deviation = np.sin(delta_angle) * length

                        if abs(deviation) > self.max_deviation:
                        #elif abs((sign * angle) - previous_angle) > self.max_angle:
                            sparse_wp[row_count].append(prev_point)
                            # Update the new anchor point
                            anchor_point = tuple(prev_point)
                            prev_point = tuple(point)

                            # Update the "previous angle" to the angle between the
                            # new anchor point and the immediately following point
                            dx = point[0]-anchor_point[0]
                            dy = point[1]-anchor_point[1]
                            angle = math.atan2(dy, dx)
                            if angle < 0:
                                sign = -1
                            else:
                                sign = 1
                            angle = abs(angle)
                            while angle > (math.pi/2):
                                angle -= (math.pi/2)
                            previous_angle = sign * angle
                        prev_point = tuple(point)
            sparse_wp[row_count].append(point)
        return sparse_wp


    def evaluate_camera(self, wp):
        ''' 
        Evaluates the area covered by the ground robot's camera
        Plotting has been commented for now
        '''
        img = cv2.imread(self.annotated_img_file, 0) # annotated image
        ret,annotated_img = cv2.threshold(img,50,1,cv2.THRESH_BINARY)
        vis = cv2.imread('world4.png')
        comp = cv2.imread(self.annotated_img_file)
        path_img = copy.deepcopy(annotated_img)
        ret,path_img = cv2.threshold(path_img,255,1,cv2.THRESH_BINARY)
        previous_point = []
        for i,row in enumerate(wp):
            if (i % 2) == 0:
                row.reverse()
            for point in row:
                if not previous_point:
                    previous_point = tuple(point)
                else:
                    cv2.line(path_img, previous_point, tuple(point), (255,255,255), self.camera_FoV)
                    cv2.line(vis, previous_point, tuple(point), (255,255,255), self.camera_FoV)
                    cv2.line(comp, previous_point, tuple(point), (255,0,0), self.camera_FoV)
                    previous_point = tuple(point)
        #cv2.imshow('path',path_img)
        #cv2.imshow('vis',vis)
        #cv2.imshow('c',comp)
        #k = cv2.waitKey(0)
        return path_img, vis, comp


    def evaluate_wheels(self, wp):
        ''' 
        Evaluates the area covered by the ground robot's wheels
        Plotting has been commented for now
        '''
        img = cv2.imread(self.annotated_img_file, 0) # annotated image
        ret,annotated_img = cv2.threshold(img,50,1,cv2.THRESH_BINARY)
        vis2 = cv2.imread('world4.png')
        comp2 = cv2.imread(self.annotated_img_file)
        wheel_img = copy.deepcopy(annotated_img)
        ret,wheel_img = cv2.threshold(wheel_img,50,1,cv2.THRESH_BINARY)

        previous_point = []
        for i,row in enumerate(wp):
            if (i % 2) == 0:
                pass
                #row.reverse()
            for point in row:
                if not previous_point:
                    previous_point = tuple(point)
                else:
                    delta_x = point[0] - previous_point[0]
                    delta_y = point[1] - previous_point[1]

                    L = math.sqrt(delta_x**2 + delta_y **2)
                    dx = int(delta_y * self.wheel_centre_distance/L)
                    dy = int(delta_x * -self.wheel_centre_distance/L)

                    cv2.line(wheel_img, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,255,255), self.wheel_width)
                    cv2.line(wheel_img, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,255,255), self.wheel_width)
                    cv2.line(vis2, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,255,255), self.wheel_width)
                    cv2.line(vis2, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,255,255), self.wheel_width)
                    cv2.line(comp2, (previous_point[0] - dx, previous_point[1] - dy), (point[0] - dx, point[1] -dy), (255,0,0), self.wheel_width)
                    cv2.line(comp2, (previous_point[0] + dx, previous_point[1] + dy), (point[0] + dx, point[1] +dy), (255,0,0), self.wheel_width)

                    previous_point = tuple(point)

        #cv2.imshow('wheels',wheel_img)
        #cv2.imshow('wheels',vis2)
        #cv2.imshow('c',comp2)
        k = cv2.waitKey(0)
        return wheel_img, vis2, comp2

    def compute_overlap(self, camera_im, wheel_im):
        img = cv2.imread(self.annotated_img_file, 0) # annotated image
        ret,annotated_img = cv2.threshold(img,50,1,cv2.THRESH_BINARY)
        crop_area = sum(sum(annotated_img))
        #print('crop area', crop_area)
        camera_overlap = cv2.bitwise_and(annotated_img, camera_im)
        wheel_overlap = cv2.bitwise_and(annotated_img, wheel_im)
        overlap_nr_camera = sum(sum(camera_overlap))
        overlap_nr_wheel = sum(sum(wheel_overlap))
        #print(sum(sum(wheel_overlap)))
        return crop_area, overlap_nr_camera, overlap_nr_wheel

    def visualization_of_waypoints(self, wp_list):
        if any(isinstance(el, list) for el in wp_list):
            for i in wp_list:
                for j in i:
                    wp = Point()
                    wp.x = j[0]
                    wp.y = j[1]
                    wp.z = 0
                    self.wp_point_cloud.points.append(wp)
        else:
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
            p[0]
        # for p in groups:
        #     mean_peak_list.append(sum(p) / len(p))

        # return mean_peak_list

    def evaluation_pipeline(self, no_soil_im, raw_im):
        deviations = range(1,21)
        stat_array = []
        im_array = []
        overlap_array = []
        overlap_images = []
        for dev in deviations:
            self.max_deviation = dev
            working_copy_raw_im = copy.deepcopy(raw_im)
            stats, im, overlap1, overlap2, overlap3, overlap4 = self.perpendicular_cumulative_sections(no_soil_im, working_copy_raw_im)
            stat_array.append(stats)
            im_array.append(im)
            overlap_array.append((overlap1, overlap2, overlap3, overlap4))
        with open('results/eval.stats', 'wb') as wp_file:
            pickle.dump(stat_array, wp_file)
        with open('results/eval.imgs', 'wb') as wp_file:
            pickle.dump(im_array, wp_file)
        with open('results/overlap.imgs', 'wb') as wp_file:
            pickle.dump(overlap_array, wp_file)

    def show_hsv_split(self, hsv_img):
        h,s,v = cv2.split(hsv_img)
        cv2.namedWindow('h',cv2.WINDOW_NORMAL)
        cv2.namedWindow('s',cv2.WINDOW_NORMAL)
        cv2.namedWindow('v',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('h', 600,600)
        cv2.resizeWindow('s', 600,600)
        cv2.resizeWindow('v', 600,600)
        cv2.imshow("h", h)
        cv2.imshow("s", s)
        cv2.imshow("v", v)
        k = cv2.waitKey(0)

if __name__ == '__main__':
    start = time.time()

    rospy.init_node('convert_to_topo_nav', anonymous=True)
    convert = convert_to_topo_nav()
    raw_image = convert.import_image()
    # cv2.imshow("raw_image", raw_image)
    no_soil_image = convert.remove_soil(raw_image)
    #cv2.imshow("no_soil_image", no_soil_image)
    #k = cv2.waitKey(0)
    #convert.evaluation_pipeline(no_soil_image, raw_image)
    convert.perpendicular_cumulative_sections(no_soil_image, raw_image)
    end = time.time()
    print(end-start)