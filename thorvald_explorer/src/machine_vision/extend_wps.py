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
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Point,Point32
from visualization_msgs.msg import MarkerArray, Marker


class convert_to_topo_nav():
    def __init__(self):
        pass

    def wp_extend(self):
        angle = [45, 0, -45, -90]
        extension_d = 1

        rows = [[(1,1), (2,2), (3,3), (4,4)],
                [(1,0), (2,0), (3,0), (4,0)],
                [(1,4), (2,3), (3,2), (4,1)],
                [(1,4), (1,3), (1,2), (1,1)]]

        for row, i in zip(rows, angle):
            xx= row[0][0] + (-extension_d * math.cos((i * math.pi) / 180))
            yy= row[0][1] + (-extension_d * math.sin((i * math.pi) / 180))
            first_new_wp = [xx, yy]
            xx= row[-1][0] + (extension_d * math.cos((i * math.pi) / 180))
            yy= row[-1][1] + (extension_d * math.sin((i * math.pi) / 180))
            last_new_wp = [xx, yy]
            print(first_new_wp, last_new_wp ,"\n")

            for j in row:
                print j
                plt.plot(j[0], j[1], 'bo')
                # plt.plot(j[1], "go")
            # plt.plot(row[0][0])
            plt.plot(first_new_wp[0], first_new_wp[1], "rx")
            plt.plot(last_new_wp[0], last_new_wp[1], "bx")
            plt.ylim([-7, 7])
            plt.xlim([-7, 7])
            plt.grid()
            plt.show()
            # plt.clf()


if __name__ == '__main__':
    convert = convert_to_topo_nav()
    convert.wp_extend()
