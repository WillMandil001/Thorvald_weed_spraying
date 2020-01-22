#!/usr/bin/env python
import tf
import cv2
import rospy
import numpy as np
import image_geometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

## this class maps the weed centroids output from the machine vision algorithm

class weed_mapper:
    def __init__(self):
        self.weed_array = PoseArray()
        self.weed_point_cloud = PointCloud()
        self.weed_point_cloud.header.frame_id = 'map'
        self.weed_array.header.frame_id = "map"
        self.pub_weed_array = rospy.Publisher("weed_map/pose_array", PoseArray, queue_size=10)
        self.pub_weed_pointcloud = rospy.Publisher("/weed_map/pointcloud", PointCloud, queue_size=1)

    def subscribe_to_weeds(self):
        self.weed_sub = rospy.Subscriber("weed_to_kill", PoseStamped, self.collect_weed)
        rospy.spin()

    def collect_weed(self, data):  # for each new pose recieved from the MV algorithms
        # self.weed_array.poses.append(data.pose)  # used for publishing a pose_array, if that would be more useful for spraying
        # self.pub_weed_array.publish(self.weed_array)
        self.weed_point_cloud.points.append(data.pose.position)
        self.pub_weed_pointcloud.publish(self.weed_point_cloud)


if __name__ == '__main__':
    rospy.init_node('weed_mapper', anonymous=True)
    wm = weed_mapper()
    wm.subscribe_to_weeds()
