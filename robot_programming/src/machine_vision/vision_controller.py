#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import segment_basil  # import the three different methods for weed segmentation
import segment_cabbage
import segment_onion
from topological_navigation.msg import GotoNodeActionGoal


class Sub:  # This is a class that listens for the current node (topo_nav) and depending on the crop it uses a different MV algorithm
    def __init__(self):
        self.sb = segment_basil.segment_basil_class()
        self.sc = segment_cabbage.segment_cabbage_class()
        self.so = segment_onion.segment_onion_class()
        self.to_run = "n"

    def callback(self, data):
        if data.goal.target == "WPline1_1" or data.goal.target == "WPline2_0":
            self.to_run = "b"  # basil
        elif data.goal.target == "WPline3_1" or data.goal.target == "WPline4_0":
            self.to_run = "c"  # cabbage
        elif data.goal.target == "WPline5_1" or data.goal.target == "WPline6_0":
            self.to_run = "o"  # Onion
        else:
            self.to_run = "n"  # not moveing through a crop row so dont weed

    def starter(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.Subscriber("/thorvald_001/topological_navigation/goal", GotoNodeActionGoal, self.callback)
            if self.to_run == "c":
                self.sc.segment_once()  # call the weed segmentation algorithm.
            elif self.to_run == "o":
                self.so.segment_once()
            elif self.to_run == "b":
                self.sb.segment_once()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('segment_basil', anonymous=True)
    s = Sub()
    s.starter()