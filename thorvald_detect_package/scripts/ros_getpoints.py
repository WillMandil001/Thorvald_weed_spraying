#!/usr/bin/env python

# Python libs
import sys
import math

# Ros libraries
import rospy
import tf
from std_srvs.srv import Empty

# Ros Messages
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry

import csv
import numpy
from numpy import savetxt, asarray

plant_file = "plants.csv"
weed_file = "weeds.csv"
weeds = []
w1 = []
w2 = []
w3 = []
w4 = []

plants = []

p1 = []
p2 = []
p3 = []
p4 = []

class image_projection():

    def __init__(self, robot):
        self.weedkeeplist = []
        self.robot = robot
        self.plantkeeplist = []
        self.notsprayed = []

        

        self.spr = rospy.ServiceProxy(
            "{}/spray".format(self.robot),
            Empty)

        rospy.Subscriber(
            "/plant/points/{}".format(self.robot),
            PointCloud,
            self.plantpoints_callback)

        rospy.Subscriber(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            self.weedpoints_callback)

        rospy.Subscriber(
            "/{}/odometry/base_raw".format(self.robot),
            Odometry,
            self.odometry_callback)

        self.weedpoints_pub = rospy.Publisher(
            "/weed/allpoints/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.plantpoints_pub = rospy.Publisher(
            "/plant/allpoints/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.weedpoints_msg = PointCloud()
        self.plantpoints_msg = PointCloud()

        self.tflistener = tf.listener.TransformListener()

    def publish_allweedpoints(self):
        time = rospy.Time(0)
        self.weedpoints_msg.points = self.weedkeeplist
        self.weedpoints_msg.header.frame_id = 'map'
        self.weedpoints_msg.header.stamp = time

        self.weedpoints_pub.publish(self.weedpoints_msg)

    def publish_allplantpoints(self):
        time = rospy.Time(0)
        self.plantpoints_msg.points = self.plantkeeplist
        self.plantpoints_msg.header.frame_id = 'map'
        self.plantpoints_msg.header.stamp = time

        self.plantpoints_pub.publish(self.plantpoints_msg)

    def odometry_callback(self, data):
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
            # print(trans, rot)
        except Exception:
            return

        newkeep = []
        shouldSpray = False
        for keep in self.notsprayed:
            dx = abs(trans[0] - keep.x)
            # print(dx)
            if dx < 0.07:
                shouldSpray = True
            else:
                newkeep.append(keep)

        if shouldSpray:
            self.notsprayed = newkeep
            # print('spray!!!')
            # self.spr()

        self.publish_allweedpoints()
        self.publish_allplantpoints()

    def weedpoints_callback(self, data):
        hasChanged = False
        for point in data.points:
            # check if point has already been added
            found_close = False
            for keep in self.weedkeeplist:
                dx = abs(point.x - keep.x)
                dy = abs(point.y - keep.y)
                dist = math.hypot(dx, dy)
                if dist < 0.07:
                    found_close = True

            # Not found on our list, append it
            if not found_close:
                hasChanged = True
                self.weedkeeplist.append(point)
                weeds.append(point)
                # print(weeds)
                self.notsprayed.append(point)

        if hasChanged:
            print('Weeds Found points: {}'.format(len(self.weedkeeplist)))
        # print(self.weedkeeplist)

    def plantpoints_callback(self, data):
        hasChanged = False
        for point in data.points:
            # check if point has already been added
            found_close = False
            for keep in self.plantkeeplist:
                dx = abs(point.x - keep.x)
                dy = abs(point.y - keep.y)
                dist = math.hypot(dx, dy)
                if dist < 0.04:
                    found_close = True

            # Not found on our list, append it
            if not found_close:
                hasChanged = True
                self.plantkeeplist.append(point)
                plants.append(point)

                # self.notsprayed.append(point)
                

        if hasChanged:
            print('Plants Found points: {}'.format(len(self.plantkeeplist)))
        # print(self.plantkeeplist)





if __name__ == '__main__':
    '''Initializes and cleanup ros node'''
    rospy.init_node('get_points', anonymous=True)
    image_projection('thorvald_001')
    csv_columns = ['x','y','z']
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    

    if rospy.is_shutdown():
        
        for weed in weeds:
            w1.append(weed.x)
            w2.append(weed.y)
        w3 = asarray(w1)
        w4 = asarray(w2)

        for plant in plants:
            p1.append(plant.x)
            p2.append(plant.y)
        p3 = asarray(p1)
        p4 = asarray(p2)

        savetxt("weedx.csv", w3, delimiter=",")
        savetxt("weedy.csv", w4, delimiter=",")
        savetxt("plantx.csv", p3, delimiter=",")
        savetxt("planty.csv", p4, delimiter=",")

       #  with open("weeds.csv") as f:
       #      writer = csv.writer(f)
       #      writer.writerows(weeds)
       # 