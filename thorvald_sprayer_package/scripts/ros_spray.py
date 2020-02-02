#!/usr/bin/env python

import math
import rospy
import tf
# Ros Messages
from sensor_msgs.msg import PointCloud
from uol_cmp9767m_base.srv import y_axes_diff, y_axes_diffRequest
from geometry_msgs.msg import Point32


class Sprayer():

    def __init__(self, robot):
        self.robot = robot
        self.sprayed = []  # Keep a list of sprayed points
        self.real_sprayed = []
        self.spray_srv = rospy.ServiceProxy(
            "{}/dynamic_sprayer".format(self.robot),
            y_axes_diff)

        self.sub = rospy.Subscriber(
            "/weed/allpoints/{}".format(self.robot),
            PointCloud,
            self.spray_weed_callback)

        self.tflistener = tf.listener.TransformListener()

        self.sprayed_points_msg = PointCloud()
        self.sprayed_points_pub = rospy.Publisher(
            "/weed/sprayed_points/{}".format(self.robot),
            PointCloud,
            queue_size=5)


    def spray_weed_callback(self, data):
        time = rospy.Time(0)
        # Get the sprayer position in the map coordinates
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
            # print(trans, rot)
        except Exception:
            return

        # Iterate every detected Weed
        for point in data.points:
            # Difference in 'x' and 'y' frame
            dx = abs(trans[0] - point.x)
            dy = trans[1] - point.y  # this creates mirror
            # Deside to spray
            if dx < 0.01 and abs(dy) < 0.5:
                if point not in self.sprayed:
                    self.sprayed.append(point)
                    real_point = Point32(trans[0], trans[1] - dy, point.z)
                    self.real_sprayed.append(real_point)
                    print('spray!!!')
                    # Add delay
                    req = y_axes_diffRequest()
                    req.y_diff = dy
                    self.spray_srv(req)

        # Publish the Sprayed Points
        self.sprayed_points_msg.points = self.real_sprayed
        self.sprayed_points_msg.header.frame_id = 'map'
        self.sprayed_points_msg.header.stamp = time
        self.sprayed_points_pub.publish(self.sprayed_points_msg)    

if __name__ == '__main__':
    rospy.init_node('spray_node', anonymous=True)
    Sprayer('thorvald_001')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"