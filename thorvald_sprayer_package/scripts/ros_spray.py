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
        self.sprayed = []  # Keep a list of sprayed points for rviz
        self.real_sprayed = [] #
        self.last_spray_pos = 0

        self.spray_srv = rospy.ServiceProxy(
            "{}/dynamic_sprayer".format(self.robot),
            y_axes_diff)

        # self.sub = rospy.Subscriber(
        #     "/weed_map/pointcloud".format(self.robot),
        #     PointCloud,
        #     self.spray_weed_callback)

        self.sub = rospy.Subscriber(
            "/weed/allpoints/{}".format(self.robot),
            PointCloud,
            self.spray_weed_callback)

        self.tflistener = tf.listener.TransformListener()

        self.sprayed_points_msg = PointCloud()
        self.sprayed_points_pub = rospy.Publisher(
            "{}/weed/sprayed_points/".format(self.robot),
            PointCloud,
            queue_size=5)

    def slep(self, sprayer_distance):

        total_travel_speed = 5
        slep_time = sprayer_distance / total_travel_speed
        rospy.sleep(slep_time)

    def spray_weed_callback(self, data):
        time = rospy.Time(0)
        # Get the sprayer position in the map coordinates
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
        except Exception:
            return

        # First initialise current_y_sprayer = result from tf
        # it's going to be updated according to the place it travels to.
        current_y_sprayer = trans[1]

        # Iterate every detected Weed
        for point in data.points:
            # pos of robot
            x_sprayer = trans[0]

            # Difference in 'x' and 'y' frame
            dx = abs(x_sprayer - point.x)
            dy = current_y_sprayer - point.y  # this creates mirror effect

            # If you see any weeds in current crop line
            # (crop line has a width of 1 meter thats why <0.5)
            if abs(trans[1] - point.y) < 0.5:
            	# TODO smart spray

                # find difference distance between point and current sprayer position
                # sprayer_dist = current_y_sprayer - point.y
                # print("I found weed with distance, ",
                #       current_y_sprayer, point.y, sprayer_dist)
                
                # When sprayer moves, sleep for travel time
                #self.slep(sprayer_dist)

                # After moving (sleeping) update the current sprayer position to it's new position
                current_y_sprayer = point.y

                if dx < 0.04: # Same as killbox radius
                    if point not in self.sprayed:

                        # TODO fix mirroring
                        # TODO if spray radius hits many points add them to the real_sprayed points

                    	# add point in sprayed array
                        self.sprayed.append(point)

                        # save the position of the sprayer (visualise in rviz)
                        real_point = Point32(
                            x_sprayer, current_y_sprayer, point.z)
                        self.real_sprayed.append(real_point)

                        # Initialise request and assign to it the difference in y axis
                        # between the sprayer and the weed
                        req = y_axes_diffRequest()
                        req.y_diff = dy

                        # Call service to spray
                        self.spray_srv(req)
                        print('I sprayied!!!', dy)

            # Publish the Sprayed Points
            self.sprayed_points_msg.points = self.real_sprayed
            self.sprayed_points_msg.header.frame_id = 'map'
            self.sprayed_points_msg.header.stamp = time
            self.sprayed_points_pub.publish(self.sprayed_points_msg)


if __name__ == '__main__':
    rospy.init_node('spray_node', anonymous=True)
    Sprayer('thorvald_002')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
