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
        self.last_spray_pos = 0

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

    def slep(self, sprayer_distance):

        total_travel_speed = 10
        slep_time = sprayer_distance/total_travel_speed
        rospy.sleep(slep_time)

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
        
        # First initialise current_y_sprayer = result from tf
        current_y_sprayer = trans[1] # it's going to be updated according to the place it travels to.

        print("bich lasagnia")
        # Iterate every detected Weed
        # for point in data.points:
        # pos of robot
        x_sprayer = trans[0] 

        # Difference in 'x' and 'y' frame
        # dx = abs(x_sprayer - point.x)
        # dy = current_y_sprayer - point.y  # this creates mirror effect

        dx = abs(x_sprayer - (-2.5))
        dy = current_y_sprayer - (-7.15)  # this creates mirror effect
        
        # Deside to spray
        print("yeah")

        if abs(dy) < 0.5:
            #find distance between point and current sprayer position
            sprayer_dist = current_y_sprayer - (-7.15) #point.y
            print("I found weed with distance, ", sprayer_dist, current_y_sprayer)
            # when sprayer moves sleep for travel time
            self.slep(sprayer_dist)

            # After moving (sleeping) update the current sprayer position to it's new position
            current_y_sprayer = (-7.5) #point.y

            if dx < 0.05 and abs(dy) < 0.5:
                # if point not in self.sprayed:
                # calculate euclidean dist between robot and weed
                # dist = math.sqrt(math.pow(x_sprayer - point.x,2)+ math.pow(y_sprayer - point.y,2))

                # self.sprayed.append(point)  # add point in sprayed array
                # save the position of the sprayer (visualise in rviz)
                #TODO fix mirroring
                real_point = Point32(x_sprayer,  dy, 0)#point.z) 
                self.real_sprayed.append(real_point)


                req = y_axes_diffRequest()
                req.y_diff = dy
                self.spray_srv(req)
                print('I sprayied!!!', dy)

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