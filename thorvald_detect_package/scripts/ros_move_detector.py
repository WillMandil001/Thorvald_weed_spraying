#!/usr/bin/env python
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveRobot():

    def __init__(self, robot):
        self.robot = robot

    def movebase_client(self, x, y, z):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print('Done moving to: {}, {}, {}'.format(x, y, z))
            return client.get_result()


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('move_robot', anonymous=True)
    mv_robot = MoveRobot('thorvald_001')

    mv_robot.movebase_client(-6, 0.05, -90)
    mv_robot.movebase_client(6, 0.05, 0)
    
    mv_robot.movebase_client(6, -1.3, 90)
    mv_robot.movebase_client(-6, -1.3, 90)
    mv_robot.movebase_client(-8, -4, 180)



    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
