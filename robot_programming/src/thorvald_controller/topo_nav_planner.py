#! /usr/bin/env python
import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal


class move_robot():
    def __init__(self,robotname):
        self.robotname = robotname
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        self.goal = GotoNodeGoal()
        self.points_up = [0, 1]
        self.points_down = [1, 0]

    def move_to_node(self, node_name):  # push out a new node goal for the topo-nav
        self.goal.target = node_name
        self.goal.no_orientation = True
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)

    def move_manuvers(self):
        # self.move_to_node("WPinit")  # - this is a pre-start pose (could be shed or something)
        self.move_to_node("WPstart")  # start and finish at the robots start pose.

        # use the below commented code to move through specific lines of the crop.
        # self.move_to_node("WPline3_0")
        # self.move_to_node("WPline3_1")
        # self.move_to_node("WPline4_1")
        # self.move_to_node("WPline4_0")

        for i in range(1, 4):  # loop to move through all the crop rows.
            j = (i * 2) - 1
            for p in self.points_up:
                self.move_to_node("WPline" + str(j) + "_" + str(p))
            for p in self.points_down:
                self.move_to_node("WPline" + str(j+1) + "_" + str(p))

        self.move_to_node("WPstart")


if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    move_robot("thorvald_001").move_manuvers()

    