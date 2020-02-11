#!/usr/bin/env python
import csv
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Used to move the robot about the workspace without the TOPO-NAV
# This code is no longer in use

def movebase_client(way_point, orientation):
    client = actionlib.SimpleActionClient('thorvald_001/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()  # publish a new goal based on the pre defined waypoints below
    print(goal)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = way_point[0]
    goal.target_pose.pose.position.y = way_point[1]
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    rospy.init_node('thorvald_farm_path')

    with open('extended_sorted_rows.csv', mode='r') as csv_file:
        reader = csv.reader(csv_file)
        # Print every value of every row. 
        wp_list = []
        for row in reader:
            wp_row = []
            for list_ in row:
                wp = list_.split(",")
                wp[0] = float(wp[0].replace('[', '').replace(']', '').replace(' ', ''))
                wp[1] = float(wp[1].replace('[', '').replace(']', '').replace(' ', ''))
                wp_row.append(wp)
            wp_list.append(wp_row)

    # now move through the way points
    # for row in wp_list:
    #     for wp in row:
    #         print wp

    n = 2
    for i in range(0,len(wp_list)):
        orientation = [0,0,1,0]
        if i % n == 1:
            wp_list[i].reverse()
            orientation = [0,0,0,1]
        for waypoint in wp_list[i]:
            print(waypoint)
            result = movebase_client(waypoint, orientation)
            if result:
                rospy.loginfo("Goal execution done!")
