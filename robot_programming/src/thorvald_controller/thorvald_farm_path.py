#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Used to move the robot about the workspace without the TOPO-NAV
# This code is no longer in use

def movebase_client(way_point):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    pub = rospy.Publisher('/thorvald_001/current_state_id', String, queue_size=10)
    goal_id = String()
    goal_id.data = way_point[7]
    pub.publish(goal_id)

    goal = MoveBaseGoal()  # publish a new goal based on the pre defined waypoints below
    print(goal)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = way_point[0]
    goal.target_pose.pose.position.y = way_point[1]
    goal.target_pose.pose.position.z = way_point[2]
    goal.target_pose.pose.orientation.x = way_point[3]
    goal.target_pose.pose.orientation.y = way_point[4]
    goal.target_pose.pose.orientation.z = way_point[5]
    goal.target_pose.pose.orientation.w = way_point[6]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node('thorvald_farm_path')
    way_points = [[-5.2, -3.75, 0, 0, 0, 0, 1, "no_row"],  # Start state.
                  [6, -3.75, 0, 0, 0, 0, 1, "basil_row"],

                  [6, -2.70, 0, 0, 0, 1, 0, "no_row"],     # Next row 
                  [-6.5, -2.70, 0, 0, 0, 1, 0, "basil_row"],

                  [-6.5, -0.7, 0, 0, 0, 0, 1, "no_row"],   # Next row  
                  [6, -0.7, 0, 0, 0, 0, 1, "cabbage_row"],

                  [6, 0.24, 0, 0, 0, 1, 0, "no_row"],   # Next row  
                  [-6.5, 0.24, 0, 0, 0, 1, 0, "cabbage_row"],

                  [-6.5, 2.2, 0, 0, 0, 0, 1, "no_row"],   # Next row  
                  [6, 2.2, 0, 0, 0, 0, 1, "onion_row"],

                  [6, 3.2, 0, 0, 0, 1, 0, "no_row"],   # Next row  
                  [-6.5, 3.2, 0, 0, 0, 1, 0, "onion_row"],

                  [-6.5, -3.75, 0, 0, 0, 0, 1, "no_row"]]  # back

    for i in way_points: 
        result = movebase_client(i)
        if result:
            rospy.loginfo("Goal execution done!")