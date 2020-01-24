#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from std_srvs.srv import Empty
from std_msgs.msg import String
from uuid import uuid4

BOX_SDF="""
<?xml version='1.0'?>
<sdf version="1.4">
<model name="killbox">
  <pose>0 0 0 0 0 0</pose>
  <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.00083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.00083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.0000083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>.1 .1 .01</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>.1 .1 .01</size>
          </box>
        </geometry>
        <material>
            <ambient>1 0 0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class Sprayer:
    # The request.initial_pose.position.y is the distance from the center of the sprayer
    def __init__(self):
        self.sdf = BOX_SDF
        #define the services
        rospy.Service('spray1', Empty, self.spray1)
        rospy.Service('spray2', Empty, self.spray2)
        rospy.Service('spray3', Empty, self.spray3)
        rospy.Service('spray4', Empty, self.spray4)
        rospy.Service('spray5', Empty, self.spray5)
        rospy.Service('spray6', Empty, self.spray6)
        rospy.Service('spray7', Empty, self.spray7)

        #call the service to spawn a little rectangle on gazebo
        self.spawner = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel)


    def spray1(self, r):
   
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = 0.3
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray2(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = 0.2
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray3(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = 0.1
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray4(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = 0
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray5(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = -0.1
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray6(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = -0.2
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

    def spray7(self, r):
        request = SpawnModelRequest()
        request.model_name = 'killbox_%s' % uuid4()
        request.model_xml = self.sdf
        request.reference_frame = 'thorvald_001/base_link'
        request.initial_pose.position.z = 0.005
        request.initial_pose.position.x = -0.45
        request.initial_pose.position.y = -0.3
        request.initial_pose.orientation.w = 1.0
        self.spawner(request)
        return []

if __name__ == "__main__":
    rospy.init_node('sprayer')
    Sprayer()
    rospy.spin()
