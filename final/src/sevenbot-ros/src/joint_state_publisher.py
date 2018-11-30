#!/usr/bin/env python

# reference
# https://github.com/asukiaaa/ros_sevenbot/blob/master/src/serial_node_parser.py
"""
  Fake joint state publisher for testing urdf
"""
import rospy
from sensor_msgs.msg import JointState
import math

class SerialNodeParser:
  degrees = []
  radians = []

  def __init__(self):
    rospy.init_node('joint_state_publisher');
    self.pub = rospy.Publisher('sevenbot/joint_states', JointState, queue_size=10)
    self.joint_state = JointState()
    # self.joint_state.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    self.joint_state.name = ['j1', 'j2', 'j2m', 'j3', 'j4', 'j5', 'j6']
    self.joint_state.position = [70,115,50,90,90,90,0]
    self.callback(self.joint_state)

  def callback(self, origin_joint_states):
    # remove first info because it is angle_type
    self.degrees = origin_joint_states.position[0:-1]
    #self.degrees = origin_joint_states.position[1:]
    self.radians = [ (int(degree) - 90) * math.pi / 180 for degree in self.degrees ]
    radians_with_mimic = self.radians
    radians_with_mimic.insert(2, -self.radians[1])
    self.joint_state.position = radians_with_mimic

  def start_publish(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.joint_state.header.stamp = rospy.get_rostime()
      self.pub.publish(self.joint_state)
      rate.sleep()

if __name__ == '__main__':
  node_parser = SerialNodeParser()
  node_parser.start_publish()
