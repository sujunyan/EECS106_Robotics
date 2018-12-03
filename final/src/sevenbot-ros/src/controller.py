#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys

#from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from math import *

# Inverse Kinematics for sevenbot from official github
# Input : translate from base_link to goal position
def IK(trans):
  # constant config for sevenbot
  a=120.0; b=40.0; c=198.50; d=30.05; e=77.80; f=22.10; g=12.0; h = 29.42;
  PI = pi
  (x,y,z) = trans
  print (x,y,z)
  theta = [0,0,0]
  theta[0] = atan(y / x);
  if (theta[0] < 0):
    theta[0] = PI + theta[0];
  x -= d * cos(theta[0]);
  y -= d * sin(theta[0]);
  z -= e;
  lengthA = sqrt(x * x + y * y + z * z);
  lengthC = sqrt(h * h + c * c);
  offsetAngle = atan(h / c);
  angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
  angleB = atan( z / sqrt(x * x + y * y) );
  angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
  theta[1] = angleB + angleC;
  theta[2] = PI - angleA - angleB - angleC + offsetAngle;
  theta[2] += 1.134464;

  # range check
  thetaMin = [ 0,  0, -1.134464,  0.17453292,  0,  0, 0];
  thetaMax = [PI, PI, 2.0071287, 2.9670596, PI, PI, PI/2];
  print theta
  if (theta[1] > thetaMin[1] and theta[1] < thetaMax[1] and
      theta[2] > thetaMin[2] and theta[2] < thetaMax[2]
      and theta[2] - 0.8203047 + theta[1] < PI and theta[2] + theta[1] > 1.44862327):
      print theta
      return theta
  return None



#Define the method which contains the main functionality of the node.
def controller(name ,cur_frame,goal_frame):
  #pub = rospy.Publisher('sevenbot/joint_cmd', JointState, queue_size=10) ## TODO maybe wrong and need to modify
  pub = rospy.Publisher('sevenbot/joint_cmd', JointState, queue_size=10) ## TODO maybe wrong and need to modify
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  r = rospy.Rate(0.2) # 1hz

  # Loop until the node is killed with Ctrl-C
  flag = 1
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(goal_frame , cur_frame , rospy.Time(0))
      #print trans.transform
      pos = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z]
      pos = [1000 * i for i in pos]
      print pos
      try:
        theta = IK(pos)
      except:
        print ("IK failed")
        continue
      if(not theta):
        print ("IK failed")
        continue
      print theta

      if flag:
        joint_array = [70,115,50,90,90,90,90]
      else:
        joint_array = [70,115,80,90,90,90,160]
      flag = not flag
      #joint_array = theta + [radians(i) for i in joint_array[3:]]
      joint_array = [degrees(i) for i in theta] + joint_array[3:]
      print joint_array

      joint_cmd = JointState()
      joint_cmd.header.stamp = rospy.Time.now()
      #joint_cmd.layout.dim = [7]

      joint_cmd.position = joint_array


      #################################### end your code ###############

      pub.publish(joint_cmd)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print "tf errors occur"
    # Use our rate object to sleep until it is time to publish again
    r.sleep()



pub = rospy.Publisher('sevenbot/joint_cmd', JointState, queue_size=10) ## TODO maybe wrong and need to modify

first_position = [91.94244384765625, 179.35252380371094, 46.40287780761719, 93.45323944091797, 180.0, 1.726618766784668, 90]
second_position = [92.37409973144531, 140.2877655029297, 42.30215835571289, 87.19424438476562, 180.0, 1.5107914209365845, 90]
third_position = [92.80575561523438, 117.48201751708984, 61.2230224609375, 87.41007232666016, 175.0, 1.72, 90]
#third_position = [90, 104.78, 71.66, 87.41007232666016, 180, 1.51, 90]

#
#Define the method which contains the main functionality of the node.
def controller_wo_frame():
  r = rospy.Rate(1) # 1hz
  desire_joint = [102.3021,99.928,71.6546,89.78,179.568,7.1223,90]
  joint_array = [101,180,45,96,180,10,90]
  joint_array[0] = desire_joint[0]
  pushlish_times = 5
  joint_cmd = JointState()
  for i in range(pushlish_times):
    joint_cmd.header.stamp = rospy.Time.now()
    joint_cmd.position = first_position
    pub.publish(joint_cmd)
    r.sleep()
  print "first joint done"

  for i in range(pushlish_times):
    joint_cmd.header.stamp = rospy.Time.now()
    joint_cmd.position = second_position
    pub.publish(joint_cmd)
    r.sleep()

  print "second joint done"

  for i in range(pushlish_times):
    joint_cmd.header.stamp = rospy.Time.now()
    joint_cmd.position = third_position
    pub.publish(joint_cmd)
    r.sleep()
  print "third joint done"
  print "entering the infinite loop"


  while not rospy.is_shutdown():
    try:
      #joint_array = [102.3021,99.928,71.6546,89.78,179.568,7.1223,90]
      print joint_cmd.position

      joint_cmd.header.stamp = rospy.Time.now()
      #joint_cmd.layout.dim = [7]
      #joint_cmd.position = joint_array


      #################################### end your code ###############

      pub.publish(joint_cmd)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print "tf errors occur"
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph
  #called /turtlebot_controller.
  rospy.init_node('sevenbot_controller', anonymous=True)
  tmp = IK((0.001,200,200))
  print [degrees(i) for i in tmp]

  try:
    pass
    controller_wo_frame()
  except rospy.ROSInterruptException:
    pass
