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

#Define the method which contains the main functionality of the node.
def controller():
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame

  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  #pub = rospy.Publisher('7bot/joint_cmd', Float64MultiArray, queue_size=10) ## TODO maybe wrong and need to modify
  pub = rospy.Publisher('7bot/joint_cmd', JointState, queue_size=10) ## TODO maybe wrong and need to modify

#  tfBuffer = tf2_ros.Buffer()
#  tfListener = tf2_ros.TransformListener(tfBuffer)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  flag = 1
  while not rospy.is_shutdown():
    try:
      #trans = tfBuffer.lookup_transform(goal_frame , turtlebot_frame , rospy.Time())
      ## Process trans to get your state error
      ## Generate a control command to send to the robot
      ##print trans.transform
      #x_err = trans.transform.translation.x
      #y_err = trans.transform.translation.y
      #control_command = Twist()
      #control_command.linear.x = x_err * K1;
      #control_command.angular.z = y_err * K2;
      #print x_err,y_err
      #print control_command
      if flag:
        joint_array = [70,115,65,90,90,90,75]
      else:
        joint_array = [70,115,65,90,90,90,0]
      flag = not flag
      joint_cmd = JointState()
      #joint_cmd.layout.dim = [7]

      joint_cmd.position = joint_array


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

  try:
    controller()
  except rospy.ROSInterruptException:
    pass
