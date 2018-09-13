#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import math 
from sensor_msgs.msg import JointState
from lab3_skeleton import lab3
import numpy as np
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    theta = np.ndarray((7,1))
    #print "position", message.position[8]
    #print theta
    theta[0] = message.position[4]
    theta[1] = message.position[5]
    theta[2] = message.position[2]
    theta[3] = message.position[3]
    theta[4] = message.position[6]
    theta[5] = message.position[7]
    theta[6] = message.position[8]

    trans = lab3(theta)
    rate = rospy.Rate(10.0)
    print "time: ", message.header.stamp
    print "translation: ", trans[0:3, 3]
    print "alpha: ", math.atan(trans[1][0]/ trans[0][0])
    print "beta ", math.atan(-trans[2][0]/ \
        math.sqrt(trans[2][1]*trans[2][1]+trans[2][2]*trans[2][2]))
    print "gama: ", math.atan(trans[2][1] / trans[2][2])
    rate.sleep()
    #print(rospy.get_name())
    
    #print("velocity",message.velocity)
    #print("effort", message.effort)

#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("robot/joint_states", JointState, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':
    listener()
