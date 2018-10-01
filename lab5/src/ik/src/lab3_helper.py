#!/usr/bin/env python

import numpy as np
import scipy as sp

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given
    the twists and displacements for each joint.

    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint

    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    #YOUR CODE HERE
    g = homog_3d(xi[0:6, 0],theta[0])
    for i in range(1, xi.size / 6):
        g =g.dot(homog_3d(xi[0:6, i],theta[i]))
    return g

def lab3(theta):
    q = np.ndarray((3,8))
    w = np.ndarray((3,7))
    #print w
    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T

    # YOUR CODE HERE
    gst_0 = np.ndarray((4,4))
    gst_0[0:3, 0:3] = R
    gst_0[0:3, 3] = np.transpose(q[0:3,7])
    gst_0[3][3] = 1
    #print "gst_", gst_0

    xi = np.ndarray((6,7))
    for i in range(0,7):
        xi[0:3, i] = -np.cross(w[0:3, i], q[0:3, i])
        xi[3:6, i] = w[0:3, i]
       # print "xi", i,xi[0:6, i]
    g = prod_exp(xi, theta)
    g_final = g.dot(gst_0)
    return g_final

## forward kinematics
# Input sensor_msgs/JointState
def forward_kinematics(message):

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
    #rate = rospy.Rate(100.0)
    print "time: ",  message.header.stamp
    print "translation: ", trans[0:3, 3]
    Roll = math.atan(trans[2][1] / trans[2][2])
    Pitch = math.atan(-trans[2][0]/ \
        math.sqrt(trans[2][1]*trans[2][1]+trans[2][2]*trans[2][2]))
    Yaw = math.atan(trans[1][0]/ trans[0][0])
    print "RPY ",[Roll,Pitch,Yaw],"\n"
