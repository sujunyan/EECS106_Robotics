#!/usr/bin/env python

import numpy as np
import scipy as sp


def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.

    Args:
    omega - (3,) ndarray: the rotation vector

    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    omega_hat = np.array([[0, -omega[2], omega[1]],[omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])

    #YOUR CODE HERE

    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.

    Args:
    theta: the angle of rotation

    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """

    #YOUR CODE HERE
    rot = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    return rot

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.

    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation

    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    #YOUR CODE HERE
    omega_hat = skew_3d(omega)
    rot = linalg.expm(omega_hat*theta)
    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation

    Args:
    xi - (3,) ndarray: the 2D twist

    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    #YOUR CODE HERE
    xi_hat = np.array([[0, -xi[2], xi[0]], [xi[2], 0, xi[1]], [0,0,0]])
    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation

    Args:
    xi - (6,) ndarray: the 3D twist

    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    xi_hat = np.zeros((4,4))
    v = xi[0 : 3]
    omega = xi[3 : 6]
    omega_hat = skew_3d(omega)
    xi_hat[0:3, 0:3] = omega_hat
    #print(np.transpose(v))
    xi_hat[0:3, 3] = np.transpose(v)
    #print xi
    #print xi_hat
    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a
    joint displacement

    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement

    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    #YOUR CODE HERE
    xi_hat = hat_2d(xi)
    g = linalg.expm(xi_hat*theta)
    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a
    joint displacement.

    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    xi_hat = hat_3d(xi)
    g = linalg.expm(xi_hat*theta)
    return g

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

    mydict = {i:j for i in message.name, for j in message.position}
    #Print the contents of the message to the console
    theta = np.ndarray((7,1))
    #print "position", message.position[8]
    #print theta
    '''
    theta[0] = message.position[4]
    theta[1] = message.position[5]
    theta[2] = message.position[2]
    theta[3] = message.position[3]
    theta[4] = message.position[6]
    theta[5] = message.position[7]
    theta[6] = message.position[8]
    '''

    theta[0] =  mydict['left_s0']
    theta[1] = mydict['left_s1']
    theta[2] = mydict['left_e0']
    theta[3] = mydict['left_e1']
    theta[4] = mydict['left_w0']
    theta[5] = mydict['left_w1']
    theta[6] = mydict['left_w2']

    trans = lab3(theta)
    #rate = rospy.Rate(100.0)
    print "time: ",  message.header.stamp
    print "translation: ", trans[0:3, 3]
    Roll = math.atan(trans[2][1] / trans[2][2])
    Pitch = math.atan(-trans[2][0]/ \
        math.sqrt(trans[2][1]*trans[2][1]+trans[2][2]*trans[2][2]))
    Yaw = math.atan(trans[1][0]/ trans[0][0])
    print "RPY ",[Roll,Pitch,Yaw],"\n"
