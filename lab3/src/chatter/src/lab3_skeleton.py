#!/usr/bin/env python
"""
Lab 3, task 1
"""


import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

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
    g = kfs.prod_exp(xi, theta)
    g_final = g.dot(gst_0)
    return g_final

if __name__ == "__main__":
    print('Lab 3')
    #theta = np.ndarray((7,1))
    theta = np.array([-0.26614566639404297, 1.094878786102295,\
     -0.12348545328369141, -0.317150527532959, 0.1817767231567383, -1.0519273240905762, -1.3230584280395508])
    print lab3(theta)
