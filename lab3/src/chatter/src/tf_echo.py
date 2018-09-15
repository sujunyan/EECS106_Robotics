#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
def tf_echo():
    rospy.init_node('tf2_left_hand_listener')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("base","left_hand",  rospy.Time())
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, \
            tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #print type(trans.transform)
        print "time :", trans.header.stamp
        print "translation: ",trans.transform.translation.x,\
            trans.transform.translation.y,\
            trans.transform.translation.z
        print "rotation: ", trans.transform.rotation.x,\
            trans.transform.rotation.y,\
            trans.transform.rotation.z,\
            trans.transform.rotation.w

        #rate.sleep()
    
    
#Python's syntax for a main() method
if __name__ == '__main__':
    tf_echo()
