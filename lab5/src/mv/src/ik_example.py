#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from baxter_interface import gripper as robot_gripper

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    position_list = [[0.5,0.5,0]]
	position_index = 0
    while not rospy.is_shutdown():
        #raw_input('Press [ Enter ]: ')
		if position_index >= len(position_list):
			break
		position = position_list[position_index]
		position_index += 1
        while True:
            position = raw_input('Press enter to compute an IK solution:(x,y,z)\n')
            position = position.split()
            if len(position) != 3:
                print "error: Please enter 3 floats"
            else:
                position = [float(i) for i in position]
                break
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = position[0]
        request.ik_request.pose_stamped.pose.position.y = position[1]
        request.ik_request.pose_stamped.pose.position.z = position[2]        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
	
	
	rospy.init_node('gripper_test')

	#Set up the left gripper
	left_gripper = robot_gripper.Gripper('left')

	#Calibrate the gripper (other commands won't work unless you do this first)
	print('Calibrating...')
	left_gripper.calibrate()
	rospy.sleep(2.0)
	
	#Open the left gripper
	print('Opening...')
	left_gripper.open()
	rospy.sleep(1.0)
	
	#Close the left gripper
	print('Closing...')
	left_gripper.close()
	rospy.sleep(1.0)
	
	#Open the left gripper
	print('Opening...')
	left_gripper.open()
	rospy.sleep(1.0)
	
	
	
#Python's syntax for a main() method
if __name__ == '__main__':
    main()

