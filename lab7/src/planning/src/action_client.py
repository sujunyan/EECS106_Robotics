#!/usr/bin/env python
import roslib; roslib.load_manifest('planning')

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, Constraints

def main():
    #Initialize the node
    rospy.init_node('moveit_client')
    
    # Create the SimpleActionClient, passing the type of the action
    # (MoveGroupAction) to the constructor.
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

    # Wait until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveGroupGoal()
    
    #----------------Construct the goal message (start)----------------
    joint_names = ['head_pan', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    
    #Set parameters for the planner    
    goal.request.group_name = 'both_arms'
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = 5.0
    
    #Define the workspace in which the planner will search for solutions
    goal.request.workspace_parameters.min_corner.x = -1
    goal.request.workspace_parameters.min_corner.y = -1
    goal.request.workspace_parameters.min_corner.z = -1
    goal.request.workspace_parameters.max_corner.x = 1
    goal.request.workspace_parameters.max_corner.y = 1
    goal.request.workspace_parameters.max_corner.z = 1
    
    goal.request.start_state.joint_state.header.frame_id = "base"
    
    #Set the start state for the trajectory
    goal.request.start_state.joint_state.name = joint_names
        
    '''
    changed by zishu yu start
    '''
    # goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # goal.request.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    position = raw_input("please input %d start position for " + str(joint_names) %len(joint_names))
    position =  position.split()
    while (not (len(position) == len(joint_names))):
        position = raw_input("please input %d start position for " + str(joint_names)%len(joint_names))
        position =  position.split()
    velocity = raw_input("please input %d start position for " + str(joint_names)%len(joint_names))
    velocity =  velocity.split()
    while (not (len(velocity) == len(joint_names))):
        velocity = raw_input("please input %d start velocity for " + str(joint_names)%len(joint_names))
        velocity =  velocity.split()

    position = [float(position[i]) for i in range(len(position))]
    velocity = [float(velocity[i]) for i in range(len(velocity))]
    goal.request.start_state.joint_state.position = position
    goal.request.start_state.joint_state.velocity = velocity

    '''
    changed by zishu yu end
    '''

    #Tell MoveIt whether to execute the trajectory after planning it
    goal.planning_options.plan_only = True
    
    #Set the goal position of the robot
    #Note that the goal is specified with a collection of individual
    #joint constraints, rather than a vector of joint angles
    arm_joint_names = joint_names[1:]
    '''
    changed by zishu yu start
    '''
    # target_joint_angles = [0.5, -0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_joint_angles = raw_input("please input %d end position for " + str(joint_names)%len(arm_joint_names))
    target_joint_angles =  target_joint_angles.split()
    while (not (len(target_joint_angles) == len(arm_joint_names))):
        target_joint_angles = raw_input("please input %d end position for " + str(joint_names)%len(arm_joint_names))
        target_joint_angles =  target_joint_angles.split()
    target_joint_angles = [float(target_joint_angles[i]) for i in range(len(target_joint_angles))]
    
    '''
    changed by zishu yu end
    '''
    tolerance = 0.0001
    consts = []
    for i in range(len(arm_joint_names)):
        const = JointConstraint()
        const.joint_name = arm_joint_names[i]
        const.position = target_joint_angles[i]
        const.tolerance_above = tolerance
        const.tolerance_below = tolerance
        const.weight = 1.0
        consts.append(const)
        
    goal.request.goal_constraints.append(Constraints(name='', joint_constraints=consts))
    #---------------Construct the goal message (end)-----------------

    # Send the goal to the action server.
    client.send_goal(goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    # Print out the result of executing the action
    print(client.get_result())
    

if __name__ == '__main__':
    main()
