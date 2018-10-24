#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import Limb
from controller import Controller
# from intera_interface import Limb

def main():
    """
    Main Script
    """
    use_orien_const = False
    input = raw_input("use orientation constraint? y/n\n")
    if input == 'y' or input == 'yes':
        use_orien_const = True
        print("using orientation constraint")
    elif input == 'n' or input == 'no':
        use_orien_const = False
        print("not using orientation constraint")
    else:
        print("input error, not using orientation constraint as default")

    add_box = False
    input = raw_input("add_box? y/n\n")
    if input == 'y' or input == 'yes':
        add_box = True
        print("add_box")
    elif input == 'n' or input == 'no':
        add_box = False
        print("not add_box")
    else:
        print("input error, not add_box as default")

    open_loop_contro = False
    input = raw_input("using open loop controller? y/n\n")
    if input == 'y' or input == 'yes':
        open_loop_contro = True
        print("using open loop controller")
    elif input == 'n' or input == 'no':
        open_loop_contro = False
        print("not using open loop controller")
    else:
        print("input error, not using open loop controller as default")
    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

    # joint_names = ['head_pan', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    my_controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    ##
    ## Add the obstacle to the planning scene here
    ##
    if add_box:
        name = "obstacle_1"
        size = np.array([0.4, 1.2, 0.1])
        pose = PoseStamped()
        pose.header.frame_id = "base"
        #x, y, and z position
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        #Orientation as a quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        planner.add_box_obstacle(size, name, pose)

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = 0.4
                goal_1.pose.position.y = -0.3
                goal_1.pose.position.z = 0.2

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0
                if not use_orien_const:
                    plan = planner.plan_to_pose(goal_1, list())
                else:
                    plan = planner.plan_to_pose(goal_1, [orien_const])

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                if open_loop_contro:
                    if not my_controller.execute_path(plan):
                        raise Exception("Execution failed")
                else:
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = 0.6
                goal_2.pose.position.y = -0.3
                goal_2.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                # plan = planner.plan_to_pose(goal_2, list())

                if not use_orien_const:
                    plan = planner.plan_to_pose(goal_2, list())
                else:
                    plan = planner.plan_to_pose(goal_2, [orien_const])

                raw_input("Press <Enter> to move the right arm to goal pose 2: ")
                if open_loop_contro:
                    if not my_controller.execute_path(plan):
                        raise Exception("Execution failed")
                else:
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
                # if not my_controller.execute_path(plan):
                #     raise Exception("Execution failed")
                # if not planner.execute_plan(plan):
                #     raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.1

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                # plan = planner.plan_to_pose(goal_3, list())
                if not use_orien_const:
                    plan = planner.plan_to_pose(goal_3, list())
                else:
                    plan = planner.plan_to_pose(goal_3, [orien_const])

                raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                if open_loop_contro:
                    if not my_controller.execute_path(plan):
                        raise Exception("Execution failed")
                else:
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")

                # if not my_controller.execute_path(plan):
                #     raise Exception("Execution failed")

                # if not planner.execute_plan(plan):
                #     raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
