#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

import tf2_ros
import tf
import sys
import math

class NavTest():
    def __init__(self):

        rospy.init_node('nav_turtlebot', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Distance to move forward
        self.dist_mv_fwd = 1    # [m]

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED',
                       'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                       'RECALLED', 'LOST']
        self.sequence = ['target']

        # self.odometry_subscriber = rospy.Subscriber(
        #     '/odom', Odometry, self.update_initial_pose)

        self.init_pose_sub = rospy.Subscriber(
            'initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher(
            'cmd_vel', Twist, queue_size=5)

        self.move_forw_sub = rospy.Subscriber(
            '/move_forward', Bool, self.move_bot_straight)

        self.fail_goal_pub = rospy.Publisher(
            '/custom_goal/fail', Bool, queue_size=10)

        self.attempt_goal_pub = rospy.Publisher(
            '/custom_goal/attempt', Bool, queue_size=10)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # self.initial_pose = Pose()
        self.initial_pose = PoseWithCovarianceStamped()

        # Try to get the initial pose before asking the user to select it in rviz
        # This avoids messing up the Amcl Particle Swarm.
        test_frame = self.frametrans('base_link', 'map')
        if test_frame == False:

            # # Get the initial pose from the user
            rospy.loginfo("Click on the map in RViz to set the intial pose...")
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

            # return

        # rospy.loginfo("Obtained initial pose from odometry")

        # Initialize a last known pose of the turtlebot.
        self.last_location = Pose()
        
        # # Guess the initial pose is at the map frame with no rotation
        # self.initial_pose.pose.pose.position.x = 0
        # self.initial_pose.pose.pose.position.y = 0
        # self.initial_pose.pose.pose.position.z = 0

        # self.initial_pose.pose.pose.orientation.x = 0
        # self.initial_pose.pose.pose.orientation.y = 0
        # self.initial_pose.pose.pose.orientation.z = 0
        # self.initial_pose.pose.pose.orientation.w = 1

        # self.initial_pose.pose.pose.position.x = trans.transform.translation.x
        # self.initial_pose.pose.pose.position.y = trans.transform.translation.y
        # self.initial_pose.pose.pose.position.z = trans.transform.translation.z

        # self.initial_pose.pose.pose.orientation.x = trans.transform.rotation.x
        # self.initial_pose.pose.pose.orientation.y = trans.transform.rotation.y
        # self.initial_pose.pose.pose.orientation.z = trans.transform.rotation.z
        # self.initial_pose.pose.pose.orientation.w = trans.transform.rotation.w

        # Make sure we have the initial pose
        while self.initial_pose.header.stamp == "":
            rospy.sleep(1)
        rospy.loginfo("Starting navigation test")

    # def move_bot_frame(self, target_frame):

    #     trans = self.frametrans(target_frame, 'map')
    #     if trans == False:
    #         return

    #     # Set up the goal locations. Poses are defined in the map frame.
    #     # An easy way to find the pose coordinates is to point-and-click
    #     # Nav Goals in RViz when running in the simulator.
    #     # Pose coordinates are then displayed in the terminal
    #     # that was used to launch RViz.
    #     locations = dict()

    #     rospy.loginfo('Moving to target frame')
        
    #     locations['target'] = Pose(Point(trans.transform.translation.x,
    #                                     trans.transform.translation.y, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    #     self.move_bot()

    def move_bot_straight(self, forward):
        
        if forward.data == True:
            trans = self.frametrans('base_link', 'map')
            if trans == False:
                return

            linear_pos  = trans.transform.translation
            orientation = trans.transform.rotation
            # dist_x = self.dist_mv_fwd*math.cos(orientation.z)
            # dist_y = self.dist_mv_fwd*math.sin(orientation.z)

           # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation)
            current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                               orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
	    dist_x = self.dist_mv_fwd*math.cos(current_z)
            dist_y = self.dist_mv_fwd*math.sin(current_z)

            # if abs(direc) == 1:
            new_target_point = Point(linear_pos.x + dist_x,
                                     linear_pos.y + dist_y,
                                     linear_pos.z)
            # else:
            #     print 'Must specify if moving forward or backward'
            #     return

            # Set up the goal locations. Poses are defined in the map frame.
            # An easy way to find the pose coordinates is to point-and-click
            # Nav Goals in RViz when running in the simulator.
            # Pose coordinates are then displayed in the terminal
            # that was used to launch RViz.
            # location = []

            rospy.loginfo('Forward movement goal commanded')

            location = Pose(new_target_point, orientation)

            self.move_bot(location)

        # elif forward.data == False:

        #     # Stop the turtlebot from moving if we tell it to stop moving forward
        #     # This makes velocity commands override goal commands.
        #     self.shutdown()

    def move_bot(self, location):

        # Variables to keep track of success rate, running time, and distance traveled
        # n_locations = len(locations)
        n_goals = 1
        n_successes = 0
        # i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        # location = ""
        # last_location = ""

        # Begin the main loop and run through a self.sequence of locations

        # Get the next location in the current self.sequence
        # location = self.sequence[0]
        last_location = location

         # Keep track of the distance traveled.
         # Use updated initial pose if available.
        # if self.initial_pose.header.stamp == "":
        #     distance = sqrt(pow(location.position.x
        #                           - location.position.x, 2) +`
        #                       pow(location.position.y -
        #                           location.position.y, 2))
        # else:
        rospy.loginfo("Updating current pose.")
        distance = sqrt(pow(location.position.x
                            - self.initial_pose.pose.pose.position.x, 2) +
                        pow(location.position.y -
                            self.initial_pose.pose.pose.position.y, 2))
        self.initial_pose.header.stamp = ""
        
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = location
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(location))

        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(
            rospy.Duration(30))

        # Tell the controller that the goal was attempted
        attempt_goal = Bool()
        attempt_goal.data = True
        self.attempt_goal_pub.publish(attempt_goal)

        # finished_within_time = self.move_base.wait_for_result(
        #     rospy.Duration(300))
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
	    # Tell the controller that the goal failed
            failed_goal = Bool()
            failed_goal.data = True
            self.fail_goal_pub.publish(failed_goal)

        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                n_successes += 1
                distance_traveled += distance
            else:
                rospy.loginfo(
                    "Goal failed with error code: " + str(self.goal_states[state]))

                # Tell the controller that the goal failed
                failed_goal = Bool()
                failed_goal.data = True
                self.fail_goal_pub.publish(failed_goal)

                # raise ValueError('Failed goal --> Likely hit a wall')

        # How long have we been running?
        running_time = rospy.Time.now() - start_time
        running_time = running_time.secs / 60.0
        # Print a summary success/failure, distance traveled and time elapsed
        rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                      str(n_goals) + " = " + str(100 * n_successes / n_goals) + "%")
        rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                      " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
        # rospy.sleep(self.rest_time)  

    def update_initial_pose(self, init_pose):
        self.initial_pose = init_pose
        # self.initial_pose = Pose()
        # self.initial_pose = odom.pose.pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def frametrans(self, current_frame, target_frame):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        r = rospy.Rate(0.5)
        time = 0
        while(True):
            try:
                trans = tfBuffer.lookup_transform(
                    target_frame, current_frame, rospy.Time())
                # trans = tfBuffer.lookup_transform('base_link','object_1green_1', rospy.Time())

                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print "tf errors occur in navigation"
                time += 1
                if time > 5:
                    return False
            r.sleep()
        return trans


def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    try:

        new_nav = NavTest()

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
