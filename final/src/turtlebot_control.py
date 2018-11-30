#!/usr/bin/env python
# The line above tells Linux that this file is a Python script,
# and that the OS should use the Python interpreter in /usr/bin/env
# to run it. Don't forget to use "chmod +x [filename]" to make
# this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from auto_navigation import NavTest
PI = 3.1415926535897


class Controller:
    # Define the method which contains the main functionality of the node.

    def __init__(self):
        self.velocity_publisher = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        self.odometry_init_check = False
        self.odometry = Odometry()
        self.init_odometry = Odometry()

    def controller(self, turtlebot_frame, goal_frame):
        """
        Controls a turtlebot whose position is denoted by turtlebot_frame,
        to go to a position denoted by target_frame

        Inputs:
        - turtlebot_frame: the tf frame of the AR tag on your turtlebot
        - target_frame: the tf frame of the target AR tag
        """

        ################################### YOUR CODE HERE ##############

        # Create a publisher and a tf buffer, which is primed with a tf listener
        # pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10) ## TODO maybe wrong and need to modify
        # TODO maybe wrong and need to modify
        pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)

        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        r = rospy.Rate(0.5)  # 10hz
        r_rotation = rospy.Rate(0.4)
        K1 = 0.3
        K2 = 1
        # Loop until the node is killed with Ctrl-C
        finded_check = False
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(
                    goal_frame, turtlebot_frame, rospy.Time())

                NavTest()
                # # Process trans to get your state error
                # # Generate a control command to send to the robot
                # #print trans.transform
                # x_err = trans.transform.translation.x
                # y_err = trans.transform.translation.y

                # control_command = Twist()
                # control_command.linear.x = 0
                # control_command.linear.y = 0
                # control_command.linear.z = 0
                # control_command.angular.x = 0
                # control_command.angular.y = 0
                # print("x_err", x_err, "y_err", y_err)
                # # control_command.linear.x = x_err * K1;
                # # control_command.angular.z = y_err * K2;

                # print x_err, y_err
                # finded_check = True
                # #print control_command

                # #################################### end your code ###############

                # pub.publish(control_command)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print "tf errors occur"
                if (not finded_check):
                    self.rotate(90, 15, 1)
                    r_rotation.sleep()
                control_command = Twist()
                control_command.linear.x = 0
                control_command.linear.y = 0
                control_command.linear.z = 0
                control_command.angular.x = 0
                control_command.angular.y = 0
                control_command.angular.z = 0
                pub.publish(control_command)
            # Use our rate object to sleep until it is time to publish again
            r.sleep()

    def update_odometry(self, data):
        if (not self.odometry_init_check):
            self.odometry_init_check = True
            self.init_odometry = data
        self.odometry = data

    def rotate(self, speed, angle, clockwise):
            # Starts a new node
        self.odometry_init_check = False
        while (not self.odometry_init_check):
            pass
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed*2*PI/360
        print(angular_speed)
        relative_angle = angle*2*PI/360

        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        #vel_msg.angular.z = 5
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        # print("here")
        r = rospy.Rate(10)  # 10hz
        init_orientation = self.init_odometry.pose.pose.orientation
        inti_z = math.atan2(init_orientation.w * init_orientation.z + init_orientation.x *
                            init_orientation.y, 1 - 2*(init_orientation.y**2 + init_orientation.z**2))

        tolerance = 2 * 2*PI/360
        # while(current_angle < relative_angle):
        #     velocity_publisher.publish(vel_msg)
        #     t1 = rospy.Time.now().to_sec()
        #     current_angle = angular_speed*(t1-t0)
        #     r.sleep()
        orientation = self.odometry.pose.pose.orientation
        current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                               orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        while (relative_angle - abs(current_z - inti_z) > tolerance):
            print(self.odometry.pose.pose.orientation.z, inti_z, abs(
                self.odometry.pose.pose.orientation.z - inti_z), relative_angle, tolerance)
            self.velocity_publisher.publish(vel_msg)
            print("here")
            r.sleep()
            orientation = self.odometry.pose.pose.orientation
            current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                                   orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        # Forcing our robot to stop
        print("finish", abs(current_z - inti_z))
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # rospy.spin()

    def go_straight(self, speed, distance, direction):
        if speed>0.5:
          return 
        # Starts a new node
        self.odometry_init_check = False
        while (not self.odometry_init_check):
            pass
        vel_msg = Twist()

        # We wont use linear components
        
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        # Checking if our movement is CW or CCW
        if direction:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        #vel_msg.angular.z = 5
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        # print("here")
        r = rospy.Rate(10)  # 10hz
        init_position = self.init_odometry.pose.pose.position

        tolerance = 0.001
        # while(current_angle < relative_angle):
        #     velocity_publisher.publish(vel_msg)
        #     t1 = rospy.Time.now().to_sec()
        #     current_angle = angular_speed*(t1-t0)
        #     r.sleep()
        use_speed_provide = True
        check_back_and_forth = False
        position = self.odometry.pose.pose.position
        while (abs(distance - math.sqrt((init_position.x - position.x)**2 + (init_position.y - position.y)**2)) > tolerance):
            current_distance =  math.sqrt((init_position.x - position.x)**2 + (init_position.y - position.y)**2)
            if (distance - current_distance < 0):
                check_back_and_forth = True
                use_speed_provide = False
            else:
                check_back_and_forth = False
            if not check_back_and_forth:
                if use_speed_provide:
                    vel_msg.linear.x = speed
                else:
                    vel_msg.linear.x = 0.05
                self.velocity_publisher.publish(vel_msg)
            else:
                vel_msg.linear.x = -0.05
                self.velocity_publisher.publish(vel_msg) 
                print "too_much"
            print "current distance", math.sqrt((init_position.x - position.x)**2 + (init_position.y - position.y)**2)
            r.sleep()
            position = self.odometry.pose.pose.position
            
            # orientation = self.odometry.pose.pose.orientation
            # current_z = math.atan2(orientation.w * orientation.z + orientation.x *
            #                        orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        # Forcing our robot to stop
        # print("finish", abs(current_z - inti_z))
        print "finish", math.sqrt((init_position.x - position.x)**2 + (init_position.y - position.y)**2)
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        return math.sqrt((init_position.x - position.x)**2 + (init_position.y - position.y)**2)
        # rospy.spin()


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
        # Check if the node has received a signal to shut down
    # If not, run the talker method

    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('turtlebot_controller', anonymous=True)

    # try:
    #   controller(sys.argv[1], sys.argv[2])
    # except rospy.ROSInterruptException:
    #   pass
    # while (1):
    # 	print("Let's rotate your robot")
    # 	speed = input("Input your speed (degrees/sec):")
    # 	angle = input("Type your distance (degrees):")
    # 	clockwise = input("Clockwise?: ") #True or false
    # 	rotate(speed, angle, clockwise)
    control = Controller()
    # control.rotate(90, 90, 1)
    control.controller('base_link', 'object_1green_1')
    # control.go_straight(0.2, 0.4, 1)
