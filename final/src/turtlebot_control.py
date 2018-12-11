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
import tf
import geometry_msgs.msg

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from auto_navigation import NavTest
PI = 3.1415926535897


class Controller:
    # Define the method which contains the main functionality of the node.

    def __init__(self):
        self.velocity_publisher = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber(
            '/odom', Odometry, self.update_odometry)
        self.to_7bot_publisher = rospy.Publisher(
            '/controller_2_7bot', String, queue_size=10)
        self.from_7bot_subscriber = rospy.Subscriber(
            '/7bot_2_controller', String, self.update_7bot_status)
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
        r_rotation = rospy.Rate(0.2)
        K1 = 0.3
        K2 = 1
        fail_time = 0
        # Loop until the node is killed with Ctrl-C
        finded_check = False
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(
                     turtlebot_frame, goal_frame, rospy.Time(0))
                x_err = trans.transform.translation.x
                y_err = trans.transform.translation.y
                print "x_err", x_err, "y_err", y_err
                theta = math.atan2(abs(y_err), abs(x_err)) / 2
                if y_err < 0: 
                    clockwise = 1
                else:
                    clockwise = -1
                tolerance = 1 * PI / 180
                # self.rotate_radians(PI / 4, abs(theta), clockwise)
                print "break rotate_radians", theta, clockwise, tolerance
                # break
                
                if abs(theta) < tolerance:
                    break
                else:
                    self.rotate_radians(PI / 4, abs(theta), clockwise)
                r_rotation.sleep()
                r_rotation.sleep()
                r_rotation.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print "tf errors occur"
                fail_time += 1
                if (not finded_check and fail_time % 5 == 4):
                    self.rotate(40, 30, 1)
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
        r_rotation.sleep()
        r_rotation.sleep()
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(
                     turtlebot_frame, goal_frame, rospy.Time(0))
                x_err = trans.transform.translation.x
                y_err = trans.transform.translation.y
                z_err = trans.transform.translation.z
                print "x_err", x_err, "y_err", y_err
                actual_x_go = self.go_straight(0.2, abs(x_err)-0.1, 1)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print "tf errors occur"
            r.sleep()

        # communicate with 7bot
    #     state = "init"
    #     self.from_7bot_recieve = "init"
    #     while not rospy.is_shutdown():
    #         control.send_static_transform("base_link", "green_near", abs(abs(x_err) - actual_x_go), y_err, z_err, 0, 0, 0, 1) 
    #         if state == 'init':
    #             self.to_7bot_publisher("start")
    #         if self.from_7bot_recieve == "start recieved":
    #             state == "waiting"
    #             self.to_7bot_publisher("processing")
    #         else if self.from_7bot_recieve == "finished":
    #             break
        

    # def update_7bot_status(self, data):
    #     self.from_7bot_recieve = data   
    def update_odometry(self, data):
        if (not self.odometry_init_check):
            self.odometry_init_check = True
            self.init_odometry = data
        self.odometry = data

    def rotate_radians(self, speed, angle, clockwise):
            # Starts a new node
        if speed > PI /2 or speed < 0:
            print "speed is negative or is more than PI / 2"
            return
        if not abs(clockwise) == 1:
            print "clockwise must be 1 or -1"
            return  
        if angle < 0:
            print "angle must be positive"
            return
        print "rotate_radians speed", speed, "angle", angle, "clockwise", clockwise
        self.odometry_init_check = False
        while (not self.odometry_init_check):
            pass
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed
        relative_angle = angle

        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise == 1:
            vel_msg.angular.z = -abs(angular_speed)
            speed_provide =  -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
            speed_provide = abs(angular_speed)
        #vel_msg.angular.z = 5
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        # print("here")
        r = rospy.Rate(10)  # 10hz
        init_orientation = self.init_odometry.pose.pose.orientation
        inti_z = math.atan2(init_orientation.w * init_orientation.z + init_orientation.x *
                            init_orientation.y, 1 - 2*(init_orientation.y**2 + init_orientation.z**2))

        tolerance = 0.5 * 2*PI/360
        # while(current_angle < relative_angle):
        #     velocity_publisher.publish(vel_msg)
        #     t1 = rospy.Time.now().to_sec()
        #     current_angle = angular_speed*(t1-t0)
        #     r.sleep()
        orientation = self.odometry.pose.pose.orientation
        current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                               orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        
        use_speed_provide = True
        use_clockwise_provide = True
        init_time = rospy.get_time()
        while (abs(relative_angle - abs(current_z - inti_z))> tolerance):
            current_time = rospy.get_time()
            if (current_time - init_time) > 7:
                break
            # print(self.odometry.pose.pose.orientation.z, inti_z, abs(
            #     self.odometry.pose.pose.orientation.z - inti_z), relative_angle, tolerance)
            if (relative_angle < abs(current_z - inti_z)):
                use_clockwise_provide = False
                use_speed_provide = False
            else:
                use_clockwise_provide = True
            if use_speed_provide:
                vel_msg.angular.z = speed_provide
            else:
                if use_clockwise_provide:
                    vel_msg.angular.z = -20 * 2*PI/360 * clockwise
                else:
                    vel_msg.angular.z = 20 * 2*PI/360 * clockwise
            self.velocity_publisher.publish(vel_msg)
            # print("here")
            # r.sleep()
            orientation = self.odometry.pose.pose.orientation
            current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                                   orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        # Forcing our robot to stop
        print("finish", abs(current_z - inti_z) / 2 /PI*360)
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # rospy.spin()

    def rotate(self, speed, angle, clockwise, tolerance = 2 * 2*PI/360):
            # Starts a new node
        if speed>90 or speed < 0:
            print "speed is negative or is more than 90"
            return
        if not abs(clockwise) == 1:
            print "clockwise must be 1 or -1"
            return  
        if angle < 0:
            print "angle must be positive"
            return
        print "rotate speed", speed, "angle", angle, "clockwise", clockwise

        self.odometry_init_check = False
        while (not self.odometry_init_check):
            pass
        vel_msg = Twist()

        # Converting from angles to radians
        angular_speed = speed*2*PI/360
        # print(angular_speed)
        relative_angle = angle*2*PI/360

        # We wont use linear components
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise == 1:
            vel_msg.angular.z = -abs(angular_speed)
            speed_provide =  -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
            speed_provide = abs(angular_speed)
        #vel_msg.angular.z = 5
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        # print("here")
        r = rospy.Rate(10)  # 10hz
        init_orientation = self.init_odometry.pose.pose.orientation
        inti_z = math.atan2(init_orientation.w * init_orientation.z + init_orientation.x *
                            init_orientation.y, 1 - 2*(init_orientation.y**2 + init_orientation.z**2))

        
        # while(current_angle < relative_angle):
        #     velocity_publisher.publish(vel_msg)
        #     t1 = rospy.Time.now().to_sec()
        #     current_angle = angular_speed*(t1-t0)
        #     r.sleep()
        orientation = self.odometry.pose.pose.orientation
        current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                               orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        
        use_speed_provide = True
        use_clockwise_provide = True
        init_time = rospy.get_time()
        while (abs(relative_angle - abs(current_z - inti_z))> tolerance):
            current_time = rospy.get_time()
            if (current_time - init_time) > 7:
                break
            print abs(current_z - inti_z),  relative_angle,abs(abs(current_z - inti_z) - relative_angle), tolerance
            if (relative_angle < abs(current_z - inti_z)):
                use_clockwise_provide = False
                use_speed_provide = False
            else:
                use_clockwise_provide = True
            
            if use_speed_provide:
                vel_msg.angular.z = speed_provide
                # print "speed_provide", speed_provide
            else:
                if use_clockwise_provide:
                    vel_msg.angular.z = -20 * 2*PI/360 * clockwise
                else:
                    vel_msg.angular.z = 20 * 2*PI/360 * clockwise
            self.velocity_publisher.publish(vel_msg)
            # print("here")
            orientation = self.odometry.pose.pose.orientation
            current_z = math.atan2(orientation.w * orientation.z + orientation.x *
                                   orientation.y, 1 - 2*(orientation.y**2 + orientation.z**2))
        # Forcing our robot to stop
        print("finish", abs(current_z - inti_z) / 2 /PI*360)
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # rospy.spin()

    def go_straight(self, speed, distance, direction):
        if speed>0.5 or speed < 0:
            print "speed is negative or is more than 0.5"
            return
        if not abs(direction) == 1:
            print "direction must be 1 or -1"
            return  
        if distance < 0:
            print "distance must be positive"
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
        if direction == 1:
            vel_msg.linear.x = abs(speed)
            speed_provide = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
            speed_provide = -abs(speed)
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
            print distance, current_distance
            if (distance - current_distance < 0):
                check_back_and_forth = True
                use_speed_provide = False
            else:
                check_back_and_forth = False
            if not check_back_and_forth:
                if use_speed_provide:
                    vel_msg.linear.x = speed_provide
                    # print "speed_provide", speed_provide
                else:
                    vel_msg.linear.x = 0.05 * direction 
                self.velocity_publisher.publish(vel_msg)
            else:
                vel_msg.linear.x = -0.05 * direction
                self.velocity_publisher.publish(vel_msg) 
                print "too_much"
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
    def send_static_transform(self, father_frame_id, child_frame_id, t_x, t_y, t_z, r_x, r_y, r_z, r_w):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped() 
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = father_frame_id
        static_transformStamped.child_frame_id = child_frame_id

        static_transformStamped.transform.translation.x = t_x
        static_transformStamped.transform.translation.y = t_y
        static_transformStamped.transform.translation.z = t_z

        static_transformStamped.transform.rotation.x = r_x
        static_transformStamped.transform.rotation.y = r_y
        static_transformStamped.transform.rotation.z = r_z
        static_transformStamped.transform.rotation.w = r_w    
        broadcaster.sendTransform(static_transformStamped)


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
    # control.rotate(40,90, -1)
    control.controller('base_link', 'object_1green_1')
    # control.go_straight(0.2, 0.636, 1)
    # control.rotate_radians(0.785398163397, 0.401055162969, -1)
    # while(True):
    #     control.send_static_transform("base_link", "green_near", 1.7, 0.9, 0.2, 0, 0, 0, 1)