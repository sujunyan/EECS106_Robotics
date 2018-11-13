
#ifndef _ROS_SETUP_H_
#define _ROS_SETUP_H_

#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
#include <Arm7Bot.h>

// the command Subscriber to listen to the action command from the PC
void cmdCallBack(const std_msgs::Float64MultiArray & cmd_msg);
void ros_setup();

extern ros::NodeHandle  nh;
extern ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub;
extern sensor_msgs::JointState joint_state;
extern ros::Publisher state_pub;
extern char * joint_name[] ;


#endif
