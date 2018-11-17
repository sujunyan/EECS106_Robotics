
#ifndef _ROS_SETUP_H_
#define _ROS_SETUP_H_

#define __ROS_DEBUG__
#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <Arm7Bot.h>

void ros_setup();
void ros_loop();

extern Arm7Bot arm;


#endif
