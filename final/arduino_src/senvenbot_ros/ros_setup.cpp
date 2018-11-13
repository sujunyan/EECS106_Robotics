
#include "ros_setup.h"


ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float64MultiArray>cmd_sub("7bot/cmd", cmdCallBack,100);
sensor_msgs::JointState joint_state;
ros::Publisher state_pub("7bot/joint_state",&joint_state);
char * joint_name[] = {"j1", "j2", "j2m", "j3", "j4", "j5", "j6"};
float g_position[SERVO_NUM] = {0};
void ros_setup(){
  nh.initNode();
  nh.advertise(state_pub);
  //nh.subscribe(cmd_sub);
  // assian the name of joint state
  joint_state.name_length = SERVO_NUM;
  joint_state.position_length = SERVO_NUM;
  joint_state.name = joint_name;
  joint_state.position = g_position;
}
