
#include "ros_setup.h"
#include "cstdlib"

ros::NodeHandle  nh;

sensor_msgs::JointState joint_state;
std_msgs::String debug_msg;
String debug_msg_string;
ros::Publisher state_pub("sevenbot/joint_state",&joint_state);
ros::Publisher debug_pub("sevenbot/debug_msg",&debug_msg);

void cmdCallBack(const sensor_msgs::JointState & cmd_msg);
//ros::Subscriber<std_msgs::Float64MultiArray>cmd_sub("7bot/cmd", cmdCallBack,100);
ros::Subscriber<sensor_msgs::JointState>cmd_sub("sevenbot/joint_cmd", cmdCallBack);
//ros::Subscriber<std_msgs::Float64>cmd_sub("sevenbot/try", &cmdCallBack);

char * joint_name[] = {"j1", "j2", "j2m", "j3", "j4", "j5", "j6"};
float g_position[SERVO_NUM] = {0};

void ros_setup(){
  nh.initNode();
  nh.advertise(state_pub);
  nh.advertise(debug_pub);
  nh.subscribe(cmd_sub);
  // assian the name of joint state
  joint_state.name_length = SERVO_NUM;
  joint_state.position_length = SERVO_NUM;
  joint_state.name = joint_name;
  joint_state.position = g_position;
}


void ros_loop(){

  for (size_t i = 0; i < SERVO_NUM; i++) {
    joint_state.position[i] = float(arm.posD[i]);
  }
  state_pub.publish(&joint_state);
  //debug_msg.data = debug_msg_string.c_str();
  //debug_pub.publish(&debug_msg);
  nh.spinOnce();
  //debug_msg_string = "hello\n";

}



#if 0
void cmdCallBack(const std_msgs::Float64 & cmd_msg){

    debug_msg_string = "cmdCallBack called\n";
    debug_msg.data = debug_msg_string.c_str();
    debug_pub.publish(&debug_msg);
  }
#else

void cmdCallBack(const sensor_msgs::JointState & cmd_msg){

    debug_msg_string = "cmdCallBack called\n";
    debug_msg.data = debug_msg_string.c_str();
    debug_pub.publish(&debug_msg);
    #if 1
    double angles[SERVO_NUM];
    //joint_state.position[i] = float(arm.posD[i]);
    for (size_t i = 0; i < SERVO_NUM; i++) {
      angles[i] = cmd_msg.position[i];
    }
    arm.move(angles);
    #endif

}
#endif
