#include <Arm7Bot.h>
#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
ros::NodeHandle  nh;
Arm7Bot arm;
// the command Subscriber to listen to the action command from the PC
void cmdCallBack(const std_msgs::Float64MultiArray & cmd_msg);
ros::Subscriber<std_msgs::Float64MultiArray>cmd_sub("7bot/cmd", cmdCallBack,100);
sensor_msgs::JointState joint_state;
ros::Publisher state_pub("7bot/joint_state",&joint_state);
String joint_name[] = {"j1", "j2", "j2m", "j3", "j4", "j5", "j6"};
void setup() {
  // initial 7Bot Arm
  arm.initialMove();
  // setup for ROS
  nh.initNode();
  nh.advertise(state_pub);
  //nh.subscribe(cmd_sub);
  // assian the name of joint state
  joint_state.name_length = SERVO_NUM;
  joint_state.position_length = SERVO_NUM;
  int tmp = 0;
  for (size_t i = 0; i < SERVO_NUM; i++) {
    size_t name_size = joint_name[i].length();
    size_t j = 0 ;
    for (j = 0; j < name_size; j++) {
        joint_state.name [i][j] = joint_name[i][j];
    }
    joint_state.name[i][j] = 0;
  }
}

void loop() {
  // update the state
  arm.updatePos();
  for (size_t i = 0; i < SERVO_NUM; i++) {
    joint_state.position[i] = arm.servoPosD[i];
  }
  state_pub.publish(&joint_state);
  nh.spinOnce();
  delay(100);

}

void cmdCallBack(const std_msgs::Float64MultiArray & cmd_msg){
    double angles[SERVO_NUM];
    for (size_t i = 0; i < SERVO_NUM; i++) {
      angles[i] = cmd_msg.data[i];
    }
    arm.move(angles);
}
