
#include "ros_setup.h"
#include "cstdlib"
#include "Servo.h"

ros::NodeHandle  nh;

sensor_msgs::JointState joint_state;
std_msgs::String debug_msg;
String debug_msg_string;
ros::Publisher state_pub("sevenbot/origin_joint_states",&joint_state);
ros::Publisher debug_pub("sevenbot/debug_msg",&debug_msg);

void cmdCallBack(const sensor_msgs::JointState & cmd_msg);
//ros::Subscriber<std_msgs::Float64MultiArray>cmd_sub("7bot/cmd", cmdCallBack,100);
ros::Subscriber<sensor_msgs::JointState>cmd_sub("sevenbot/joint_cmd", cmdCallBack);
//ros::Subscriber<std_msgs::Float64>cmd_sub("sevenbot/try", &cmdCallBack);

char * joint_name[] = {"j1", "j2", "j2m", "j3", "j4", "j5", "j6"};
float g_position[SERVO_NUM] = {0};
float g_force[SERVO_NUM] = {0};
int g_update_timer = 0;
int g_control_timer = 0;

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
  joint_state.effort = g_force;
}

const float DgreeToRadian = (M_PI / 180.0f);
void ros_loop(){
  // update the state every 33ms
  if (millis() - g_update_timer >= 33){
    g_update_timer = millis();
    for (size_t i = 0; i < SERVO_NUM; i++) {
      joint_state.position[i] = float(arm.posD[i] ) ;
      joint_state.effort[i] = float(arm.force[i]);
    }
    joint_state.header.stamp = nh.now();
    state_pub.publish(&joint_state);
  }

  // subscribe the control every 50ms
  if (millis() - g_control_timer >= 10){
    g_control_timer = millis();
    nh.spinOnce();
  }
}


extern int gripper_position;
void cmdCallBack(const sensor_msgs::JointState & cmd_msg){
    double angles[SERVO_NUM];
    //joint_state.position[i] = float(arm.posD[i]);
    // TODO the last servo is modified so we need to treat it seperately.
    for (size_t i = 0; i < SERVO_NUM ; i++) {
      angles[i] = cmd_msg.position[i];
    }
    arm.move(angles);

    //TODO cannot call the servo.write function inside this
    //gripper_position = int(cmd_msg.position[SERVO_NUM-1]);


}
