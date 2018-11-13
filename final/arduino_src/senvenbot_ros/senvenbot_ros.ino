#include "ros_setup.h"
#include <Arduino.h>
#define __ROS_DEBUG__
Arm7Bot arm;
void setup() {
  // initial 7Bot Arm
  arm.initialMove();
  arm.forcelessMode();
  #ifndef __ROS_DEBUG__
    Serial.begin(9600);
  #else
    ros_setup();

  #endif
}

void loop() {
  // update the state
  #ifndef __ROS_DEBUG__
    Serial.println("\nLoop start");
  #endif
  arm.updatePos();
  #if 1
  for (size_t i = 0; i < SERVO_NUM; i++) {
    joint_state.position[i] = float(arm.posD[i]);
    #ifndef __ROS_DEBUG__
    Serial.println(arm.posD[i]);
    Serial.println(joint_state.position[i]);
    #endif
  }
  #endif
  #ifdef __ROS_DEBUG__
  state_pub.publish(&joint_state);
  nh.spinOnce();
  #endif
  delay(100);

}

void cmdCallBack(const std_msgs::Float64MultiArray & cmd_msg){
    double angles[SERVO_NUM];
    for (size_t i = 0; i < SERVO_NUM; i++) {
      angles[i] = cmd_msg.data[i];
    }
    arm.move(angles);
}
