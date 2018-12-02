#include "ros_setup.h"
#include <Arduino.h>
#include <Servo.h>
Arm7Bot arm;
#define GRIPPER_SERVO_PIN 8
int gripper_position = 90;
Servo gripper_servo;
void setup() {
  // initial 7Bot Arm
  arm.initialMove();
  arm.forcelessMode();
  ros_setup();
  // TODO the gripper servo has been changed
  gripper_servo.attach(GRIPPER_SERVO_PIN);
  pinMode(GRIPPER_SERVO_PIN, OUTPUT);
}

int flag = 1;
void loop() {
  // update the state
  arm.updatePos();
  ros_loop();
  #if 0
  flag = !flag;
  if(flag)gripper_servo.write(45);
  else gripper_servo.write(135);
  delay(20);
  #endif
}
