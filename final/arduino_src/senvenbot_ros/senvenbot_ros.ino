#include "ros_setup.h"
#include <Arduino.h>
Arm7Bot arm;
void setup() {
  // initial 7Bot Arm
  arm.initialMove();
  arm.forcelessMode();
  ros_setup();
}

void loop() {
  // update the state
  arm.updatePos();
  ros_loop();
  //delay(1);
}
