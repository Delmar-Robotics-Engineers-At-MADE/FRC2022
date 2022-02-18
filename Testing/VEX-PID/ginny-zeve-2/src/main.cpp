/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\VEX                                              */
/*    Created:      Tue Feb 08 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision21             vision        21              
// Drivetrain           drivetrain    1, 10           
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int Brain_precision = 0;
//int Vision21_objectIndex = 0;

const char* printToBrain_numberFormat() {
  // look at the current precision setting to find the format string
  switch(Brain_precision){
    case 0:  return "%.0f"; // 0 decimal places (1)
    case 1:  return "%.1f"; // 1 decimal place  (0.1)
    case 2:  return "%.2f"; // 2 decimal places (0.01)
    case 3:  return "%.3f"; // 3 decimal places (0.001)
    default: return "%f"; // use the print system default for everthing else
  }
}
int main() {
  float P, error;

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

   P = 0.4;
  while (true) {
    // No Vision Sensor signature selected
    // select_a_sig=sig_1
    Vision21.takeSnapshot(Vision21__SIG_1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(Brain.Screen.row(), 1);
    Brain.Screen.print(printToBrain_numberFormat(), static_cast<float>(Vision21.objectCount));
    for(int i = 0; i < Vision21.objectCount; i++){
      if (Vision21.objects[i].width > 100)  {
        error = Vision21.objects[i].centerX - 180.0;
        Drivetrain.setTurnVelocity((P * error), percent);
        Drivetrain.turn(right);
      } else {
        Drivetrain.stop();
    }
  wait(5, msec);
  }
  
}
}