#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <VelocityController2.h>

class TeleopController {

public:
  TeleopController (VelocityController2 *vcontroller); // constructor
  void AutonomousInit();
  void AutonomousPeriodic();

private:

  VelocityController2 *mVelocityController;
};
