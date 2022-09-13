#pragma once

#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Joystick.h>
#include <Intake.h>

class Feeder {

private:
  WPI_TalonSRX mFeeder{7};
  // void ManualFeed (frc::Joystick *copilot);
  // bool mManualFeeding = false;
  frc::DigitalInput mEyeFeeder{5}; 

  bool mManualFeeding = false; // might never be true, as we have not figured out where to put controls yet 

#ifdef SUMMER
  void DemoReturnBall(Intake *intake);
#endif

public:
  bool CargoAvailable();
  void FeedCargo();
  void StopFeedingCargo();
  void ManualFeed (frc::Joystick *pilot);
  void TeleopPeriodic (Intake *intake);
  void RobotInit();

};
