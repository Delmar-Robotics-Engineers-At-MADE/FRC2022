#pragma once

// #include <frc/PneumaticsControlModule.h>
// #include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"

class Intake {
public:
  void TeleopPeriodic (frc::Joystick *pilot);
  void RobotInit();
  void DoOnceInit();
private:
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};
  WPI_TalonSRX mRoller{8};

};