#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"

class Intake {
public:
  void TeleopPeriodic (frc::Joystick *pilot);
  void RobotInit();
private:
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  WPI_TalonSRX mRoller{8};

};