#pragma once

#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Climber {
public:
  void TelopPeriodic (frc::Joystick *copilot);
  void RobotInit();
  void DoOnceInit();
  void TeleopPeriodic();
  void TeleopInit();

private:
  // WPI_TalonSRX mClimberPort{3};
  // WPI_TalonSRX mClimberStar{12};
  WPI_TalonSRX mClimberPort{3};  
  WPI_TalonSRX mClimberStar{8};  // on pinoccio
  frc::DigitalInput mLimitSwitchPort{4};
  frc::DigitalInput mLimitSwitchStar{5};
  bool mSmartClimberEnabled {false};

  frc::Timer mTimer;

  void SmartClimber(int povPad);
  void ManualClimber(frc::Joystick *copilot);
  void CheckHomePositions();
};