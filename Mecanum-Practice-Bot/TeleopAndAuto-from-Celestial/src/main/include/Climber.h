#pragma once

#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>

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
  WPI_TalonSRX mClimberStar{12}; 
  frc::DigitalInput mLimitSwitchPort{2};
  frc::DigitalInput mLimitSwitchStar{3};
  bool mSmartClimberEnabled {false};
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};

  frc::Timer mTimer;

  void SmartClimber(int povPad);
  void ManualClimber(frc::Joystick *copilot);
  void CheckHomePositions();
  void OpenRatchetIfExtending (double powerPort, double powerStar);
};