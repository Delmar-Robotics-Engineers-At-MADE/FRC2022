#pragma once

#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>

enum RatchetState {
  kRatchetUnknown = 0,
  kRatchetMoving,
  kRatchetMoved,
  kRatchetIdle
};

class Climber {
public:
  void TeleopPeriodic (frc::Joystick *copilot);
  void RobotInit();
  void DoOnceInit();
  void RepeatableInit();
  void TeleopPeriodic();
  void TeleopInit();

private:
  // WPI_TalonSRX mClimberPort{3};
  // WPI_TalonSRX mClimberStar{12};
  WPI_TalonSRX mClimberPort{3};  
  WPI_TalonSRX mClimberStar{12}; 
  frc::DigitalInput mLimitSwitchPort{3};
  frc::DigitalInput mLimitSwitchStar{2};
  bool mSmartClimberEnabled {false};
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};

  frc::Timer mEndgameTimer;
  frc::Timer mRatchetTimer;
  RatchetState mRatchetState = kRatchetUnknown;

  void SmartClimber(int povPad);
  void ManualClimber(frc::Joystick *copilot, bool slowspeed);
  void CheckHomePositions();
  void OpenRatchetIfExtending (double powerPort, double powerStar);
  void StopClimbers();
};