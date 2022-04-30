#pragma once

// #include <frc/PneumaticsControlModule.h>
// #include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include <RaspPi.h>

enum FetchBallStates {
		kFBSUnknownState = 0,
    kFBSWaitingForBall,
    kFBSBallAhead,
    kFBSBallGone,
		kFBSBallAtFeeder,
    kFBSBallReturning
	};

class Intake {
public:
  void TeleopPeriodic (frc::Joystick *pilot, bool ballAtFeeder, RaspPi *rPi);
  void AutonomousPeriodic ();
  void RobotInit();
  void DoOnceInit();
  bool DemoReturningBall();
  void TeleopInit();

private:
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};
  WPI_TalonSRX mRoller{8};
  frc::Timer mTimer; 
  FetchBallStates mFetchState = kFBSUnknownState;
  bool mEnableSummerDemo = false;

  void FetchBall (bool ballAtFeeder, RaspPi *rPi);
  void Deploy();
  void Retract();


};