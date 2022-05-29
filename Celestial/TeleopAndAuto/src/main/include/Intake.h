#pragma once

// #include <frc/PneumaticsControlModule.h>
// #include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include <RaspPi.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>

enum FetchBallStates {
		kFBSUnknownState = 0,
    kFBSWaitingForBall,
    kFBSBallAhead,
    kFBSBallGone,
		kFBSBallAtFeeder,
    kFBSBallReturning,
    kFBSShooting
	};

class Intake {
public:
  void TeleopPeriodic (frc::Joystick *pilot, bool ballAtFeeder, RaspPi *rPi);
  void AutonomousPeriodic ();
  void RobotInit();
  void DoOnceInit();
  // bool DemoReturningBall();
  void TeleopInit();
  FetchBallStates mFetchState = kFBSUnknownState;
  bool mEnableSummerDemo = false;
  bool mEnableSummerDemoShoot = false;

private:
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};
  WPI_TalonSRX mRoller{8};
  frc::Timer mTimer; 
  frc::Encoder mEncoder{6, 7, false, frc::Encoder::k4X};
  frc2::PIDController *mPIDController;

  void FetchBall (bool ballAtFeeder, RaspPi *rPi);
  void Deploy();
  void Retract();

};