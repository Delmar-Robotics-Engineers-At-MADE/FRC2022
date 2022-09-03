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
    kFBSRotating,
    kFBSRotatingBack,
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
  bool mEnableSummerDemoRotateShoot = false;

private:
  frc::DoubleSolenoid mSolenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};
  WPI_TalonFX mRoller{5};
  frc::Timer mTimer; 
  frc::Encoder mEncoder{6, 7, false, frc::Encoder::k4X};
  // frc2::PIDController *mPIDController;

#ifdef SUMMER
  void FetchBall (bool ballAtFeeder, RaspPi *rPi);
#endif

  void Deploy();
  void Retract();

};