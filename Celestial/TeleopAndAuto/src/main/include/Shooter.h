#pragma once

#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>
#include "ctre/Phoenix.h"
#include <Elevator.h>
#include <Feeder.h>

static const double kShooterSpeedForAuto = 13300; // was 14000
static const double kShooterSpeedForBlindShot = 13300; 

enum ShooterState {
  kShooterUnknownState = 0,
  kRotatingToTarget,
  kOnTarget,
  kShooterReady,
  kShooting,
  kEmpty,
  kIdle
};

enum BlindShotState {
		kBSSUnknownState = 0,
    kBSSBegin,
    kBSSReadying,
    kBSSShooting,
		kBSSCompleted
	};



class Shooter {
private:

  // shooter proper and Limelight

  constexpr static double kPhi = 21.5;  // angle in degrees of limelight from vertical
  constexpr static double kH1 = 2.5;  // height in feet of limelight from floor
  constexpr static double kH2 = 8.6 - kH1; // height in feet of target ring from limelight
  double mPhi = kPhi;
  double mH2 = kH2;
  // double *mCoeff;  // array of coefficients for calcs

  std::shared_ptr<nt::NetworkTable> mLimeTable; // for LimeLight

  bool mLightOn = false;  // light should be off when booted up; if not, turn it off on web console
  ShooterState mState = kShooterUnknownState;
  frc2::PIDController *mPIDController;

  WPI_TalonFX mPortShooter{0};  // LEADER
  WPI_TalonFX mStarShooter{15};

  double mMotorOutVelocity = 0.0; // for collecting data for targeting

  bool ReadyShooter(bool hightTarget);
  double CalcHighTargetSpeed(double d);

  double mAutoShootSpeed= kShooterSpeedForAuto;

  // blind shot stuff
  BlindShotState mBlindShotState = kBSSUnknownState;
  frc::Timer mBlindShotTimer; 

  // elevator
  Elevator mElevator;

  // feeder
  Feeder *mFeeder;

public:

  bool mTargetSeen = false; // tv is non-zero if there is a target detected
  double mTargetArea = 0.0;
  double mTargetAngleHorizontal = 0.0;
  double mTargetAngleVertical = 0.0;
  double mTargetDistance = 0.0;
  double mSpeedMultiplier = 1.0;

  Shooter (); // constructor
  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, DriveSysTargetingState driveState);
  void RobotInit(Feeder *feeder);
  void RepeatableInit();
  void TeleopInit();
  void RobotPeriodic();
  void FeedCargo();
  void Shoot (bool highTarget, DriveSysTargetingState driveState);
  void Idle();
  void CheckLimelight();
  void TurnLightOnOrOff (bool turnOn);
  void DoOnceInit();
  void FixedSpeedForAuto();
  bool FixedElevationForAuto();
  void ShootForAuto();
  void AutonomousInit();
  void BlindShot(frc::Joystick *copilot);
  void DemoReturnBall(bool returning);

};