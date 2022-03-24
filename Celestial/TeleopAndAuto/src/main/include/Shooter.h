#pragma once

#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>

enum ShooterState {
  kShooterUnknownState = 0,
  kRotatingToTarget,
  kOnTarget,
  kShooterReady,
  kShooting,
  kEmpty,
  kIdle
};

class Shooter {
private:
  constexpr static double kPhi = 21.5;  // angle in degrees of limelight from vertical
  constexpr static double kH1 = 2.5;  // height in feet of limelight from floor
  constexpr static double kH2 = 8.4 - kH1; // height in feet of target ring from limelight
  double mPhi = kPhi;
  double mH2 = kH2;

  std::shared_ptr<nt::NetworkTable> mLimeTable; // for LimeLight

  bool mLightOn = false;
  ShooterState mState = kShooterUnknownState;
  frc2::PIDController *mPIDController;

  void CheckLimelight(double direction);
  void TurnLightOnOrOff (bool turnOn);
  void Shoot (bool highTarget, DriveSysTargetingState driveState);
  void Idle();
  // bool RotateToTarget();
  bool ReadyShooter();
  void FeedCargo();
  bool CargoAvailable();

public:

  bool mTargetSeen = false; // tv is non-zero if there is a target detected
  double mTargetArea = mLimeTable->GetNumber("ta",0.0);
  double mTargetAngleHorizontal = 0.0;
  double mTargetAngleVertical = 0.0;
  double mTargetDistance = 0.0;

  Shooter (); // constructor
  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, DriveSysTargetingState driveState);
  void RobotInit();
  void DoOnceInit();
  void TeleopInit();
};