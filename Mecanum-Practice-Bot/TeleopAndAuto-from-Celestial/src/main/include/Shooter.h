#pragma once

#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>
#include "ctre/Phoenix.h"
#include <Elevator.h>

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

  // shooter proper and Limelight

  constexpr static double kPhi = 21.5;  // angle in degrees of limelight from vertical
  constexpr static double kH1 = 2.5;  // height in feet of limelight from floor
  constexpr static double kH2 = 8.6 - kH1; // height in feet of target ring from limelight
  double mPhi = kPhi;
  double mH2 = kH2;

  std::shared_ptr<nt::NetworkTable> mLimeTable; // for LimeLight

  bool mLightOn = false;
  ShooterState mState = kShooterUnknownState;
  frc2::PIDController *mPIDController;

  WPI_TalonFX mPortShooter{0};  // LEADER
  WPI_TalonFX mStarShooter{15};

  double mMotorOutVelocity = 0.0; // for collecting data for targeting

  void CheckLimelight();
  void TurnLightOnOrOff (bool turnOn);
  void Shoot (bool highTarget, DriveSysTargetingState driveState);
  void Idle();
  bool ReadyShooter(bool hightTarget);

  // feeder
  WPI_TalonSRX mFeeder{7};
  void FeedCargo();
  void StopFeeder();
  void ManualFeed (frc::Joystick *copilot);
  bool CargoAvailable();

  // elevator
  Elevator mElevator;

public:

  bool mTargetSeen = false; // tv is non-zero if there is a target detected
  double mTargetArea = 0.0;
  double mTargetAngleHorizontal = 0.0;
  double mTargetAngleVertical = 0.0;
  double mTargetDistance = 0.0;

  Shooter (); // constructor
  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, DriveSysTargetingState driveState);
  void RobotInit();
  void DoOnceInit();
  void TeleopInit();
  void RobotPeriodic();
};