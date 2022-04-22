// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace frc;

static const double kFtuned = 0.0465; //.0451 for Celestial
static const double kPtuned = 0.2;
static const double kDtuned = 0.002;
static const double kRampTuned = 3.0;

class Robot : public frc::TimedRobot {
private:
  WPI_TalonFX mPortShooter{0};  // LEADER
  WPI_TalonFX mStarShooter{15};
  double mMotorOutVelocity = 0.0;

  double kF = kFtuned;
  double kP = kPtuned;
  double kD = kDtuned;
  //double kRamp = kRampTuned;
  
 public:
  void RobotInit() override {
    frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutVelocity);
    mStarShooter.ConfigFactoryDefault();
    mPortShooter.ConfigFactoryDefault();

    // one follower and one reversed
    mStarShooter.Follow(mPortShooter);
    mStarShooter.SetInverted(true);
    mPortShooter.SetInverted(false);

    mPortShooter.SetNeutralMode(NeutralMode::Coast);
    mStarShooter.SetNeutralMode(NeutralMode::Coast);

    // mPortShooter.Config_kF(0, 0.045, 30);
    // mPortShooter.Config_kP(0, 0.009, 30);
    // mPortShooter.Config_kI(0, 0.00005, 30);
    // mPortShooter.Config_kD(0, 0.0, 30);
    mPortShooter.Config_kF(0, kFtuned, 30);
    mPortShooter.Config_kP(0, kPtuned, 30);
    mPortShooter.Config_kI(0, 0.0, 30);
    mPortShooter.Config_kD(0, kDtuned, 30);
    mPortShooter.ConfigClosedloopRamp(kRampTuned);

    mPortShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);
    
  }

  void TeleopInit() override {
    mMotorOutVelocity = 0.0;
    frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutVelocity);
    frc::SmartDashboard::PutNumber("motor kF", kF);
    frc::SmartDashboard::PutNumber("motor kP", kP);
    frc::SmartDashboard::PutNumber("motor kD", kD);
  }

  void TeleopPeriodic() override {
    double v = frc::SmartDashboard::GetNumber("motor output percentage", mMotorOutVelocity);
    double f = frc::SmartDashboard::GetNumber("motor kF", kF);
    double p = frc::SmartDashboard::GetNumber("motor kP", kP);
    double d = frc::SmartDashboard::GetNumber("motor kD", kD);

    if (f != kF) {kF = f; mPortShooter.Config_kF(0, kF, 30);}
    if (p != kP) {kP = p; mPortShooter.Config_kP(0, kP, 30);}
    if (d != kD) {kD = d; mPortShooter.Config_kD(0, kD, 30);}
    if (v != mMotorOutVelocity) {mMotorOutVelocity = v; mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);}

  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
