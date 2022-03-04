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

class Robot : public frc::TimedRobot {
private:
  WPI_TalonFX mPortShooter{0};  // LEADER
  WPI_TalonFX mStarShooter{15};
  double mMotorOutVelocity = 0.0;
  
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

    mPortShooter.Config_kF(0, 0.045, 30);
    mPortShooter.Config_kP(0, 0.009, 30);
    mPortShooter.Config_kI(0, 0.00005, 30);
    mPortShooter.Config_kD(0, 0.0, 30);

    mPortShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);
    
  }

  void TeleopInit() override {
    mMotorOutVelocity = 0.0;
    frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutVelocity);
  }

  void TeleopPeriodic() override {
    mMotorOutVelocity = frc::SmartDashboard::GetNumber("motor output percentage", 0);
    mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
