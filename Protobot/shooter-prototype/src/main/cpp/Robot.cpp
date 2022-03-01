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
  double mMotorOutPercent = 0.0;

 public:
  void RobotInit() override {
    frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutPercent);

    // one follower and one reversed
    mStarShooter.Follow(mPortShooter);
    mStarShooter.SetInverted(false);
    mPortShooter.SetInverted(true);
    
  }

  void TeleopInit() override {
    frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutPercent);
  }

  void TeleopPeriodic() override {
    mMotorOutPercent = frc::SmartDashboard::GetNumber("motor output percentage", 0);
    mPortShooter.Set(ControlMode::PercentOutput, mMotorOutPercent);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
