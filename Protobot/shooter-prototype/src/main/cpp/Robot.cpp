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

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  WPI_TalonFX m_leftMotor{0};
  WPI_TalonFX m_rightMotor{12};
  //frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};

  double motorOutPercent = 0.0;

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);
    frc::SmartDashboard::PutNumber("motor output percentage", motorOutPercent);
    
  }

  void TeleopInit() override {
    frc::SmartDashboard::PutNumber("motor output percentage", motorOutPercent);
  }

  void TeleopPeriodic() override {
    // Drive with tank style
    //m_robotDrive.TankDrive(m_leftStick.GetY(), m_rightStick.GetY());
    motorOutPercent = frc::SmartDashboard::GetNumber("motor output percentage", 0);
    m_leftMotor.Set(ControlMode::PercentOutput, motorOutPercent);
    // std::cout << "motor setpoint: " << motorOutPercent << std::endl;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
