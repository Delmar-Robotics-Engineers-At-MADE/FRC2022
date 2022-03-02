// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../include/Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>



/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
 

void::Robot::RobotInit(){
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);

    mTracker = new pixyBallTracker (kp, ki, kd);
}

void::Robot::TeleopPeriodic(){
    bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
    bool track_ball_button_pressed = m_stick.GetRawButton(2);
      if ( track_ball_button_pressed && mTracker->ballSeen()){
          double speed = mTracker->calculateResponse();
          m_robotDrive.DriveCartesian(0, 0, speed);
      }
      else{
      m_robotDrive.DriveCartesian(m_stick.GetX(), m_stick.GetY(), m_stick.GetZ());
      }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
