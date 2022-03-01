// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/controller/PIDController.h>

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);




    nt::NetworkTableInstance networkTables = nt::NetworkTableInstance::GetDefault();
    m_pixyTable = networkTables.GetTable("PixyBlocks");

    m_pidController = new frc2::PIDController (kP, kI, kD);
    m_pidController->SetTolerance(8, 8);
    m_pidController->SetSetpoint(150);
  }

  void TeleopPeriodic() override {
    bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
    bool track_ball_button_pressed = m_stick.GetRawButton(2);
      if ( track_ball_button_pressed){
        bool ball_seen = (m_pixyTable->GetNumber("STATUS", 0) == 1);
        if (ball_seen){
          double ball_x = m_pixyTable->GetNumber("X", 150);
          double rotate_speed = m_pidController->Calculate(ball_x);
          m_robotDrive.DriveCartesian(0, 0, rotate_speed);
        }
      }
      else{
      m_robotDrive.DriveCartesian(m_stick.GetX(), m_stick.GetY(), m_stick.GetZ());
      }
  }

 private:
  static constexpr int kFrontLeftChannel = 0;
  static constexpr int kRearLeftChannel = 1;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 3;

  static constexpr int kJoystickChannel = 0;

  frc::Talon m_frontLeft{kFrontLeftChannel};
  frc::Talon m_rearLeft{kRearLeftChannel};
  frc::Talon m_frontRight{kFrontRightChannel};
  frc::Talon m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};

  frc::Joystick m_stick{kJoystickChannel};

  std::shared_ptr<nt::NetworkTable> m_pixyTable;
  frc2::PIDController *m_pidController;
  double kP = 0.02;
  double kI = 0.0;
  double kD = 0.002;

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
