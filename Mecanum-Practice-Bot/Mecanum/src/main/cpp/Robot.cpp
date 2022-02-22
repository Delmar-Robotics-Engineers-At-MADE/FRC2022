// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/VictorSP.h>
#include "AHRS.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


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

    try{
        /***********************************************************************
         * navX-MXP:
         * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
         * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
         * 
         * navX-Micro:
         * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
         * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
         * 
         * VMX-pi:
         * - Communication via USB.
         * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
         * 
         * Multiple navX-model devices on a single robot are supported.
         ************************************************************************/
        m_ahrs = new AHRS(frc::SPI::Port::kMXP);


      } catch (std::exception &ex) {
        std::string what_string = ex.what();
        std::string err_msg("Error instantiating navX MXP:  " + what_string);
        const char *p_err_msg = err_msg.c_str();
        // frc::DriverStation::ReportError(p_err_msg);
      }
  }

  void TeleopPeriodic() override {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    m_robotDrive.DriveCartesian(m_stick.GetY(), -m_stick.GetX(), -m_stick.GetZ(), m_ahrs->GetAngle());

    bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
    if ( reset_yaw_button_pressed ) {
      m_ahrs->ZeroYaw();  
      }

  }

 private:
  static constexpr int kFrontLeftChannel = 1; //1
  static constexpr int kRearLeftChannel = 0; //0
  static constexpr int kFrontRightChannel = 3; //3
  static constexpr int kRearRightChannel = 2; //2

  static constexpr int kJoystickChannel = 0;

  frc::VictorSP m_frontLeft{kFrontLeftChannel};
  frc::VictorSP m_rearLeft{kRearLeftChannel};
  frc::VictorSP m_frontRight{kFrontRightChannel};
  frc::VictorSP m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};

  frc::Joystick m_stick{0};

  AHRS *m_ahrs;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

