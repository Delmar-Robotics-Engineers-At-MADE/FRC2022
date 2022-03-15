#include <Robot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

const double kP = 0.012;
const double kI = 0.0;
const double kD = 0.0005;

void Robot::RobotInit() {
  // Invert the right side motors. You may need to change or remove this to
  // match your robot.
  m_frontRight.SetInverted(true);
  m_rearRight.SetInverted(true);

  mTracker = new PixyBallTracker (kP, kI, kD);

  try{
      m_ahrs = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception &ex) {
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char *p_err_msg = err_msg.c_str();
      FRC_ReportError(0, "{}", p_err_msg); // frc::DriverStation::ReportError(p_err_msg);
    }
}

void Robot::TeleopInit()  {
  m_ahrs->ZeroYaw();   // use current robot orientation as field forward
}

void Robot::TeleopPeriodic() {

  bool reset_yaw_button_pressed = m_stick.GetRawButton(1);
  bool track_ball_button_pressed = m_stick.GetRawButton(2);

  if (track_ball_button_pressed && mTracker->BallSeen()) { 
      double rotateSpeed = mTracker->CalculateResponse();
      // use cartesian drive as if we were driver controlled, but only rotate
      m_robotDrive.DriveCartesian(0, 0, rotateSpeed);
    

  } else { // no buttons pressed

    /* Use the joystick X axis for lateral movement, Y axis for forward
    * movement, and Z axis for rotation.
    */
    m_robotDrive.DriveCartesian(m_stick.GetY(), -m_stick.GetX(), -m_stick.GetZ(), m_ahrs->GetAngle());

  }

}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

