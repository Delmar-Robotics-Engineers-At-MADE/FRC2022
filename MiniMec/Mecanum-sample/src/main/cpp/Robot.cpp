#include <Robot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

const double kP = 0.012;
const double kI = 0.0;
const double kD = 0.0005;

void Robot::RobotInit() {
  mFrontRight.SetInverted(true);
  mRearRight.SetInverted(true);

  mTracker = new PixyBallTracker (kP, kI, kD);
  mVelocityController = new VelocityController2(&mRobotDrive);

  try{
      mAHRS = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception &ex) {
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char *p_err_msg = err_msg.c_str();
      FRC_ReportError(0, "{}", p_err_msg); // frc::DriverStation::ReportError(p_err_msg);
    }
}

void Robot::DoOnceInit()  {
  if (!mDoOnceInited) {
    mDoOnceInited = true;
    }  
}

void Robot::RepeatableInit() {
  mAHRS->ZeroYaw();   // use current robot orientation as field forward
}

void Robot::TeleopInit() {
  DoOnceInit();
  RepeatableInit();
}

void Robot::TeleopPeriodic() {

  bool reset_yaw_button_pressed = mStick.GetRawButton(1);
  bool track_ball_button_pressed = mStick.GetRawButton(2);

  if (track_ball_button_pressed && mTracker->BallSeen()) { 
      double rotateSpeed = mTracker->CalculateResponse();
      // use cartesian drive as if we were driver controlled, but only rotate
      mRobotDrive.DriveCartesian(0, 0, rotateSpeed);
    

  } else { // no buttons pressed

    /* Use the joystick X axis for lateral movement, Y axis for forward
    * movement, and Z axis for rotation.
    */
    mRobotDrive.DriveCartesian(mStick.GetY(), -mStick.GetX(), -mStick.GetZ(), mAHRS->GetAngle());

  }

}

void Robot::AutonomousInit() {
  DoOnceInit();
  RepeatableInit();
  mAutonomousTimer.Reset();
  mAutonomousTimer.Start();
  mVelocityController->SetTrapezoidGoal(5.0_ft, 0_fps);
  mVelocityController->StartMotionTimer();
}

void Robot::AutonomousPeriodic() {
  // 16in/1s
  // if (mAutonomousTimer.Get() < 2.0_s) {
  //   mVelocityController->ForwardAtSpeed((units::feet_per_second_t) 2.0);   // mRobotDrive.DriveCartesian(-0.5, 0, 0);
  // } else {
  //   mRobotDrive.DriveCartesian(0, 0, 0); // same as mRobotDrive.StopMotor(); ?
  // }
  mVelocityController->DriveTrapezoid();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

