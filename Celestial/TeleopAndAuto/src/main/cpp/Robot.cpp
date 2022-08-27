#include <Robot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

// #define SUMMER

const double kP = 0.012;
const double kI = 0.0;
const double kD = 0.0005;

void Robot::RobotInit() {
  mFrontRight.SetInverted(true);
  mRearRight.SetInverted(true);
  mTracker = new PixyBallTracker (kP, kI, kD);
  mClimber.RobotInit();
  mShooter = new Shooter();
  mShooter->RobotInit(&mFeeder, &mIntake);
  mIntake.RobotInit();
  mFeeder.RobotInit();

  mRobotDrive.RobotInit(mShooter, &mIntake, 
                    &mFrontLeft, &mRearLeft, &mFrontRight, &mRearRight,
                    &mPIDFrontLeft, &mPIDRearLeft, &mPIDFrontRight, &mPIDRearRight,
                    &mEncoderFrontLeft, &mEncoderRearLeft, &mEncoderFrontRight, &mEncoderRearRight);

  mAutoController = new AutonomousController(&mRobotDrive, mShooter, &mFeeder);

}

void Robot::RobotPeriodic() {
  // for testing
  // mShooter->RobotPeriodic();  // display limit switch
  mRobotDrive.RobotPeriodic(); // display heading and color sensor
}

void Robot::DoOnceInit()  {
  if (!mDoOnceInited) {
    mDoOnceInited = true;
    mClimber.DoOnceInit();
    mRobotDrive.DoOnceInit();
    mIntake.DoOnceInit();
    mShooter->DoOnceInit();
    }  
}

void Robot::RepeatableInit() {
  mRobotDrive.RepeatableInit();
  mClimber.RepeatableInit();
  mShooter->RepeatableInit();
}

void Robot::TeleopInit() {
  DoOnceInit();
  RepeatableInit();
  mShooter->TeleopInit();
  mClimber.TeleopInit();
  mIntake.TeleopInit();
}

void Robot::TeleopPeriodic() {

  mClimber.TeleopPeriodic(&mCopilot);
  mShooter->TeleopPeriodic(&mPilot, &mCopilot, mRobotDrive.mTargetingState);
#ifdef SUMMER  
  mRobotDrive.TeleopPeriodic(&mPilot, &mCopilot, &mRaspPi);
#else  
  mRobotDrive.TeleopPeriodic(&mPilot, &mCopilot);
#endif
  mIntake.TeleopPeriodic(&mPilot, mFeeder.CargoAvailable(), &mRaspPi);
  mFeeder.TeleopPeriodic(&mIntake);

}

void Robot::AutonomousInit() {
  DoOnceInit();
  RepeatableInit();
  mAutoController->AutonomousInit();
  mShooter->AutonomousInit();
}

void Robot::AutonomousPeriodic() {
  // 16in/1s
  // if (mAutonomousTimer.Get() < 2.0_s) {
  //   mVelocityController->ForwardAtSpeed((units::feet_per_second_t) 2.0);   // mRobotDrive.DriveCartesian(-0.5, 0, 0);
  // } else {
  //   mRobotDrive.DriveCartesian(0, 0, 0); // same as mRobotDrive.StopMotor(); ?
  // }
  mAutoController->AutonomousPeriodic();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

