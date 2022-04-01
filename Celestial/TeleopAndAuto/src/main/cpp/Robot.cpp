#include <Robot.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

const double kP = 0.012;
const double kI = 0.0;
const double kD = 0.0005;

void Robot::RobotInit() {
  mFrontRight.SetInverted(true);
  mRearRight.SetInverted(true);
  mTracker = new PixyBallTracker (kP, kI, kD);
  mClimber.RobotInit();
  mShooter = new Shooter();
  mShooter->RobotInit();

  mRobotDrive.RobotInit(mShooter, &mPIDFrontLeft, &mPIDRearLeft, &mPIDFrontRight, &mPIDRearRight,
                    &mEncoderFrontLeft, &mEncoderRearLeft, &mEncoderFrontRight, &mEncoderRearRight);

  mFrontLeft .SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mRearLeft  .SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mFrontRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mRearRight .SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  mAutoController = new AutonomousController(&mRobotDrive);

}

void Robot::RobotPeriodic() {
  // for testing
  mShooter->RobotPeriodic();  // display limit switch
  mRobotDrive.RobotPeriodic(); // display heading
}

void Robot::DoOnceInit()  {
  if (!mDoOnceInited) {
    mDoOnceInited = true;
    mClimber.DoOnceInit();
    mRobotDrive.DoOnceInit();
    mIntake.DoOnceInit();
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
}

void Robot::TeleopPeriodic() {

  mClimber.TelopPeriodic(&mCopilot);
  mShooter->TelopPeriodic(&mPilot, &mCopilot, mRobotDrive.mTargetingState);
  mRobotDrive.TelopPeriodic(&mPilot, &mCopilot);
  mIntake.TeleopPeriodic(&mPilot);
 
}

void Robot::AutonomousInit() {
  DoOnceInit();
  RepeatableInit();
  mAutoController->AutonomousInit();
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

