#ifdef DELETEME

#include <VelocityController2.h>
#include <algorithm>
#include <iostream>

static constexpr units::second_t kDt = 20_ms;

const static double kPtunedGyro = 0.009;
const static double kItunedGyro = 0.0;
const static double kDtunedGyro = 0.0;

double VelocityController2::ForwardAtSpeed (units::feet_per_second_t feetPerSec, double gyroAngle) {
    units::feet_per_second_t conversionFactor = (units::feet_per_second_t)4.0;
    double speedInFeetPerSec = feetPerSec/conversionFactor;
    mRobotDrive->DriveCartesian(-speedInFeetPerSec, 0, 0, gyroAngle);
}

VelocityController2::VelocityController2 (frc::MecanumDrive *drive, AHRS *gyro) { // constructor
  mRobotDrive = drive;
  mAHRS = gyro;
  mPIDControllerGyro = new frc2::PIDController (kPtunedGyro, kItunedGyro, kDtunedGyro);
  mPIDControllerGyro->SetTolerance(8, 8);  // within 8 degrees of direction is considered on set point
}

void VelocityController2::SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps) {
  mGoal = {distance, fps};
  mInitialState = {0_ft, 0_fps};
  mProfile = new frc::TrapezoidProfile<units::feet> (mConstraints, mGoal, mInitialState);
}

bool VelocityController2::DriveTrapezoid () {
  frc::TrapezoidProfile<units::length::feet>::State setpoint = mProfile->Calculate(mTimer.Get()); 
  ForwardAtSpeed(setpoint.velocity, mAHRS->GetAngle());
  //std::cout << "v:" << (double)setpoint.velocity << "  x" << (double)setpoint.position << std::endl;
  return mProfile->IsFinished(mTimer.Get());
}

void VelocityController2::StartMotionTimer() {
  mTimer.Reset();
  mTimer.Start();
}

bool VelocityController2::TurnRight (double degrees){
  // offset everything 180 deg. to avoid discontinuity at 0/360
  mPIDControllerGyro->SetSetpoint(degrees + 180.0);
  double rotateRate = mPIDControllerGyro->Calculate(mAHRS->GetAngle() + 180.0);
  mRobotDrive->DriveCartesian(0, 0, -rotateRate, mAHRS->GetAngle());
  return mPIDControllerGyro->AtSetpoint();
}

bool VelocityController2::TurnStraight (){
  // offset everything 180 deg. to avoid discontinuity at 0/360
  // mPIDControllerGyro->SetSetpoint(180.0);
  // double rotateRate = mPIDControllerGyro->Calculate(mAHRS->GetAngle() + 180.0);
  // mRobotDrive->DriveCartesian(0, 0, 0, rotateRate);
  // return mPIDControllerGyro->AtSetpoint();
  TurnRight(0);
}

void VelocityController2::StopDriving() {
  mRobotDrive->DriveCartesian(0, 0, 0);
}

void VelocityController2::TrackBall (PixyBallTracker *tracker) {
  double rotateSpeed = tracker->CalculateResponse();
  // use cartesian drive as if we were driver controlled, but only rotate
  mRobotDrive->DriveCartesian(0, 0, rotateSpeed);
}

void VelocityController2::TelopPeriodic (frc::Joystick *pilot){
    mRobotDrive->DriveCartesian(pilot->GetY(), -pilot->GetX(), -pilot->GetZ(), mAHRS->GetAngle());
}

#endif