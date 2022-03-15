#include <VelocityController2.h>
#include <algorithm>
// #include <iostream>

static constexpr units::second_t kDt = 20_ms;

double VelocityController2::ForwardAtSpeed (units::feet_per_second_t feetPerSec) {
    units::feet_per_second_t conversionFactor = (units::feet_per_second_t)2.2;
    double speedInFeetPerSec = feetPerSec/conversionFactor;
    mRobotDrive->DriveCartesian(-speedInFeetPerSec, 0, 0);
}

VelocityController2::VelocityController2 (frc::MecanumDrive *drive) { // constructor
  mRobotDrive = drive;
}

void VelocityController2::SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps) {
  mGoal = {distance, fps};
  mInitialState = {0_ft, 0_fps};
  mProfile = new frc::TrapezoidProfile<units::feet> (mConstraints, mGoal, mInitialState);
}

bool VelocityController2::DriveTrapezoid () {
  frc::TrapezoidProfile<units::length::feet>::State setpoint = mProfile->Calculate(mTimer.Get()); 
  ForwardAtSpeed(setpoint.velocity);
  // std::cout << "v:" << (double)setpoint.velocity << "  x" << (double)setpoint.position << std::endl;
  return mProfile->IsFinished(mTimer.Get());
}

void VelocityController2::StartMotionTimer() {
  mTimer.Reset();
  mTimer.Start();
}