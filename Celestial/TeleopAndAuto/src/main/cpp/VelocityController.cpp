#include <VelocityController.h>
#include <algorithm>

static constexpr units::second_t kDt = 20_ms;

double VelocityController::ForwardAtSpeed (units::feet_per_second_t feetPerSec) {
    units::feet_per_second_t conversionFactor = (units::feet_per_second_t)3.2;
    double speedInFeetPerSec = feetPerSec/conversionFactor;
    mRobotDrive->DriveCartesian(-speedInFeetPerSec, 0, 0);
}

VelocityController::VelocityController (frc::MecanumDrive *drive) { // constructor
  mRobotDrive = drive;
}

void VelocityController::SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps) {
  mGoal = {distance, fps};
}

bool VelocityController::DriveTrapezoid () {
  frc::TrapezoidProfile<units::feet> profile{mConstraints, mGoal, mSetpoint};
  mSetpoint = profile.Calculate(kDt); 
  if (mSetpoint.position >= mGoal.position) {
    return true;
  } else {
    ForwardAtSpeed(mSetpoint.velocity);
    return false;
  }
}