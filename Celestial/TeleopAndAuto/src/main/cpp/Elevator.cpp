#include <Elevator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <Constants.h>

static const double kEncoderLimitTop = 300.0;
static const double kEncoderLimitBottom = 20.0;

void Elevator::ManualElevate (frc::Joystick *copilot) {
  // positive power moves nut up, lowering elevation, flattening trajectory, for farther away
  // negative power moves nut down, increasing elevation, for steeper trajectory
  double speed = -copilot->GetY();
  double position = -mEncoder.GetDistance();
  bool onLimitSwitch = !mLimitSwitch.Get();
  frc::SmartDashboard::PutNumber("elevator speed", speed);
  frc::SmartDashboard::PutNumber("elevator pos", position);
  std::cout << "elevator speed "<< speed << std::endl;
  if (speed > 0.0) {
    // don't go past top encoder limit
    if (mHomed && position >= kEncoderLimitTop) {speed = 0.0; std::cout << "elevator at encoder top"<< std::endl;}
  } else { // speed < 0
    if (mHomed && position <= kEncoderLimitBottom) {speed = 0.0; std::cout << "elevator at encoder bottom" << std::endl;}
    if (onLimitSwitch) {speed = 0.0; std::cout << "elevator on limit" << std::endl;}
  }
  mMotor.Set(speed);  
}

// bool Elevate(double distance) {
//   bool TODO_Elevate_Per_Target_Distance = false;
//   return true;
// }

void Elevator::CheckHomePosition() {
  bool TODO_Elevator_Limit = false;
  if (mHomed) {
    // no need to check any more
  } else {
    bool onLimitSwitch = !mLimitSwitch.Get();
    mHomed = onLimitSwitch;
    if (mHomed) {
      mEncoder.Reset();
    }
    frc::SmartDashboard::PutBoolean("Elevator Homed", mHomed);
  }
  frc::SmartDashboard::PutBoolean("Elevator", mEncoder.GetDistance());
}

void Elevator::DoOnceInit() {
  std::cout << "elevator DoOnce pos: " << mEncoder.GetDistance() << std::endl;
  CheckHomePosition();
  frc::SmartDashboard::PutNumber("Elevator Homed", mHomed);
}

void Elevator::TelopPeriodic (frc::Joystick *copilot) {
  CheckHomePosition();
  ManualElevate(copilot);
}

bool Elevator::Elevate (double distance) {
  bool TODO_Elevate_Per_Target_Distance = false;
  return true;
}

void Elevator::RobotInit() {
  mMotor.ConfigFactoryDefault();
  mMotor.ConfigNominalOutputForward(0, kTimeoutMs);
  mMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
  mMotor.SetInverted(true);
}

void Elevator::RobotPeriodic() {
  // for testing
  frc::SmartDashboard::PutNumber("elevator limit", mLimitSwitch.Get());
}