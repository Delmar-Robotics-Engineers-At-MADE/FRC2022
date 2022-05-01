#include <Elevator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <Constants.h>

static const double kEncoderLimitTop = 300.0;
static const double kEncoderLimitBottom = 20.0;
static const double kElevationForAuto = 55.75;

const static double kPtuned = 0.1;
const static double kItuned = 0.0;
const static double kDtuned = 0.001;
const static double kPIDTolerance = 5;

void Elevator::ManualElevate (frc::Joystick *copilot) {
  // positive power moves nut up, lowering elevation, flattening trajectory, for farther away
  // negative power moves nut down, increasing elevation, for steeper trajectory
  double speed = -copilot->GetY();
  double position = -mEncoder.GetDistance();  // IMPORTANT: invert encoder!
  bool onLimitSwitch = !mLimitSwitch.Get();
  // frc::SmartDashboard::PutNumber("elevator speed", speed);
  frc::SmartDashboard::PutNumber("elevator pos", position);
  // std::cout << "elevator speed "<< speed << std::endl;
  if (speed > 0.0) {
    // don't go past top encoder limit
    if (mHomed && position >= kEncoderLimitTop) {speed = 0.0; std::cout << "elevator at encoder top"<< std::endl;}
  } else if (speed < 0.0) {
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
}

void Elevator::DoOnceInit() {
  std::cout << "elevator DoOnce pos: " << mEncoder.GetDistance() << std::endl;
  CheckHomePosition();
  frc::SmartDashboard::PutNumber("Elevator Homed", mHomed);
}

void Elevator::TelopPeriodic (frc::Joystick *copilot) {
  CheckHomePosition();
  if (mHomed && copilot->GetRawButton(4)) { // shooting at high goal
    // Elevate method is called from Shoot in Shooter, so do nothing here except check bump factor
    mBump = frc::SmartDashboard::GetNumber("Elevator Bump", mBump);
  } else {
    ManualElevate(copilot);
  }
}

double Elevator::CalcHighTargetElevation(double d){
  // used for 2022 regional:
  // double result = (59.0/180.0) * d * d - (1303.0/180.0) * d + 1081.0/10.0;
  // std::cout << "elevation target: " << result << std::endl;
  // std::cout << "distance: " << d << std::endl;
  // std::cout << "intermediate: " << (59.0/180.0) * d * d  << std::endl;
  // std::cout << "intermediate: " << (1303.0/180.0) * d << std::endl;
  // std::cout << "intermediate: " << 1081.0/10.0 << std::endl;

  // for summer 2022:
  // 0.985622 x^3 - 40.8047 x^2 + 558.493 x - 2446.18
  // double result = (0.985622 * d * d * d) - (40.8047 * d * d) + (558.493 * d) - 2446.18;

  // simplify things: just two or three elevations depending on distance
  double result = 0.0;
  if (d < 16.5) {
    result = 83;
  } else { 
    result = 117;
  }

  // allow drivers to adjust calculated elevation
  result += mBump;

  return result;
}

bool Elevator::Elevate (bool hightTarget, double distance) {
  bool result = false;
  if (mHomed && hightTarget) {
    double target = CalcHighTargetElevation(distance);
    double position = -mEncoder.GetDistance(); // invert encoder, as in ManualElevate
    mPIDController->SetSetpoint(target);
    double speed = mPIDController->Calculate(position);  
    if (target > kEncoderLimitTop || target < kEncoderLimitBottom) {std::cout << "elevator target beyond limit"<< std::endl;}
    else {mMotor.Set(speed);}
    result = mPIDController->AtSetpoint();
  } else { // low target
    // for now, allow manual position
  }
  bool TODO_Low_Target_Elevation = false;
  return result;
}

bool Elevator::FixedElevationForAuto() {   
  bool result = false;
  if (mHomed) { 
    double position = -mEncoder.GetDistance(); // invert encoder, as in ManualElevate
    mPIDController->SetSetpoint(kElevationForAuto);
    double speed = mPIDController->Calculate(position);  
    mMotor.Set(speed);
    result = mPIDController->AtSetpoint();
  } else {
    result = true; // just assume elevator is set at right elevation at start of match
  }
  return result;
}

void Elevator::RobotInit() {
  mMotor.ConfigFactoryDefault();
  mMotor.ConfigNominalOutputForward(0, kTimeoutMs);
  mMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
  mMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  mMotor.SetInverted(true);

  mPIDController = new frc2::PIDController (kPtuned, kItuned, kDtuned);
  mPIDController->SetTolerance(kPIDTolerance);
  frc::SmartDashboard::PutData("Elevator PID", mPIDController);  // dashboard should be able to change values
  frc::SmartDashboard::PutNumber("Elevator Bump", mBump);
}

void Elevator::RobotPeriodic() {
  // for testing
  // frc::SmartDashboard::PutNumber("elevator limit", mLimitSwitch.Get());
}
