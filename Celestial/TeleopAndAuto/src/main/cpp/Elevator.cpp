#include <Elevator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <Constants.h>

void Elevator::ManualElevate (frc::Joystick *copilot) {
  mMotor.Set(copilot->GetY());
}

// bool Elevate(double distance) {
//   bool TODO_Elevate_Per_Target_Distance = false;
//   return true;
// }

void Elevator::CheckHomePosition() {
  bool TODO_Check_Limit_Switch = false;
  mHomed = false; // for now allow manual control of elevator
  frc::SmartDashboard::PutNumber("Elevator", mEncoder.GetDistance());
}

void Elevator::DoOnceInit() {
  bool TODO_Move_Reset_To_Check_Home_Position = false;
  mEncoder.Reset();
  std::cout << "elevator DoOnce pos: " << mEncoder.GetDistance() << std::endl;
  CheckHomePosition();
}

void Elevator::TelopPeriodic (frc::Joystick *copilot) {
  if (!mHomed) { // allowed only if not homed
    ManualElevate(copilot);
    CheckHomePosition();
  }
}

bool Elevator::Elevate (double distance) {
  bool TODO_Elevate_Per_Target_Distance = false;
  return true;
}

void Elevator::RobotInit() {
  mMotor.ConfigFactoryDefault();
  mMotor.ConfigNominalOutputForward(0, kTimeoutMs);
  mMotor.ConfigNominalOutputReverse(0, kTimeoutMs);
}