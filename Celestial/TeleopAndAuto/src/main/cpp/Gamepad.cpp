#include <Gamepad.h>
#include <iostream>

// constructor
Gamepad::Gamepad(int port) : Joystick(port) {
  // call parent class constructor
}

void Gamepad::RobotInit(Gamepad * otherGamepad) {
  mOtherGamepad = otherGamepad;
}

bool Gamepad::GetRawButton(int button) const {
  bool result = false;
  switch (button) {
    case 2:
    case 4:
      result = frc::Joystick::GetRawButton(button) || mOtherGamepad->GetRawButton(button);
      break;
  }
  return result;
}


bool Gamepad::GetRawButtonPressed(int button) {
  return false;
}
bool Gamepad::GetRawButtonReleased(int button) {
  return false;
}
double Gamepad::GetRawAxis(int axis) const {
  std::cout << "Gamepad raw axis returning 0" << std::endl;
  return 0.0;
}
int Gamepad::GetPOV(int pov) const {
  return -1;  // nothing pressed
}
double Gamepad::GetX() const {
  std::cout << "Gamepad X returning 0" << std::endl;
  return 0.0;
}
double Gamepad::GetY() const {
  return 0.0;
}
double Gamepad::GetZ() const {
  return 0.0;
}
