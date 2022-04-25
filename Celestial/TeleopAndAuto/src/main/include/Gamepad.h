#pragma once

#include <frc/Joystick.h>

class Gamepad : public frc::Joystick {

private:
  Gamepad * mOtherGamepad = nullptr;

public:
  bool GetRawButton(int button) const ;
  bool GetRawButtonPressed(int button) ;
  bool GetRawButtonReleased(int button) ;
  double GetRawAxis(int axis) const ;
  int GetPOV(int pov = 0) const ;
  double GetX() const override;
  double GetY() const ;
  double GetZ() const ;

  Gamepad(int port); // constructor

  void RobotInit(Gamepad * otherGamepad);

};