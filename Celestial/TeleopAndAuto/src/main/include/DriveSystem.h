#pragma once

#include <frc/drive/MecanumDrive.h>
#include <frc/Joystick.h>

class DriveSystem : public frc::MecanumDrive {
public:
  void TeleopPeriodic (frc::Joystick *copilot);

};