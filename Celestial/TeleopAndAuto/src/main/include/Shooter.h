#pragma once

#include <frc/Joystick.h>
#include <networktables/NetworkTable.h>


class Shooter {
private:
  constexpr static double kPhi = 20.0;  // angle in degrees of limelight from vertical
  constexpr static double kH1 = 3.0;  // height in feet of limelight from floor
  constexpr static double kH2 = 8.0 - kH1; // height in feet of target ring from limelight
  double mPhi = kPhi;
  double mH2 = kH2;

  std::shared_ptr<nt::NetworkTable> mLimeTable; // for LimeLight

public:
  Shooter (); // constructor
  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot);
  void RobotInit();
};