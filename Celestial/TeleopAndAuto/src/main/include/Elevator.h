#pragma once

#include <frc/Encoder.h>
#include "ctre/Phoenix.h"
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>

static const int kButtonShooterBlind = 3;
static const int kButtonShooterShort = 4; // was 2 in 2022
static const int kButtonShooterLong = 2; // was 4 in 2022

enum ElevationButtonOption {
  kEBOLongOrMidRange = 0,
  kEBODangerClose,
  kEBOManual
};

enum TargetRange {
  kTRShort = 0,
  kTRMid,
  kTRLong
};

class Elevator {
public:
  void TeleopPeriodic (frc::Joystick *copilot);
  void DoOnceInit();
  void RobotInit();
  void RobotPeriodic();
  bool Elevate (TargetRange shortMidLong, double d); // was (bool hightTarget, double distance);
  bool FixedElevationForAuto();
  // bool Elevate(double distance);

private:
  WPI_TalonSRX mMotor{11};
  bool mHomed = false;
  double mBump = 0.0;  // allow drivers to adjust calculated elevation

  /**
   * The Encoder object is constructed with 4 parameters, the last two being
   * optional.
   *
   * The first two parameters (1, 2 in this case) refer to the ports on the
   * roboRIO which the encoder uses. Because a quadrature encoder has two signal
   * wires, the signal from two DIO ports on the roboRIO are used.
   *
   * The third (optional) parameter is a boolean which defaults to false. If you
   * set this parameter to true, the direction of the encoder will  be reversed,
   * in case it makes more sense mechanically.
   *
   * The final (optional) parameter specifies encoding rate (k1X, k2X, or k4X)
   * and defaults to k4X. Faster (k4X) encoding gives greater positional
   * precision but more noise in the rate.
   */
  frc::Encoder mEncoder{0, 1, false, frc::Encoder::k4X};
  frc::DigitalInput mLimitSwitch{4};
  frc2::PIDController *mPIDController;
  
  void ManualElevate (frc::Joystick *copilot);
  void CheckHomePosition();
  double CalcElevatorTarget(TargetRange shortMidLong, double d);

};