#pragma once

#include "frc/drive/RobotDriveBase.h"
#include <frc/drive/MecanumDrive.h>
#include <frc/Joystick.h>
#include "AHRS.h"

#include <frc/controller/PIDController.h>
#include <Constants.h>

class DriveSystem : public frc::MecanumDrive {
public:

  DriveSysTargetingState mTargetingState = kDriveUnknownState;

  void TeleopPeriodic (frc::Joystick *copilot);
  DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
              frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor);

  // ~DriveSystem() override = default;

  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot);
  void RobotInit();
  void DoOnceInit();
  void RepeatableInit();
  void RotateToTarget(frc::Joystick *pilot, frc::Joystick *copilot); // angle in degrees

private:
  
  AHRS *mAHRS;


  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  frc2::PIDController *mPIDControllerLimelight; // for orienting robot with limelight

};
