#pragma once

#include "frc/drive/RobotDriveBase.h"
#include <frc/drive/MecanumDrive.h>
#include <frc/Joystick.h>
#include "AHRS.h"
#include "rev/ColorSensorV3.h" // install online: https://software-metadata.revrobotics.com/REVLib.json
#include <Shooter.h>
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
  void RobotInit(Shooter *shooter);
  void DoOnceInit();
  void RepeatableInit();
  void RotateToTarget(frc::Joystick *pilot, frc::Joystick *copilot); // angle in degrees

private:
  
  AHRS *mAHRS;
  Shooter *mShooter;

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 mColorSensor{i2cPort};
  frc2::PIDController *mPIDControllerLimelight; // for orienting robot with limelight

};
