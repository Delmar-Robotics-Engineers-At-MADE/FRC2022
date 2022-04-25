#pragma once

#include "frc/drive/RobotDriveBase.h"
#include <frc/drive/MecanumDrive.h>
// #include <frc/Joystick.h>
#include <Gamepad.h>
#include "AHRS.h"
#include <Shooter.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>
#include "rev/ColorSensorV3.h" // install online: https://software-metadata.revrobotics.com/REVLib.json
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxPIDController.h"
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <units/acceleration.h>

class DriveSystem : public frc::MecanumDrive {
public:

  DriveSysTargetingState mTargetingState = kDriveUnknownState;

  void TeleopPeriodic (Gamepad *copilot);
  DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
              frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor);

  // ~DriveSystem() override = default;

  void TelopPeriodic (Gamepad *pilot, Gamepad *copilot);
  void DoOnceInit();
  void RepeatableInit();
  void RobotPeriodic();
  void RotateToTarget(Gamepad *pilot, Gamepad *copilot); // angle in degrees
  void DriveTrapezoid();
  void DriveSlowForAuto(double x, double y);
  void DriveSlowAndSnapForHanging (Gamepad *pilot);

  void RobotInit(Shooter *shooter, 
                rev::SparkMaxPIDController *pidFL, rev::SparkMaxPIDController *pidRL, 
                rev::SparkMaxPIDController *pidFR, rev::SparkMaxPIDController *pidRR,
                rev::SparkMaxRelativeEncoder *mEncoderFL, rev::SparkMaxRelativeEncoder *mEncodeRL,
                rev::SparkMaxRelativeEncoder *mEncodeFR, rev::SparkMaxRelativeEncoder *mEncodeRR);

private:
  
  AHRS *mAHRS;
  Shooter *mShooter;

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  // rev::ColorSensorV3 mColorSensor{i2cPort};
  frc2::PIDController *mPIDControllerLimelight; // for orienting robot with limelight
  frc2::PIDController *mPIDControllerGyro; // for orienting robot with gyro

  frc::TrapezoidProfile<units::feet>::Constraints mConstraints{5_fps, 5_fps_sq};
  frc::TrapezoidProfile<units::feet>::State mGoal;
  frc::TrapezoidProfile<units::feet>::State mInitialState;
  frc::TrapezoidProfile<units::feet> *mProfile;
  frc::Timer mTimer;  

};
