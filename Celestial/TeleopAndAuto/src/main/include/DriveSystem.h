#pragma once

#include "frc/drive/RobotDriveBase.h"
#include <frc/drive/MecanumDrive.h>
#include <frc/Joystick.h>
#include "AHRS.h"
#include "rev/ColorSensorV3.h" // install online: https://software-metadata.revrobotics.com/REVLib.json

//namespace frc {

class DriveSystem : public frc::MecanumDrive {
public:
  void TeleopPeriodic (frc::Joystick *copilot);
  DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
              frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor);

  // ~DriveSystem() override = default;

  void TelopPeriodic (frc::Joystick *pilot);
  void RobotInit();
  void DoOnceInit();

private:
  
  AHRS *mAHRS;

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 mColorSensor{i2cPort};

};

//} // namespace frc