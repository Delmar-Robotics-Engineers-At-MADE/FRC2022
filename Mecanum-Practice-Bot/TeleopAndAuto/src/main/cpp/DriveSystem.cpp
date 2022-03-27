#include <DriveSystem.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

const static double kPtunedGyro = 0.009;
const static double kItunedGyro = 0.0;
const static double kDtunedGyro = 0.0;

const static double kPtunedLimelight= 0.02;
const static double kItunedLimelight = 0.0;
const static double kDtunedLimelight = 0.0;
const static double kPIDToleranceLimeLight = 2.0;

const static double kSlowSpeedMultiplier = 0.2;
const static double kNormalSpeedMultiplier = 0.9;
const static double kNormalYawMultiplier = 0.75;

const static double kDefaultRotateToTargetRate = 0.5;

// constructor
DriveSystem::DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
                         frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor) :

  MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor) {
                
    // merely call parent class constructor

  }

void DriveSystem::RotateToTarget (frc::Joystick *pilot, frc::Joystick *copilot) { 
  double rotateRate = 0.0;
  if (mShooter->mTargetSeen) {
    double angleToTarget = mShooter->mTargetAngleHorizontal;
    rotateRate = mPIDControllerLimelight->Calculate(angleToTarget);
    if (mPIDControllerLimelight->AtSetpoint()) {
      mTargetingState = kDriveOnTarget;
    } else {
      mTargetingState = kDriveRotatingToTarget;
    }
  } else { // no target in sight, so rotate until we see it
    mTargetingState = kDriveRotatingToTarget;
  //   rotateRate = -pilot->GetZ();
  //   if (rotateRate == 0.0) {
  //     rotateRate = -copilot->GetZ();
  //   }
  //   if (rotateRate == 0.0) {
  //     rotateRate = copilot->GetX();
  //   }
  //   if (rotateRate == 0.0) {
  //     rotateRate = kDefaultRotateToTargetRate;
  //   }
  }
  DriveCartesian(0, 0, rotateRate);
}

void DriveSystem::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot){
  bool shooting = (copilot->GetRawButton(4) || copilot->GetRawButton(2));
  if (shooting) {
    RotateToTarget(pilot, copilot);
  } else {
    mTargetingState = kDriveNotTargeting;
    double x = pilot->GetX();
    double y = pilot->GetY();
    double z = pilot->GetZ();
    if (pilot->GetRawButton(6) && pilot->GetRawButton(4)) {
      mAHRS->ZeroYaw();   // use current robot orientation as field forward
    } else if (pilot->GetRawButton(5) || pilot->GetRawButton(6)) {
      DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -z*kSlowSpeedMultiplier, mAHRS->GetAngle());
    } else {
      DriveCartesian(y*kNormalSpeedMultiplier, -x*kNormalSpeedMultiplier, -z*kNormalYawMultiplier, mAHRS->GetAngle());
    }
    double IR = mColorSensor.GetIR();
    frc::SmartDashboard::PutNumber("Rev Color IR", IR);
  }
}

void DriveSystem::RobotInit(Shooter *shooter) {

  mShooter = shooter;

  mPIDControllerLimelight = new frc2::PIDController (kPtunedLimelight, kItunedLimelight, kDtunedLimelight);
  mPIDControllerLimelight->SetTolerance(kPIDToleranceLimeLight, kPIDToleranceLimeLight); // degrees
  mPIDControllerLimelight->SetSetpoint(0.0); // always centering target, so always zero
  frc::SmartDashboard::PutData("Rotate To Target PID", mPIDControllerLimelight);  // dashboard should be able to change values

  try{
      mAHRS = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception &ex) {
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char *p_err_msg = err_msg.c_str();
      FRC_ReportError(0, "{}", p_err_msg); // frc::DriverStation::ReportError(p_err_msg);
    }
}

void DriveSystem::DoOnceInit()  {

}

void DriveSystem::RepeatableInit() {
  // do this whenever we start either auto or teleop
  mAHRS->ZeroYaw();   // use current robot orientation as field forward
}