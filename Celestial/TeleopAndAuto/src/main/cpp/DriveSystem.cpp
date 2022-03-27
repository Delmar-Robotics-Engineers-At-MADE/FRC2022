#include <DriveSystem.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>

const static double kPtunedGyro = 0.009;
const static double kItunedGyro = 0.0;
const static double kDtunedGyro = 0.0;

const static double kPtunedLimelight= 0.02;
const static double kItunedLimelight = 0.0;
const static double kDtunedLimelight = 0.0;
const static double kPIDToleranceLimeLight = 2.0;

const static double kPtunedDrive = 0.1;
const static double kItunedDrive = 0.0; 
const static double kDtunedDrive = 0.001;
const static double kIZtunedDrive = 0.0;
const static double kFFtunedDrive =  0.0;
const static double kMaxOutputDrive = 1.0;
const static double kMinOutputDrive = -1.0;

const static double kSlowSpeedMultiplier = 0.2;
const static double kNormalSpeedMultiplier = 0.9;
const static double kNormalYawMultiplier = 0.75;

const static double kDefaultRotateToTargetRate = 0.5;

// constructor
DriveSystem::DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
                         frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor) :

  MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor) {
                
    // call parent class constructor automatically

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
  std::cout << "driving" << std::endl;
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
      std::cout << "normal: " << y << ", " << x << std::endl;
      DriveCartesian(y*kNormalSpeedMultiplier, -x*kNormalSpeedMultiplier, -z*kNormalYawMultiplier, mAHRS->GetAngle());
    }
    double IR = mColorSensor.GetIR();
    frc::SmartDashboard::PutNumber("Rev Color IR", IR);
  }
}

void SetPIDValues (rev::SparkMaxPIDController *pidController) {
    pidController->SetP     (kPtunedDrive );
    pidController->SetI     (kItunedDrive );
    pidController->SetD     (kDtunedDrive );
    pidController->SetIZone (kIZtunedDrive);
    pidController->SetFF    (kFFtunedDrive);
    pidController->SetOutputRange(kMinOutputDrive, kMaxOutputDrive);
}

void SetEncoderConversion (rev::SparkMaxRelativeEncoder *encoder) {
  // conversion factor from RPM to FPS (feet per second):
  //  on diff drive chassis with toughbox, gearbox is 14:50, and wheel is 6" diameter
  //  1 motor rev = 2 * pi * 3" * 1ft/12" * 1/10.71 = 0.1467 ft
  //  1 RPM = 0.1467 ft/min * 1min/60sec = .0024 FPS
  //
  //  celestial meccanum chassis 11:70 1/4.375
  //  1 motor rev = 2 * pi * 3" * 1ft/12" * 1/4.375 = 0.3590 ft
  //  1 RPM = 0.3590 ft/min * 1min/60sec = .006 FPS
  encoder->SetPositionConversionFactor(0.3590);
  encoder->SetVelocityConversionFactor(0.006);
}

void DriveSystem::RobotInit(Shooter *shooter, 
                rev::SparkMaxPIDController *pidFL, rev::SparkMaxPIDController *pidRL, 
                rev::SparkMaxPIDController *pidFR, rev::SparkMaxPIDController *pidRR,
                rev::SparkMaxRelativeEncoder *encoderFL, rev::SparkMaxRelativeEncoder *encoderRL,
                rev::SparkMaxRelativeEncoder *encoderFR, rev::SparkMaxRelativeEncoder *encoderRR) {

  mShooter = shooter;

  // PID for limelight
  mPIDControllerLimelight = new frc2::PIDController (kPtunedLimelight, kItunedLimelight, kDtunedLimelight);
  mPIDControllerLimelight->SetTolerance(kPIDToleranceLimeLight, kPIDToleranceLimeLight); // degrees
  mPIDControllerLimelight->SetSetpoint(0.0); // always centering target, so always zero

  // allow dashboard adjustment of PID
  frc::SmartDashboard::PutData("Rotate To Target PID", mPIDControllerLimelight);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Front Left", pidFL);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Rear Left", pidRL);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Front Right", pidFR);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Rear Right", pidRR);  // dashboard should be able to change values

  // Spark Max stuff

  SetPIDValues (pidFL);
  SetPIDValues (pidRL);
  SetPIDValues (pidFR);
  SetPIDValues (pidRR);

  SetEncoderConversion(encoderFL);
  SetEncoderConversion(encoderRL);
  SetEncoderConversion(encoderFR);
  SetEncoderConversion(encoderRR);

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