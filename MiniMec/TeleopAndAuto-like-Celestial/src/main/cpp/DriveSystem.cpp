#include <DriveSystem.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>

const static double kPtunedGyro = 0.01;
const static double kItunedGyro = 0.0;
const static double kDtunedGyro = 0.0;

const static double kPtunedLimelight= 0.02;
const static double kItunedLimelight = 0.0;
const static double kDtunedLimelight = 0.0;
const static double kPIDToleranceLimeLight = 5.0;

const static double kPtunedDrive = 0.1;
const static double kItunedDrive = 0.0; 
const static double kDtunedDrive = 0.001;
const static double kIZtunedDrive = 0.0;
const static double kFFtunedDrive =  0.0;
const static double kMaxOutputDrive = 1.0;
const static double kMinOutputDrive = -1.0;

const static double kSlowSpeedMultiplier = 0.3;
const static double kAutoSpeedMultiplier = 0.2;
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
  double angleToTarget = mShooter->mTargetAngleHorizontal;
  std::cout << "rotating, target seen: " << mShooter->mTargetSeen << std::endl;
  switch (mTargetingState) {
    default:
      if (mShooter->mTargetSeen) {
        mTargetingState = kDriveRotatingToTarget;
      } else { // no target in sight; allow copilot to rotate
        mTargetingState = kDriveWaitingForTarget;
      }
      break;
    case kDriveRotatingToTarget: // target has been seen, so no more driver control
      rotateRate = mPIDControllerLimelight->Calculate(angleToTarget);
      if (mPIDControllerLimelight->AtSetpoint()) {
        mTargetingState = kDriveOnTarget;
      } else {
        mTargetingState = kDriveRotatingToTarget;
      }
      break;
    case kDriveOnTarget:
      // settle; don't keep moving
      rotateRate = 0.0;
      break;
    case kDriveWaitingForTarget:
      // let copilot rotate robot toward target until it locks on
      if (mShooter->mTargetSeen) {
        mTargetingState = kDriveRotatingToTarget;
      } else {
        rotateRate = copilot->GetX();
      }
      break;
  }
  DriveCartesian(0, 0, rotateRate);
}

void DriveSystem::DriveSlowAndSnapForHanging (frc::Joystick *pilot){
  // offset everything 180 deg. to avoid discontinuity at 0/360
  double x = pilot->GetX();
  double y = pilot->GetY();
  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading + 180.0);
  DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -rotateRate, currHeading);
  // return mPIDControllerGyro->AtSetpoint();
}

void DriveSystem::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot){
  bool shooting = (copilot->GetRawButton(4) || copilot->GetRawButton(2));
  // std::cout << "driving" << std::endl;
  if (shooting) {
    std::cout << "drivesys: shooting" << std::endl;
    RotateToTarget(pilot, copilot);
  } else {
    mTargetingState = kDriveNotTargeting;
    double x = pilot->GetX();
    double y = pilot->GetY();
    double z = pilot->GetZ();
    if (pilot->GetRawButton(6) && pilot->GetRawButton(4)) {
      mAHRS->ZeroYaw();   // use current robot orientation as field forward
    } else if (pilot->GetRawButton(5) || pilot->GetRawButton(6)) { // drive slowly
      DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -z*kSlowSpeedMultiplier, mAHRS->GetAngle());
    } else if (pilot->GetRawButton(2)) { // drive slowly and snap to hanging line
      DriveSlowAndSnapForHanging (pilot);
    } else {
      // std::cout << "normal: " << y << ", " << x << std::endl;
      DriveCartesian(y*kNormalSpeedMultiplier, -x*kNormalSpeedMultiplier, -z*kNormalYawMultiplier, mAHRS->GetAngle());
    }
    // double IR = mColorSensor.GetIR();
    // frc::SmartDashboard::PutNumber("Rev Color IR", IR);
  }
}

// void SetPIDValues (rev::SparkMaxPIDController *pidController) {
//     pidController->SetP     (kPtunedDrive );
//     pidController->SetI     (kItunedDrive );
//     pidController->SetD     (kDtunedDrive );
//     pidController->SetIZone (kIZtunedDrive);
//     pidController->SetFF    (kFFtunedDrive);
//     pidController->SetOutputRange(kMinOutputDrive, kMaxOutputDrive);
// }

// void SetEncoderConversion (rev::SparkMaxRelativeEncoder *encoder) {
//   // conversion factor from RPM to FPS (feet per second):
//   //  on diff drive chassis with toughbox, gearbox is 14:50, and wheel is 6" diameter
//   //  1 motor rev = 2 * pi * 3" * 1ft/12" * 1/10.71 = 0.1467 ft
//   //  1 RPM = 0.1467 ft/min * 1min/60sec = .0024 FPS
//   //
//   //  celestial meccanum chassis 11:70 1/4.375
//   //  1 motor rev = 2 * pi * 3" * 1ft/12" * 1/4.375 = 0.3590 ft
//   //  1 RPM = 0.3590 ft/min * 1min/60sec = .006 FPS
//   encoder->SetPositionConversionFactor(0.3590);
//   encoder->SetVelocityConversionFactor(0.006);
// }

void DriveSystem::RobotInit(Shooter *shooter /*, 
                rev::SparkMaxPIDController *pidFL, rev::SparkMaxPIDController *pidRL, 
                rev::SparkMaxPIDController *pidFR, rev::SparkMaxPIDController *pidRR,
                rev::SparkMaxRelativeEncoder *encoderFL, rev::SparkMaxRelativeEncoder *encoderRL,
                rev::SparkMaxRelativeEncoder *encoderFR, rev::SparkMaxRelativeEncoder *encoderRR */) {

  mShooter = shooter;

  // PID for limelight
  mPIDControllerLimelight = new frc2::PIDController (kPtunedLimelight, kItunedLimelight, kDtunedLimelight);
  mPIDControllerLimelight->SetTolerance(kPIDToleranceLimeLight, kPIDToleranceLimeLight); // degrees
  mPIDControllerLimelight->SetSetpoint(0.0); // always centering target, so always zero

  // PID for snap to hanging
  
  mPIDControllerGyro = new frc2::PIDController (kPtunedGyro, kItunedGyro, kDtunedGyro);
  mPIDControllerGyro->SetTolerance(8, 8);  // within 8 degrees of direction is considered on set point
  mPIDControllerGyro->SetSetpoint(180.0); // will use this to snap to zero, by passing angle + 180 as error

  // allow dashboard adjustment of PID
  // frc::SmartDashboard::PutData("Rotate PID", mPIDControllerLimelight);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Front Left", pidFL);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Rear Left", pidRL);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Front Right", pidFR);  // dashboard should be able to change values
  // frc::SmartDashboard::PutData("Rear Right", pidRR);  // dashboard should be able to change values
  frc::SmartDashboard::PutData("Gyro PID", mPIDControllerGyro);

  // Spark Max stuff

  // SetPIDValues (pidFL);
  // SetPIDValues (pidRL);
  // SetPIDValues (pidFR);
  // SetPIDValues (pidRR);

  // SetEncoderConversion(encoderFL);
  // SetEncoderConversion(encoderRL);
  // SetEncoderConversion(encoderFR);
  // SetEncoderConversion(encoderRR);

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

void DriveSystem::RobotPeriodic() {
  // for debugging
  frc::SmartDashboard::PutNumber("Heading", mAHRS->GetAngle());
}

void DriveSystem::DriveTrapezoid() {
  // future TODO
}

void DriveSystem::DriveSlowForAuto(double x, double y) {
  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading + 180.0);  // offset by 180 to avoid discontinuity
  // DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -rotateRate, currHeading);
  DriveCartesian(-y*kAutoSpeedMultiplier, x*kAutoSpeedMultiplier, -rotateRate, currHeading);
}