#include <DriveSystem.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>
#include <frc/MathUtil.h>
#include <units/voltage.h>

// #define SUMMER
#define FALL

const static double kPtunedGyro = 0.005;
const static double kItunedGyro = 0.0;
const static double kDtunedGyro = 0.0;

const static double kPtunedLimelight= 0.005;
const static double kItunedLimelight = 0.0;
const static double kDtunedLimelight = 0.0;
const static double kPIDToleranceLimeLight = 3.0;

const static double kPtunedRaspPi= 0.25;
const static double kItunedRaspPi = 0.0;
const static double kDtunedRaspPi = 0.0;
const static double kPIDToleranceRaspPi = 0.05;

static const int kButtonDriveFieldReset = 3;

static const int kLoggingFrequency = 5; // log every Nth time through loop

// from SysID tool
constexpr auto kSysIdkS = 0.14615_V;
constexpr auto kSysIdkV = 0.97133_V * 1_s / 1_tr;  // 12_V / kShooterFreeRPS;
constexpr auto kSysIdkA = 0.16704_V * 1_s * 1_s / 1_tr; // volts / acceleration
const static double kSysIdkP = 2.7382E-07;
const static double kSysIdkD = 0;

// followed this ref for calculating PID numbers from SysID numbers: https://www.chiefdelphi.com/t/how-to-do-characterization-for-velocity-control-for-spark-max-and-neo/378990/6
// SysID gives: kSysIdkP / (12.0 * 60); // this is a very small number!

// Try this reference instead: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Velocity%20PID%20Control/src/main/cpp/Robot.cpp

const static double kPtunedDrive = .00005; 
const static double kItunedDrive = 0.0;
const static double kDtunedDrive = 0.0;
const static double kIZtunedDrive = 0.0;
const static double kFFtunedDrive =  0.001; 
const static double kMaxOutputDrive = 1.0;
const static double kMinOutputDrive = -1.0;
const double kMaxRPM = 5700;
// const auto maxRPS = 5700_tr / 1_s; // 5700 RPM; see Frisbeebot example for SimpleMotorFeedforward
// const auto maxRPSPS = 10000_tr / 1_s / 1_s; // 0 to 10000 RPM in 1 sec
// const auto maxVoltage = 12_V;
// const auto averageSetpointRPS = 1800_tr / 1_s; // 1800 RPM

const static double kSlowSpeedMultiplier = 0.1;
const static double kAutoSpeedMultiplier = 0.05;
const static double kNormalSpeedMultiplierY = 0.1;
const static double kNormalSpeedMultiplierX = 0.3;
const static double kNormalYawMultiplier = 0.1;
//const static double kDemoYawMultiplier = 0.25;
// const static double kDemoSpeedMultX = 0.1;
// const static double kDemoSpeedMultY = 0.05;

const static double kDefaultRotateToTargetRate = 0.04;
const static double kRotationDeadZone = 0.1;

// constructor
DriveSystem::DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
                         frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor) :

  // parent class constructor
  MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor) ,

  // FeedForward constructor, for setting PID using numbers from SysID
  //mFeedForwardCalculator = new SimpleMotorFeedforward<units::turns>(kSysIdkS, kSysIdkV, kSysIdkA);
  mFeedForwardCalculator(kSysIdkS, kSysIdkV, kSysIdkA) // , kSysIdkA ?

  // constructor code
  {
    // m_controller.SetTolerance(kShooterToleranceRPS.value());
    // m_shooterEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
    // SetSetpoint(kShooterTargetRPS.value());
  }
    

void DriveSystem::RotateToTarget (/*frc::Joystick *pilot, frc::Joystick *copilot*/) { 
  double rotateRate = 0.0;
  double angleToTarget = mShooter->mTargetAngleHorizontal;
  // std::cout << "rotating, target seen: " << mShooter->mTargetSeen << std::endl;
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
      // } else {
      //   rotateRate = copilot->GetX();
      }
      break;
  }
  MyDriveCartesian(0, 0, rotateRate, 0);
}

#ifdef SUMMER

void DriveSystem::RotateToBall(RaspPi *rPi) {
  double rotateRate = 0.0;
  // for Summer demo, Intake calls rPi->CheckForBall()
  double angleToTarget = rPi->mNearestBallX; 
  // frc::SmartDashboard::PutNumber("Ball X", angleToTarget);
  // mTargetingState = kDriveOnTarget;
  // frc::SmartDashboard::PutNumber("Rotate State", mTargetingState);
  switch (mTargetingState) {
    default:
      if (rPi->mBallAhead) {
        mTargetingState = kDriveRotatingToTarget;
      } else { 
        mTargetingState = kDriveWaitingForTarget;
      }
      break;
    case kDriveRotatingToTarget: 
      rotateRate = mPIDControllerRaspPi->Calculate(angleToTarget);
      if (!rPi->mBallAhead) { // when lose ball, stop rotating
        mTargetingState = kDriveWaitingForTarget;
      } 
      break;
    case kDriveWaitingForTarget:
      rotateRate = 0;
      if (rPi->mBallAhead) {
        mTargetingState = kDriveRotatingToTarget;
      }
      break;
  }
  // frc::SmartDashboard::PutNumber("Rotate", rotateRate);
  MyDriveCartesian(0, 0, rotateRate, 0);
}

#else

double DriveSystem::RotateToBall(RaspPi *rPi) {
  double rotateRate = 0.0;
  double angleToTarget = rPi->mNearestBallX; 
  if (rPi->mBallAhead) {
      rotateRate = mPIDControllerRaspPi->Calculate(angleToTarget);
  } else {
      rotateRate = 0;
  }
  return rotateRate;
}

#endif

void DriveSystem::DriveSlowAndSnapForHanging (frc::Joystick *pilot){
  // offset everything 180 deg. to avoid discontinuity at 0/360
  double x = pilot->GetX();
  double y = pilot->GetY();
  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading + 180.0);
  DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -rotateRate, currHeading);
  // return mPIDControllerGyro->AtSetpoint();
}

void DriveSystem::MyDriveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
  ySpeed = ApplyDeadband(ySpeed, m_deadband);
  xSpeed = ApplyDeadband(xSpeed, m_deadband);
  auto [frontLeft, frontRight, rearLeft, rearRight] = DriveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);
  mPIDFrontLeft->SetReference(frontLeft*kMaxRPM, rev::ControlType::kVelocity); // velocity in RPM
  mPIDFrontRight->SetReference(frontRight*kMaxRPM, rev::ControlType::kVelocity); // velocity in RPM
  mPIDRearLeft->SetReference(rearLeft*kMaxRPM, rev::ControlType::kVelocity); // velocity in RPM
  mPIDRearRight->SetReference(rearRight*kMaxRPM, rev::ControlType::kVelocity); // velocity in RPM
  Feed();
  frc::SmartDashboard::PutNumber("FL", frontLeft*kMaxRPM);
  frc::SmartDashboard::PutNumber("FR", frontRight*kMaxRPM);
  frc::SmartDashboard::PutNumber("RL", rearLeft*kMaxRPM);
  frc::SmartDashboard::PutNumber("RR", rearRight*kMaxRPM);

  // front right motor has been problematic so check it
  double frActualV = mEncoderFrontRight->GetVelocity()*60;  // 60 because RPM vs RPS
  // bool frCheck = (abs(frActualV - frontRight*kMaxRPM)); // error greater than 25%
  frc::SmartDashboard::PutNumber("FR Actual V", frActualV);
  // frc::SmartDashboard::PutBoolean("FR OK", frCheck);
  if (mLoggingEnable) {
    if (mLoggingCount == kLoggingFrequency) {
      mLoggingCount = 0;
      mLogFRVSetpoint.Append(frontRight*kMaxRPM);
      mLogFRVActual.Append(frActualV);
    } else {
      mLoggingCount++;
    }
  }

}


#if defined(SUMMER)

void DriveSystem::TeleopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, RaspPi *rPi){
  bool shooting = (copilot->GetRawButton(4) || copilot->GetRawButton(2));

  // monitor intake's summer demo mode
  bool trackBall = (mIntake->mFetchState == kFBSBallAhead
                 || mIntake->mFetchState == kFBSBallGone);

  bool rotate180ForSummer = (mIntake->mFetchState == kFBSRotating);
  
  shooting = (mIntake->mFetchState == kFBSShooting);

  // std::cout << "driving" << std::endl;

  if (trackBall) {
    RotateToBall(rPi);
  } else if (rotate180ForSummer) {
    Rotate180ForSummer();
  } else if (shooting) {
    RotateToTarget(pilot, copilot);
  } else {
    mTargetingState = kDriveNotTargeting;
    if (pilot->GetRawButton(6) && pilot->GetRawButton(4)) {
        mAHRS->ZeroYaw();   // use current robot orientation as field forward
    } else {
      double x = pilot->GetX();
      double y = pilot->GetY();
      double z = pilot->GetZ();
      bool overrideSummerSafeBox = pilot->GetRawButton(4);
      if (overrideSummerSafeBox) {
        MyDriveCartesian(-y*kNormalSpeedMultiplier, x*kNormalSpeedMultiplier, -z*kNormalYawMultiplier, mAHRS->GetAngle());
      } else { // summer demo
        DriveSlowForSummer(x, y);
      }
    }
  }
}

#elif defined(FALL)

void DriveSystem::TeleopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, RaspPi *rPi){

  bool trackBall = (mIntake->mFetchState == kFBSBallAhead
                 || mIntake->mFetchState == kFBSBallGone);

  bool fieldRelative = (pilot->GetZ() > 0.5);
  bool robotRelative = !fieldRelative;
  frc::SmartDashboard::PutBoolean("Robot Relative", robotRelative);
  bool fallDemo = fieldRelative; // for fall demo, field relative setting is same as "enable demo"

  mTargetingState = kDriveNotTargeting;
  double x = fallDemo ? 0.0 : pilot->GetX();
  double y = fallDemo ? 0.0 : pilot->GetY();
  double z = fallDemo ? 0.0 : pilot->GetRawAxis(3);  // with Logitech use: GetZ();
  bool driveSlowly = false;

  // use rotation to keep robot at constant angle, unless driver is trying to spin
  double currHeading = mAHRS->GetAngle();
  double rotateRate = 0.0;
  if (trackBall) { // auto intake button, so rotation goverened by rasp. pi
    rotateRate = RotateToBall(rPi);
    // do not update lock heading, so we will rotate back, was... mPIDControllerGyro->SetSetpoint(currHeading + 180.0); // update lock heading
  } else if (abs(z) > kRotationDeadZone) { // driver is rotating
    // std::cout << "driver rotating" << std::endl;
    rotateRate = driveSlowly ? -z*kSlowSpeedMultiplier : -z*kNormalYawMultiplier;
    mPIDControllerGyro->SetSetpoint(currHeading + 180.0); // update lock heading
  } else { // driver not rotating, so keep robot on lock heading
    // std::cout << "driver NOT rotating" << std::endl;
    rotateRate = -(mPIDControllerGyro->Calculate(currHeading + 180.0));  // offset by 180 to avoid discontinuity
    // clip rate to max rotation
    rotateRate = std::min(kDefaultRotateToTargetRate, rotateRate);
    rotateRate = std::max(-kDefaultRotateToTargetRate, rotateRate);
  }
  frc::SmartDashboard::PutNumber("Lck Hd -180", mPIDControllerGyro->GetSetpoint() - 180);
  // frc::SmartDashboard::PutNumber("Rot Rate", rotateRate);

  double angle = 0.0;      // for robot relative
  if (fieldRelative) {
    angle = currHeading;   // for field relative
  }

  if (/*pilot->GetRawButton(kButtonIntakeAuto) &&*/ pilot->GetRawButton(kButtonDriveFieldReset)) { // logitech: 6 and 4
    mAHRS->ZeroYaw();   // use current robot orientation as field forward
  } else if (driveSlowly) { // drive slowly
    MyDriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, rotateRate, angle);
  // } else if (pilot->GetRawButton(2)) { // drive slowly and snap to hanging line
  //   DriveSlowAndSnapForHanging (pilot);
  } else {
    // std::cout << "normal: " << y << ", " << x << std::endl;
    MyDriveCartesian(y*kNormalSpeedMultiplierY, -x*kNormalSpeedMultiplierX, rotateRate, angle);
  }
  
}

#else

void DriveSystem::TeleopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, RaspPi *rPi){

  bool shooting = (copilot->GetRawButton(4) || copilot->GetRawButton(2));
  // std::cout << "driving" << std::endl;

  bool trackBall = pilot->GetRawButton(kButtonIntakeAuto) && 
      (mIntake->mFetchState == kFBSBallAhead || mIntake->mFetchState == kFBSBallGone);

  bool fieldRelative = (pilot->GetZ() > 0.5);
  frc::SmartDashboard::PutBoolean("Robot Relative", !fieldRelative);

  if (shooting) {
    // std::cout << "drivesys: shooting" << std::endl;
    RotateToTarget(/*pilot, copilot*/);
  } else {
    mTargetingState = kDriveNotTargeting;
    double x = pilot->GetX();
    double y = pilot->GetY();
    double z = pilot->GetRawAxis(3);  // with Logitech use: GetZ();
    bool driveSlowly = false;

    // use rotation to keep robot at constant angle, unless driver is trying to spin
    double currHeading = mAHRS->GetAngle();
    double rotateRate = 0.0;
    if (trackBall) { // auto intake button, so rotation goverened by rasp. pi
      rotateRate = RotateToBall(rPi);
      mPIDControllerGyro->SetSetpoint(currHeading + 180.0); // update lock heading
    } else if (abs(z) > kRotationDeadZone) { // driver is rotating
      // std::cout << "driver rotating" << std::endl;
      rotateRate = driveSlowly ? -z*kSlowSpeedMultiplier : -z*kNormalYawMultiplier;
      mPIDControllerGyro->SetSetpoint(currHeading + 180.0); // update lock heading
    } else { // driver not rotating, so keep robot on lock heading
      // std::cout << "driver NOT rotating" << std::endl;
      rotateRate = -(mPIDControllerGyro->Calculate(currHeading + 180.0));  // offset by 180 to avoid discontinuity
      // clip rate to max rotation
      rotateRate = std::min(kDefaultRotateToTargetRate, rotateRate);
      rotateRate = std::max(-kDefaultRotateToTargetRate, rotateRate);
    }
    frc::SmartDashboard::PutNumber("Lck Hd -180", mPIDControllerGyro->GetSetpoint() - 180);
    // frc::SmartDashboard::PutNumber("Rot Rate", rotateRate);

    double angle = 0.0;      // for robot relative
    if (fieldRelative) {
      angle = currHeading;   // for field relative
    }

    if (/*pilot->GetRawButton(kButtonIntakeAuto) &&*/ pilot->GetRawButton(kButtonDriveFieldReset)) { // logitech: 6 and 4
      mAHRS->ZeroYaw();   // use current robot orientation as field forward
    } else if (driveSlowly) { // drive slowly
      MyDriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, rotateRate, angle);
    // } else if (pilot->GetRawButton(2)) { // drive slowly and snap to hanging line
    //   DriveSlowAndSnapForHanging (pilot);
    } else {
      // std::cout << "normal: " << y << ", " << x << std::endl;
      MyDriveCartesian(y*kNormalSpeedMultiplierY, -x*kNormalSpeedMultiplierX, rotateRate, angle);
    }
  }
}

#endif

void DriveSystem::SetLimits (rev::CANSparkMax *motor) {
  motor->SetSmartCurrentLimit(60);
  motor->SetClosedLoopRampRate(0.5);
}

void DriveSystem::SetPIDValues (rev::SparkMaxPIDController *pidController) {
  // ref for calculating using SysID numbers: https://www.chiefdelphi.com/t/how-to-do-characterization-for-velocity-control-for-spark-max-and-neo/378990/6
  // According to Frisbeebot, mFeedForwardCalculator.Calculate returns volts
  // auto feedForward =  mFeedForwardCalculator.Calculate(averageSetpointRPS, maxRPS)
  //     / (maxVoltage * maxRPS);
  // auto feedForward =  0.0;
  // auto feedForwardV = mFeedForwardCalculator.Calculate(averageSetpointRPS, maxRPSPS);
  // auto feedForward = feedForwardV * maxRPS / maxVoltage;
  // std::cout << "setting Spark Max FF: " << feedForward.value() << std::endl;

  // retry if any of these do not succeed, especially P
  for (int i = 1; i <= 5; i++) {
    pidController->SetP     (kPtunedDrive );
    pidController->SetI     (kItunedDrive );
    pidController->SetD     (kDtunedDrive );
    pidController->SetIZone (kIZtunedDrive);
    pidController->SetFF    (kFFtunedDrive);  // takes a gain, which is unitless, was feedForward.value()
    pidController->SetOutputRange(kMinOutputDrive, kMaxOutputDrive);
    
    sleep(0.050); // wait for set to stick
    double pcheck = pidController->GetP();
    if (pcheck/kPtunedDrive < 1.1) { // good!  Don't need to retry
      std::cout << "Drive motor P set to " << pcheck << " on try " << i << std::endl;
      break;
    } else { // didn't take?
      std::cout << "Drive motor P NOT YET SET, is still " << pcheck << " on try " << i << ", should be " << kPtunedDrive << std::endl;
    }
  }

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

void DriveSystem::RobotInit(Shooter *shooter, Intake *intake,
                rev::CANSparkMax *fl, rev::CANSparkMax *rl, 
                rev::CANSparkMax *fr, rev::CANSparkMax *rr,
                rev::SparkMaxPIDController *pidFL, rev::SparkMaxPIDController *pidRL, 
                rev::SparkMaxPIDController *pidFR, rev::SparkMaxPIDController *pidRR,
                rev::SparkMaxRelativeEncoder *encoderFL, rev::SparkMaxRelativeEncoder *encoderRL,
                rev::SparkMaxRelativeEncoder *encoderFR, rev::SparkMaxRelativeEncoder *encoderRR) {

  mShooter = shooter;
  mIntake = intake; // for summer

  // even though we are a subclass of MecanumDrive, the motors are private, so we need
  // our own additional handles to them
  mFrontLeft  = fl;
  mRearLeft   = rl;
  mFrontRight = fr;
  mRearRight  = rr;

  //m_deadband = 0.07; // default is 0.02;

  mPIDFrontLeft  = pidFL;
  mPIDRearLeft   = pidRL;
  mPIDFrontRight = pidFR;
  mPIDRearRight  = pidRR;

  mEncoderFrontRight = encoderFR;

  // PID for limelight
  mPIDControllerLimelight = new frc2::PIDController (kPtunedLimelight, kItunedLimelight, kDtunedLimelight);
  mPIDControllerLimelight->SetTolerance(kPIDToleranceLimeLight, kPIDToleranceLimeLight); // degrees
  mPIDControllerLimelight->SetSetpoint(0.0); // always centering target, so always zero

  // PID for rotating to nearest ball (Summer demo)
  mPIDControllerRaspPi = new frc2::PIDController (kPtunedRaspPi, kItunedRaspPi, kDtunedRaspPi);
  mPIDControllerRaspPi->SetTolerance(kPIDToleranceRaspPi, kPIDToleranceRaspPi); // degrees
  mPIDControllerRaspPi->SetSetpoint(0.5); // always centering target
  
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
  
  // frc::SmartDashboard::PutData("Gyro PID", mPIDControllerGyro);
  frc::SmartDashboard::PutData("RPi PID", mPIDControllerRaspPi);

  // Spark Max stuff


  mFrontLeft ->RestoreFactoryDefaults();
  mRearLeft  ->RestoreFactoryDefaults();
  mFrontRight->RestoreFactoryDefaults();
  mRearRight ->RestoreFactoryDefaults();

  mFrontRight->SetInverted(true);
  mRearRight ->SetInverted(true);
  mFrontLeft ->SetInverted(false);
  mRearLeft  ->SetInverted(false);

  mFrontLeft ->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mRearLeft  ->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mFrontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  mRearRight ->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  SetLimits(mFrontLeft);
  SetLimits(mRearLeft);
  SetLimits(mFrontRight);
  SetLimits(mRearRight);

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

  // logging
  frc::DataLogManager::Start();
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  mLogFRVSetpoint = wpi::log::DoubleLogEntry(log, "/8077/FRVSetpoint");
  mLogFRVActual = wpi::log::DoubleLogEntry(log, "/8077/FRVActual");
  frc::SmartDashboard::PutBoolean("Logging", mLoggingEnable);
}

void DriveSystem::RepeatableInit() {
  // do this whenever we start either auto or teleop
  mAHRS->ZeroYaw();   // use current robot orientation as field forward
  mPIDControllerGyro->SetSetpoint(180.0);
  mLoggingEnable = frc::SmartDashboard::GetBoolean("Logging", mLoggingEnable);
}

void DriveSystem::RobotPeriodic() {
  // for debugging

  frc::SmartDashboard::PutNumber("Heading", mAHRS->GetAngle());

  // rev::ColorSensorV3::RawColor rawColor = mColorSensor.GetRawColor();
  // frc::SmartDashboard::PutNumber("Color R", rawColor.red);
  // frc::SmartDashboard::PutNumber("Color G", rawColor.green);
  // frc::SmartDashboard::PutNumber("Color B", rawColor.blue);
  // frc::SmartDashboard::PutNumber("Color IR", rawColor.ir);
}

void DriveSystem::DriveTrapezoid() {
  // future TODO
}

void DriveSystem::DriveSlowForAuto(double x, double y) {
  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading + 180.0);  // offset by 180 to avoid discontinuity
  // DriveCartesian(y*kSlowSpeedMultiplier, -x*kSlowSpeedMultiplier, -rotateRate, currHeading);
  MyDriveCartesian(-y*kAutoSpeedMultiplier, x*kAutoSpeedMultiplier*2, -rotateRate, currHeading);
}

void DriveSystem::CheckColorForAllClear(bool isWhite, bool isRed, bool isBlue) {
  if (!isWhite && !isRed && !isBlue) {
    mDemoDriveState = kSDDClear;
  }
}

#ifdef SUMMER
void DriveSystem::DriveSlowForSummer(double x, double y) {

  // check for boundary lines for summer demo
  rev::ColorSensorV3::RawColor rawColor = mColorSensor.GetRawColor();
  bool isWhite = rawColor.blue > 1400 && rawColor.red > 1400;
  bool isRed = rawColor.blue < 1000 && rawColor.red > 1400;
  bool isBlue = rawColor.blue > 1400 && rawColor.red < 1000;

  switch (mDemoDriveState) {
    default:
      if (isRed) {
        if (y < 0) {mDemoDriveState = kSDDOnFrontBoundary;}
        else {mDemoDriveState = kSDDOnRearBoundary;}
      } else if (isBlue) {
        if (x > 0) {mDemoDriveState = kSDDOnLeftBoundary;}
        else {mDemoDriveState = kSDDOnRightBoundary;}
      }
      break;
    // case kSDDOnEdgeBoundary:
    //   x = 0; y = 0; // stop and don't move again until someone manually gets us off boundary
    //   break;
    case kSDDOnLeftBoundary:
      if (x > 0) {x = 0;} // don't permit more forward motion
      CheckColorForAllClear(isWhite, isRed, isBlue);
      break;
		case kSDDOnRightBoundary:
      if (x < 0) {x = 0;} // don't permit more forward motion
      CheckColorForAllClear(isWhite, isRed, isBlue);
      break;
    case kSDDOnFrontBoundary:
      if (y < 0) {y = 0;} // don't permit more forward motion
      CheckColorForAllClear(isWhite, isRed, isBlue);
      break;
    case kSDDOnRearBoundary:
      if (y > 0) {y = 0;} // don't permit more rearward motion
      CheckColorForAllClear(isWhite, isRed, isBlue);
      break;
  }
  // frc::SmartDashboard::PutNumber("Demo State", mDemoDriveState);

  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading + 180.0);  // offset by 180 to avoid discontinuity


  // clip rate to max rotation
  rotateRate = std::min(kDefaultRotateToTargetRate, rotateRate);
  rotateRate = std::max(-kDefaultRotateToTargetRate, rotateRate);

  MyDriveCartesian(-y*kDemoSpeedMultY, x*kDemoSpeedMultX, -rotateRate, currHeading);
}

void DriveSystem::Rotate180ForSummer() {
  double currHeading = mAHRS->GetAngle();
  double rotateRate = mPIDControllerGyro->Calculate(currHeading);  // setpoint is 180
  // clip rate to max rotation to keep ball from flying out
  rotateRate = std::min(kDefaultRotateToTargetRate, rotateRate);
  rotateRate = std::max(-kDefaultRotateToTargetRate, rotateRate);
  MyDriveCartesian(0, 0, -rotateRate, currHeading);
}

#endif
