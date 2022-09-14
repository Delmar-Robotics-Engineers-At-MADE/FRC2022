#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <DriveSystem.h>
#include <iostream>

// #define SUMMER

static const double kMinTargetAreaPercent = 0.0;
static const double kRollerIdleSpeed = 0.0;  // was 2300 for 2022 competition
static const double kVelocityTolerance = 500;
constexpr units::time::second_t kBlindShotReadyTime = 2.0_s; // seconds

static const double kFtuned = 0.0451;
static const double kPtuned = 0.15;  // was 0.1
static const double kDtuned = 0.002;
static const double kRampTuned = 2.5;

static const int kButtonShooterBlind = 3;
static const int kButtonShooterShort = 2;
static const int kButtonShooterLong = 4;

// static const double kCoeff2022[3] = {-173.0/18.0, 11845.0/18.0, 5057}; // used at regional

double ConvertRadsToDegrees (double rads) {
    const static double conversion_factor = 180.0/3.141592653589793238463;
    return rads * conversion_factor;
}

double ConvertDegreesToRads (double degs) {
    const static double conversion_factor = 3.141592653589793238463/180.0;
    return degs * conversion_factor;
}

// h1 = height of limelight; h2 = height of target above limelight; 
// phi = angle of limelight from vertical; theta = vertical angle of target reported by limelight
// distance to target = h2 / TAN (theta + phi)


Shooter::Shooter () {  // constructor
  // get network table populated by LimeLight
  mLimeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Shooter::TurnLightOnOrOff (bool turnOn) {
  bool turnOff = !turnOn;
  bool lightIsOff = !mLightOn;
  if (mLightOn && turnOff) {
    std::cout << "sending command to turn OFF light " << std::endl;
    mLimeTable->PutNumber("ledMode",1.0); // LED off
    mLightOn = false;
  } else if (lightIsOff && turnOn) {
    std::cout << "sending command to turn ON light " << std::endl;
    mLimeTable->PutNumber("ledMode",3.0); // LED on bright
    mLightOn = true;
  }
  frc::SmartDashboard::PutBoolean("Light is On", mLightOn);
  // if (turnOn) {
  //   std::cout << "sending command to turn ON light " << std::endl;
  //   mLimeTable->PutNumber("ledMode",3.0); // LED on bright
  // } else { // turn off
  //   std::cout << "sending command to turn OFF light " << std::endl;
  //   mLimeTable->PutNumber("ledMode",1.0); // LED off
  // }
}

void Shooter::CheckLimelight() {
  double tv = mLimeTable->GetNumber("tv",0.0); 
  mTargetSeen = (tv != 0.0);

  // do NOT set these to zero if target not seen, because target flickers in and out;
  // let the numbers reflect the last time it was seen
  // mTargetAngleHorizontal = 0.0;
  // mTargetAngleVertical = 0.0;
  // mTargetDistance = 0.0;

  if (mTargetSeen) {
    mTargetArea = mLimeTable->GetNumber("ta",0.0);  
    if (mTargetArea > kMinTargetAreaPercent) {  
      mTargetAngleHorizontal = mLimeTable->GetNumber("tx",0.0);
      mTargetAngleVertical = mLimeTable->GetNumber("ty",0.0);   
      mTargetDistance = mH2 / tan(ConvertDegreesToRads(mTargetAngleVertical + mPhi));
    }
  }
  
  frc::SmartDashboard::PutBoolean("Target Seen", mTargetSeen);
  frc::SmartDashboard::PutNumber("Target Area", mTargetArea);
  frc::SmartDashboard::PutNumber("Angle Horiz", mTargetAngleHorizontal);
  frc::SmartDashboard::PutNumber("Angle Vert", mTargetAngleVertical);
  frc::SmartDashboard::PutNumber("Distance", mTargetDistance);
}

double Shooter::CalcHighTargetSpeed(double d){

  // used for 2022 regional: 
  // double result = (-173.0/18.0) * d * d + (11845.0/18.0) * d + 5057;
  // std::cout << "speed target: " << result << std::endl;
  
  // for summer 2022: 11.5601 x^3 - 570.057 x^2 + 9517.92 x - 41011.8
  double result = (11.5601 * d * d * d) - (570.057 * d * d) + (9517.92 * d) - 41011.8;

  // allow drivers to boost or deboost with multiplier
  result *= mSpeedMultiplier;

  return result;
}

bool FalconSpeedInRange(double speed) {
  bool result = true;
  if (speed < 10000 || speed > 25000) { 
    // std::cout << "Falcon speed out of range: " << speed << std::endl;
    result = false;
  }
  return result;
}

bool Shooter::ReadyShooter(ElevationOption option) {
  bool result = false;
  switch (option) {
    case kEOLongRange:
    case kEOShortRange:
      if (mTargetSeen) {
        bool elevationReady = mElevator.Elevate(option);
        double speedTarget = CalcHighTargetSpeed(mTargetDistance);
        double actualSpeed = mPortShooter.GetSelectedSensorVelocity();
        double pidError = 0.0;
        if (FalconSpeedInRange(speedTarget)) {
          mPortShooter.Set(ControlMode::Velocity, speedTarget);
          frc::SmartDashboard::PutNumber("Shooter V target", speedTarget);
          pidError = mPortShooter.GetClosedLoopError();
          result = elevationReady 
                && abs(pidError) < kVelocityTolerance
                && FalconSpeedInRange(actualSpeed);
        }
        // frc::SmartDashboard::PutNumber("Shooter Actual", actualSpeed);
        // frc::SmartDashboard::PutNumber("Shooter Error", pidError);
      }
      break;
    case kEOManual:
      default:
      // for now permit dashboard widget to control speed
      mMotorOutVelocity = frc::SmartDashboard::GetNumber("Shooter Speed Low Target", 0);
      mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);
      double actualVelocity = mPortShooter.GetSelectedSensorVelocity();
      if (abs(actualVelocity - mMotorOutVelocity) < kVelocityTolerance) {
        result = true; // up to speed, so ok to feed;
      }
  } // switch
  return result;
}

void Shooter::ShootForAuto() {
  // mFeeder.Set(kFeederSpeed);  // change to FeedCargo() eventually
  mFeeder->FeedCargo();
}

bool Shooter::FixedElevationForAuto() {
  bool result = mElevator.FixedElevationForAuto();
  return result;
}

void Shooter::Shoot (ElevationOption option, DriveSysTargetingState driveState) {
  bool onTarget = false;
  bool shooterReady = false;
  // frc::SmartDashboard::PutNumber("Shooter State", mState);
  // frc::SmartDashboard::PutNumber("Drive State", driveState);
  switch (mState) {
    default:
    case kIdle:
      mState = kRotatingToTarget;
      mFeeder->StopFeedingCargo();
      break; 
    case kRotatingToTarget:
      mFeeder->StopFeedingCargo();
      // drive system has access to state info, and will know to rotate
      onTarget = (driveState == kDriveOnTarget);
      shooterReady = ReadyShooter(option);
      frc::SmartDashboard::PutBoolean("On Target", onTarget);
      frc::SmartDashboard::PutBoolean("Shooter Ready", shooterReady);
      if (onTarget && shooterReady) {mState = kShooterReady;}
      break;
    case kShooterReady:
      if (mFeeder->CargoAvailable()) {
        mFeeder->FeedCargo();
      } else {
        mState = kEmpty;
      }
      break;
    case kEmpty:
      mFeeder->StopFeedingCargo();
      Idle();
      break;
  }

}

void Shooter::Idle(){
  mState = kIdle;
  mPortShooter.Set(ControlMode::Velocity, kRollerIdleSpeed);
}


void Shooter::TeleopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, 
                              DriveSysTargetingState driveState){
  // mPhi = frc::SmartDashboard::GetNumber("Phi", mPhi); // angle of limelight from vertical
  // mH2 = frc::SmartDashboard::GetNumber("H2", mH2); // height of target above limelight
  bool shootLongRange = copilot->GetRawButton(kButtonShooterLong);
  bool shootShortRange = copilot->GetRawButton(kButtonShooterShort);
  bool shootBlind = copilot->GetRawButton(kButtonShooterBlind);

#ifdef SUMMER
  // monitor intake's summer demo mode, and shoot if necessary
  if (mIntake->mFetchState == kFBSShooting) {
    shootAtLowGoal = true;
  }
#endif

  if (shootLongRange || shootShortRange) { 
    mSpeedMultiplier = frc::SmartDashboard::GetNumber("Shooter Boost", mSpeedMultiplier);
    TurnLightOnOrOff(true);
    CheckLimelight();
    ElevationOption option = shootLongRange ? kEOLongRange : kEOShortRange;
    Shoot(option, driveState);
    // std::cout << "shooter state" << mState << std::endl;
  } else if (shootBlind) {
    TurnLightOnOrOff(true);
    CheckLimelight();  // to populate numbers in dashboard; not used for blind shot
    // for now, do blind shot
    BlindShot(copilot);
  } else { // no shooter buttons pressed
    TurnLightOnOrOff(false);
    Idle();
    mBlindShotState = kBSSUnknownState;
    frc::SmartDashboard::PutBoolean("On Target", false);
    frc::SmartDashboard::PutBoolean("Shooter Ready", false);
    frc::SmartDashboard::PutBoolean("Elevator Ready", false);
    frc::SmartDashboard::PutBoolean("Feeding Cargo", false);
    // ManualFeed(pilot);  // alowed if not shooting
    mFeeder->StopFeedingCargo();
 }
  // }
  // mMotorOutVelocity = frc::SmartDashboard::GetNumber("motor output percentage", 0);
  // mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);

  // we own the elevator, so run it
  mElevator.TeleopPeriodic(copilot);
}

void Shooter::RobotInit(Feeder *feeder, Intake *intake) {
  mFeeder = feeder;
  mIntake = intake;

  // frc::SmartDashboard::PutNumber("Phi", mPhi);
  // frc::SmartDashboard::PutNumber("H2", mH2);
  frc::SmartDashboard::PutNumber("Auto Shoot V", mAutoShootSpeed);
  frc::SmartDashboard::PutNumber("Shooter Speed Low Target", mAutoShootSpeed);
  frc::SmartDashboard::PutNumber("Shooter Boost", mSpeedMultiplier);

  mStarShooter.ConfigFactoryDefault();
  mPortShooter.ConfigFactoryDefault();

  // one follower and one reversed
  mStarShooter.Follow(mPortShooter);
  mStarShooter.SetInverted(true);
  mPortShooter.SetInverted(false);

  mPortShooter.SetNeutralMode(NeutralMode::Coast);
  mStarShooter.SetNeutralMode(NeutralMode::Coast);

  // Port is the leader, so set its PID and sensor

  mPortShooter.Config_kF(0, kFtuned, 30); // was .045 in 2020
  mPortShooter.Config_kP(0, kPtuned, 30);
  mPortShooter.Config_kI(0, 0.0, 30); // was .00005 in 2020
  mPortShooter.Config_kD(0, kDtuned, 30); // was 0 in 2020
  mPortShooter.ConfigClosedloopRamp(kRampTuned);
  mPortShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);

   // Elevator
  mElevator.RobotInit();

  // Limelight
  std::cout << "calling TurnLightOff " <<  std::endl;
  TurnLightOnOrOff(false);
  frc::SmartDashboard::PutBoolean("Light is On", mLightOn);
  mLimeTable->PutNumber("camMode",0.0); // camera in normal CV mode
  mLimeTable->PutNumber("stream",0.0);  // secondary camera side-by-side

}

void Shooter::RepeatableInit() {
  Idle();
  mFeeder->StopFeedingCargo();
}

void Shooter::TeleopInit() {
  // mMotorOutVelocity = kRollerIdleSpeed;
  mLightOn = frc::SmartDashboard::GetBoolean("Light is On", mLightOn);
}

void Shooter::RobotPeriodic() {
  // for testing
  mElevator.RobotPeriodic();
}

void Shooter::DoOnceInit() {
  TurnLightOnOrOff(false); // mLimeTable->PutNumber("ledMode",3.0); // LED on bright
}

void Shooter::FixedSpeedForAuto(){
  mPortShooter.Set(ControlMode::Velocity, mAutoShootSpeed); // was kShooterSpeedForAuto
}

void Shooter::AutonomousInit() {
  // frc::SmartDashboard::PutNumber("Distance", mTargetDistance);
  mAutoShootSpeed = frc::SmartDashboard::GetNumber("Auto Shoot V", mAutoShootSpeed);
}

void Shooter::BlindShot(frc::Joystick *copilot) {
  bool shootButtonPressed = copilot->GetRawButton(kButtonShooterBlind);
#ifdef SUMMER
  // monitor intake's summer demo mode, and shoot if necessary
  if (mIntake->mFetchState == kFBSShooting) {
    shootButtonPressed = true;
  }
#endif

  switch (mBlindShotState) {
    default:
    case kBSSBegin:
      mBlindShotTimer.Reset();
      mBlindShotTimer.Start();
      mBlindShotState = kBSSReadying;
      break;
    case kBSSReadying:
      // FixedSpeedForAuto();
      // FixedElevationForAuto();
      // leave elevator in place, and set shooter speed according to dashboard
      ReadyShooter(kEOManual);  // don't wait for this to be true
      if (mBlindShotTimer.Get() > kBlindShotReadyTime) {
        mBlindShotState = kBSSShooting;
      }
      break;
    case kBSSShooting:
      if (shootButtonPressed) {
        ShootForAuto();  // stay on this state as long as button is pressed
      } else {
        mBlindShotState = kBSSCompleted;
      }
      break;
    case kBSSCompleted:
      Idle();
      mFeeder->StopFeedingCargo();
      break;    
  }
}
