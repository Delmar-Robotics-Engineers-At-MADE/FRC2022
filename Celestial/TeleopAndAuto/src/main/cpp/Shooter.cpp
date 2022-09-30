#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <DriveSystem.h>
#include <iostream>

// #define SUMMER
#define FALL

static const double kMinTargetAreaPercent = 0.0;
static const double kRollerIdleSpeed = 0.0;  // was 2300 for 2022 competition
static const double kVelocityTolerance = 500;
constexpr units::time::second_t kBlindShotReadyTime = 2.0_s; // seconds

static const double kFtuned = 0.0451;
static const double kPtuned = 0.15;  // was 0.1
static const double kDtuned = 0.002;
static const double kRampTuned = 2.5;

static const double kShortRangeCutoff = 13.0;
static const double kMidRangeCutoff = 17.0;

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

double CalcHighTargetSpeedShortRange(double d){
  // before limelight moved: double result = (28.6096 * d * d) - (215.259 * d) + 10589.2;
  // after: 19765.4 - 2619.06 x + 189.107 x^2
  //   https://www.wolframalpha.com/input?i=quadratic+fit+%7B6.1%2C10857.672%7D%2C%7B8.5%2C11367.825%7D%2C%7B7.8%2C10648%7D%2C%7B10%2C12446%7D%2C
  double result = 19765.4 - 2619.06 * d + 189.107 * d * d;
  return result;
}

double CalcHighTargetSpeedMidRange(double d){

  // quadratic before limelight moved
  // double result = (-3.78142 * d * d) + (405.464 * d) + 7292.46;
  // cubic
  // double result = (-14.0222 * d * d * d) + (716.558 * d * d) - (11712.6 * d) + 74009.4;
  // after: 8966.86 + 199.429 x + 5.43823 x^2
  //   https://www.wolframalpha.com/input?i=quadratic+fit+%7B17.6%2C14660.667%7D%2C%7B12.8%2C12712.791%7D%2C%7B14%2C12573.6555%7D%2C%7B15.7%2C12991.062%7D%2C%7B18.1%2C14435.652%7D%2C%7B21.3%2C15553.142%7D%2C%7B15%2C13130.035%7D%2C
  double result = 8966.86 + 199.429 * d + 5.43823 * d * d;
  return result;
}

double CalcHighTargetSpeedLongRange(double d){

  // quadratic
  // double result = (9.97246 * d * d) - (315.143 * d) + 16516.9;
  // cubic before limelight moved
  // double result = (9.65017 * d * d * d) - (611.192 * d * d) + (12903 * d) - 76437.9;
  // after:  18780.6 - 403.236 x + 12.2417 x^2
  //   https://www.wolframalpha.com/input?i=quadratic+fit+%7B25.1%2C16427.708%7D%2C%7B24.2%2C16184.773%7D%2C%7B22.1%2C15747.49%7D%2C%7B19.1%2C15747.49%7D%2C%7B17.7%2C15358.794%7D%2C%7B21.3%2C15795%7D%2C%7B22.5%2C15824%7D
  double result = 18780.6 - 403.236 * d + 12.2417 * d * d;
  return result;
}

double Shooter::CalcHighTargetSpeed(TargetRange shortMidLong, double d){

  // used for 2022 regional: 
  // double result = (-173.0/18.0) * d * d + (11845.0/18.0) * d + 5057;
  // std::cout << "speed target: " << result << std::endl;
  // for summer 2022: 11.5601 x^3 - 570.057 x^2 + 9517.92 x - 41011.8
  // double result = (11.5601 * d * d * d) - (570.057 * d * d) + (9517.92 * d) - 41011.8;
  
  double result = 0.0;
  switch (shortMidLong) {
  case kTRShort:
    result = CalcHighTargetSpeedShortRange(d);
    break;
  case kTRMid:
    result = CalcHighTargetSpeedMidRange(d);
    break;
  case kTRLong:
    result = CalcHighTargetSpeedLongRange(d);
    break;
  }

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

TargetRange CalcShortMidLongRange(ElevationButtonOption option, double d){
  TargetRange result = kTRShort;
  switch (option) {
  case kEBODangerClose:
    result = kTRShort;
    break;
  default:
  case kEBOLongOrMidRange:
    if (d < kShortRangeCutoff) {
      result = kTRShort;
    } else if (d < kMidRangeCutoff) {
      result = kTRMid;
    } else { 
      result = kTRLong;
    }
    break;
  }
  return result;
}

bool Shooter::ReadyShooter(ElevationButtonOption option) {
  bool result = false;
  TargetRange shortMidLong = CalcShortMidLongRange(option, mTargetDistance);
  switch (option) {
    case kEBOLongOrMidRange:
    case kEBODangerClose:
      if (mTargetSeen || option == kEBODangerClose) { // ok to shoot without target up close
        bool elevationReady = mElevator.Elevate(shortMidLong, mTargetDistance);
        double speedTarget = 0.0;
        if (option == kEBODangerClose) {speedTarget = kShooterSpeedForDangerClose;}
        else {speedTarget = CalcHighTargetSpeed(shortMidLong, mTargetDistance);}
        double actualSpeed = mPortShooter.GetSelectedSensorVelocity();
        double pidError = 0.0;
        frc::SmartDashboard::PutNumber("Shooter V target", speedTarget);
        frc::SmartDashboard::PutNumber("Target Range", shortMidLong);
        if (FalconSpeedInRange(speedTarget)) {
          mPortShooter.Set(ControlMode::Velocity, speedTarget);
          pidError = mPortShooter.GetClosedLoopError();
          result = elevationReady 
                && abs(pidError) < kVelocityTolerance
                && FalconSpeedInRange(actualSpeed);
        }
        // frc::SmartDashboard::PutNumber("Shooter Actual", actualSpeed);
        // frc::SmartDashboard::PutNumber("Shooter Error", pidError);
      }
      break;
    case kEBOManual:
    default:
      // for now permit dashboard widget to control speed
      frc::SmartDashboard::PutNumber("Target Range", -1); // range is N/A
      mMotorOutVelocity = frc::SmartDashboard::GetNumber("Shooter Speed Blind Shot", 0);
      // std::cout << "shooter speed" << mMotorOutVelocity << std::endl;
      mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);
      double actualVelocity = mPortShooter.GetSelectedSensorVelocity();
      if (abs(actualVelocity - mMotorOutVelocity) < kVelocityTolerance) {
        result = true; // up to speed, so ok to feed;
      }
  } // switch
  return result;
}

void Shooter::ShootForAuto() {
  mFeeder->FeedCargo();
}

bool Shooter::FixedElevationForAuto() {
  bool result = mElevator.FixedElevationForAuto();
  return result;
}

void Shooter::Shoot (ElevationButtonOption option, DriveSysTargetingState driveState) {
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
      onTarget = ((driveState == kDriveOnTarget) || (option == kEBODangerClose)); // can't see target so close; shoot anyway
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

void Shooter::ExpelBall (){
  mFeeder->ReverseFeed();
  mPortShooter.Set(ControlMode::Velocity, kExpelBallSpeed);
}


void Shooter::TeleopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, 
                              DriveSysTargetingState driveState){
  mPhi = frc::SmartDashboard::GetNumber("Phi", mPhi); // angle of limelight from vertical
  // mH2 = frc::SmartDashboard::GetNumber("H2", mH2); // height of target above limelight
  bool shootLongRange = copilot->GetRawButton(kButtonShooterLong);
  bool shootBlind = copilot->GetRawButton(kButtonShooterBlind);
  bool shootShortRange = copilot->GetRawButton(kButtonShooterShort);
  bool expelBall = (copilot->GetPOV() == 0);

#if defined(SUMMER)
  // monitor intake's summer demo mode, and shoot if necessary
  if (mIntake->mFetchState == kFBSShooting) {
    shootAtLowGoal = true;
  }
#elif defined(FALL)  
  // monitor intake's fall demo mode, and shoot if necessary
  if (mIntake->mFetchState == kFBSShooting) {
    shootBlind = true;
  }
#endif

  if (shootLongRange || shootShortRange) { 
    mSpeedMultiplier = frc::SmartDashboard::GetNumber("Shooter Boost", mSpeedMultiplier);
    TurnLightOnOrOff(true);
    CheckLimelight();
    ElevationButtonOption option = shootLongRange ? kEBOLongOrMidRange : kEBODangerClose;
    Shoot(option, driveState);
    // std::cout << "shooter state" << mState << std::endl;
  } else if (shootBlind) {
    TurnLightOnOrOff(true);
    CheckLimelight();  // to populate numbers in dashboard; not used for blind shot
    // for now, do blind shot
    BlindShot(copilot);
  } else if (expelBall) {
    ExpelBall();
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

  frc::SmartDashboard::PutNumber("Phi", mPhi);
  // frc::SmartDashboard::PutNumber("H2", mH2);
  frc::SmartDashboard::PutNumber("Auto Shoot V", mAutoShootSpeed);
  frc::SmartDashboard::PutNumber("Shooter Speed Blind Shot", kShooterSpeedForBlindShot);
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
#if defined(SUMMER)
  // monitor intake's summer demo mode, and shoot if necessary
  if (mIntake->mFetchState == kFBSShooting) {
    shootButtonPressed = true;
  }
#elif defined (FALL)
  // monitor intake's fall demo mode, and shoot if necessary
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
      ReadyShooter(kEBOManual);  // don't wait for this to be true
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
  // frc::SmartDashboard::PutNumber("Shooter State", mBlindShotState);

}
