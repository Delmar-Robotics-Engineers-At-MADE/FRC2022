#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <DriveSystem.h>
#include <iostream>

static const double kMinTargetAreaPercent = 0.0;
static const double kRollerIdleSpeed = 2300.0;
static const double kVelocityTolerance = 500;
static const double kFeederSpeed = 0.9;
constexpr units::time::second_t kBlindShotReadyTime = 2.0_s; // seconds

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

bool Shooter::CargoAvailable() {
  // TODO: check photo eyes; if both are false, we're empty
  return true;
}

double CalcHighTargetSpeed(double d){
  double result = (173.0/18.0) * d * d + (11845.0/18.0) * d + 5057;
  // std::cout << "speed target: " << result << std::endl;
  return result;
}

bool FalconSpeedInRange(double speed) {
  bool result = true;
  if (abs(speed) > 30000) {
    std::cout << "Falcon speed out of range: " << speed << std::endl;
    result = false;
  }
  return result;
}

bool Shooter::ReadyShooter(bool hightTarget) {
  bool result = false;
  if (hightTarget) {
    if (mTargetSeen) {
      double speed = CalcHighTargetSpeed(mTargetDistance);
      frc::SmartDashboard::PutNumber("Shooter V target", speed);
      if (FalconSpeedInRange(speed)) { mPortShooter.Set(ControlMode::Velocity, speed); }
      bool elevationReady = mElevator.Elevate(hightTarget, mTargetDistance);
      result = elevationReady && mPortShooter.GetClosedLoopError() < kVelocityTolerance;  // shooter is ready when elevation is achieved and shooter speed is achieved
      frc::SmartDashboard::PutBoolean("Elevator Ready", elevationReady);
    }
  } else { // low target
    // for now permit dashboard widget to control speed
    mMotorOutVelocity = frc::SmartDashboard::GetNumber("motor output percentage", 0);
    mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);
    double actualVelocity = mPortShooter.GetSelectedSensorVelocity();
    if (abs(actualVelocity - mMotorOutVelocity) < kVelocityTolerance) {
      result = true; // up to speed, so ok to feed;
    }
  }
  bool TODO_Speed_for_Low_Target = false;
  return result;
}

void Shooter::FeedCargo() {
  if (mManualFeeding) {
    // let driver control feeder
  } else {
    std::cout << "feeding from FeedCargo: " << kFeederSpeed << std::endl;
    mFeeder.Set(kFeederSpeed);
  }
}

void Shooter::ShootForAuto() {
  mFeeder.Set(kFeederSpeed);  // change to FeedCargo() eventually
}

bool Shooter::FixedElevationForAuto() {
  bool result = mElevator.FixedElevationForAuto();
  return result;
}

void Shooter::Shoot (bool highTarget, DriveSysTargetingState driveState) {
  bool onTarget = false;
  bool shooterReady = false;
  frc::SmartDashboard::PutNumber("Shooter State", mState);
  frc::SmartDashboard::PutNumber("Drive State", driveState);
  switch (mState) {
    default:
    case kIdle:
      mState = kRotatingToTarget;
      StopFeeder();
      break; 
    case kRotatingToTarget:
      StopFeeder();
      // drive system has access to state info, and will know to rotate
      onTarget = (driveState == kDriveOnTarget);
      shooterReady = ReadyShooter(highTarget);
      frc::SmartDashboard::PutBoolean("On Target", onTarget);
      frc::SmartDashboard::PutBoolean("Shooter Ready", shooterReady);
      if (onTarget && shooterReady) {mState = kShooterReady;}
      break;
    case kShooterReady:
      if (CargoAvailable()) {
        frc::SmartDashboard::PutBoolean("Feeding Cargo", true);
        FeedCargo();
      } else {
        mState = kEmpty;
      }
      break;
    case kEmpty:
      StopFeeder();
      Idle();
      break;
  }

}

void Shooter::Idle(){
  mState = kIdle;
  mPortShooter.Set(ControlMode::Velocity, kRollerIdleSpeed);
}

void Shooter::StopFeeder() {
  // std::cout << "stopping feeder" << std::endl;
  mFeeder.Set(0.0);
}

void Shooter::ManualFeed (frc::Joystick *pilot) {
  // if (pilot->GetRawButton(3)) {
  //   mManualFeeding = true;
  //   std::cout << "stopping feed from ManualFeed" << std::endl;
  //   StopFeeder();
  // // } else if (pilot->GetRawButton(2)) {
  // //   mManualFeeding = true;
  // //   mFeeder.Set(1.0);
  // // } else if (pilot->GetRawButton(1)) {
  // //   mManualFeeding = true;
  // //   mFeeder.Set(0.7);
  // } else if (pilot->GetRawButton(4)) {
  //   mManualFeeding = true;
  //   std::cout << "feeding from ManualFeed: " << 0.3 << std::endl;
  //   mFeeder.Set(0.3);
  // } else {
  //   mManualFeeding = false;
  // }
  // frc::SmartDashboard::PutBoolean("Manual Feed", mManualFeeding);
  // bool TODO_Finish_Manual_Feed = false;
}

void Shooter::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, DriveSysTargetingState driveState){
  mPhi = frc::SmartDashboard::GetNumber("Phi", mPhi); // angle of limelight from vertical
  mH2 = frc::SmartDashboard::GetNumber("H2", mH2); // height of target above limelight
  bool shootAtHighGoal = copilot->GetRawButton(4);
  bool shootAtLowGoal = copilot->GetRawButton(2);

  if (shootAtHighGoal) { //  || shootAtLowGoal
    TurnLightOnOrOff(true);
    CheckLimelight();
    Shoot(shootAtHighGoal, driveState);
    // std::cout << "shooter state" << mState << std::endl;
  } else if (shootAtLowGoal) {
    // for now, do blind shot
    BlindShot(copilot);
  } else { // no shooter buttons pressed
    TurnLightOnOrOff(false);
    Idle();
    frc::SmartDashboard::PutBoolean("On Target", false);
    frc::SmartDashboard::PutBoolean("Shooter Ready", false);
    frc::SmartDashboard::PutBoolean("Elevator Ready", false);
    frc::SmartDashboard::PutBoolean("Feeding Cargo", false);
    // ManualFeed(pilot);  // alowed if not shooting
    StopFeeder();
 }
  // }
  // mMotorOutVelocity = frc::SmartDashboard::GetNumber("motor output percentage", 0);
  // mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);

  // we own the elevator, so run it
  mElevator.TelopPeriodic(copilot);
}

void Shooter::RobotInit() {
  frc::SmartDashboard::PutNumber("Phi", mPhi);
  frc::SmartDashboard::PutNumber("H2", mH2);

  mLimeTable->PutNumber("camMode",0.0); // camera in normal CV mode
  TurnLightOnOrOff(false); // mLimeTable->PutNumber("ledMode",1.0); // LED off
  mLimeTable->PutNumber("stream",0.0);  // secondary camera side-by-side

  mStarShooter.ConfigFactoryDefault();
  mPortShooter.ConfigFactoryDefault();

  // one follower and one reversed
  mStarShooter.Follow(mPortShooter);
  mStarShooter.SetInverted(true);
  mPortShooter.SetInverted(false);

  mPortShooter.SetNeutralMode(NeutralMode::Coast);
  mStarShooter.SetNeutralMode(NeutralMode::Coast);

  // Port is the leader, so set its PID and sensor

  mPortShooter.Config_kF(0, 0.045, 30); // was .045 in 2020
  mPortShooter.Config_kP(0, 0.009, 30);
  mPortShooter.Config_kI(0, 0.000005, 30); // was .00005 in 2020
  mPortShooter.Config_kD(0, 0.0009, 30); // was 0 in 2020
  mPortShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);

  // Other motors

  mFeeder.ConfigFactoryDefault();
  mFeeder.SetInverted(true);
  mFeeder.ConfigNominalOutputForward(0, kTimeoutMs);
  mFeeder.ConfigNominalOutputReverse(0, kTimeoutMs);

  // Elevator
  mElevator.RobotInit();

  // Limelight
  std::cout << "calling TurnLightOff " <<  std::endl;
  TurnLightOnOrOff(false);

  frc::SmartDashboard::PutNumber("Auto Shoot V", mAutoShootSpeed);
}

void Shooter::RepeatableInit() {
  Idle();
  StopFeeder();
}

void Shooter::TeleopInit() {
  mMotorOutVelocity = kRollerIdleSpeed;
  frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutVelocity);
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
  bool dontCare = false;
  switch (mBlindShotState) {
    default:
    case kBSSBegin:
      mBlindShotTimer.Reset();
      mBlindShotTimer.Start();
      mBlindShotState = kBSSReadying;
      break;
    case kBSSReadying:
      FixedSpeedForAuto();
      FixedElevationForAuto();
      if (mBlindShotTimer.Get() > kBlindShotReadyTime) {
        mBlindShotState = kBSSShooting;
      }
      break;
    case kBSSShooting:
      if (copilot->GetRawButton(2)) {
        ShootForAuto();  // stay on this state as long as button is pressed
      } else {
        mBlindShotState = kBSSCompleted;
      }
      break;
    case kBSSCompleted:
      Idle();
      StopFeeder();
      break;    
  }
}
