#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <DriveSystem.h>
#include <iostream>

static const double kMinTargetAreaPercent = 0.0;
static const double kRollerIdleSpeed = 1000.0;
static const double kFeederSpeed = 100.0;

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
  if (mLightOn && turnOff) {
    mLimeTable->PutNumber("ledMode",1.0); // LED off
  } else if (!mLightOn && turnOn) {
    mLimeTable->PutNumber("ledMode",3.0); // LED on bright
  }
}

void Shooter::CheckLimelight() {
  double tv = mLimeTable->GetNumber("tv",0.0); 
  mTargetSeen = (tv != 0.0);
  mTargetArea = mLimeTable->GetNumber("ta",0.0);  
  mTargetAngleHorizontal = 0.0;
  mTargetAngleVertical = 0.0;
  mTargetDistance = 0.0;

  if (mTargetSeen) {
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

bool Shooter::ReadyShooter() {
  bool TODO_Set_Speed_Function_Of_Distance = false;
  bool elevationReady = mElevator.Elevate(mTargetDistance);
  return true;
}

void Shooter::FeedCargo() {
  mFeeder.Set(kFeederSpeed);
}

void Shooter::Shoot (bool highTarget, DriveSysTargetingState driveState) {
  bool onTarget = false;
  bool shooterReady = false;
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
      shooterReady = ReadyShooter();
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
  mFeeder.Set(0.0);
}

void Shooter::ManualFeed (frc::Joystick *copilot) {
  if (copilot->GetRawButton(3) && copilot->GetRawButton(4)) {
    FeedCargo();
  } else if (copilot->GetRawButton(3) && copilot->GetRawButton(2)) {
    mFeeder.Set(kFeederSpeed);
  }
}

void Shooter::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot, DriveSysTargetingState driveState){
  mPhi = frc::SmartDashboard::GetNumber("Phi", mPhi); // angle of limelight from vertical
  mH2 = frc::SmartDashboard::GetNumber("H2", mH2); // height of target above limelight
  bool shootAtHighGoal = copilot->GetRawButton(4);
  bool shootAtLowGoal = copilot->GetRawButton(2);

  if (shootAtHighGoal || shootAtLowGoal) {
    TurnLightOnOrOff(true);
    CheckLimelight();
    Shoot(shootAtHighGoal, driveState);
    std::cout << "shooter state" << mState << std::endl;
  } else { // no shooter buttons pressed
    TurnLightOnOrOff(false);
    Idle();
    frc::SmartDashboard::PutBoolean("On Target", false);
    frc::SmartDashboard::PutBoolean("Shooter Ready", false);
    frc::SmartDashboard::PutBoolean("Feeding Cargo", false);
  }
  mMotorOutVelocity = frc::SmartDashboard::GetNumber("motor output percentage", 0);
  mPortShooter.Set(ControlMode::Velocity, mMotorOutVelocity);
  ManualFeed(copilot);  // alowed at any time

  // we own the elevator, so run it
  mElevator.TelopPeriodic(copilot);

}

void Shooter::RobotInit() {
  frc::SmartDashboard::PutNumber("Phi", mPhi);
  frc::SmartDashboard::PutNumber("H2", mH2);

  mLimeTable->PutNumber("camMode",0.0); // camera in normal CV mode
  mLimeTable->PutNumber("ledMode",1.0); // LED off
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

  mPortShooter.Config_kF(0, 0.045, 30);
  mPortShooter.Config_kP(0, 0.009, 30);
  mPortShooter.Config_kI(0, 0.00005, 30);
  mPortShooter.Config_kD(0, 0.0, 30);
  mPortShooter.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);

  // Other motors

  mFeeder.ConfigFactoryDefault();
  mFeeder.ConfigNominalOutputForward(0, kTimeoutMs);
  mFeeder.ConfigNominalOutputReverse(0, kTimeoutMs);

}

void Shooter::DoOnceInit() {
  Idle();
}

void Shooter::TeleopInit() {
  mMotorOutVelocity = 1000.0;
  frc::SmartDashboard::PutNumber("motor output percentage", mMotorOutVelocity);
}