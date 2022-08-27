#include <Intake.h>
#include <Constants.h>
#include <RaspPi.h>
#include <frc/smartdashboard/SmartDashboard.h>

// #define SUMMER

static const double kIntakeSpeed = -14000; // before encoder was 0.6;

const static double kPtuned = .0001;
const static double kItuned = 0.0;
const static double kDtuned = 0.0;
const static double kFtuned = .00004;
const static double kPIDTolerance = 500;

void Intake::TeleopInit() {
#ifdef SUMMER
  frc::SmartDashboard::PutBoolean("Ball Demo", mEnableSummerDemo);
  frc::SmartDashboard::PutBoolean("Also Shoot", mEnableSummerDemoShoot);
  frc::SmartDashboard::PutBoolean("Also Rotate", mEnableSummerDemoRotateShoot);
#endif  
  
}


void Intake::TeleopPeriodic (frc::Joystick *pilot, bool ballAtFeeder, RaspPi *rPi) {

#ifdef SUMMER
  mEnableSummerDemo = frc::SmartDashboard::GetBoolean("Ball Demo", mEnableSummerDemo);
  mEnableSummerDemoShoot = frc::SmartDashboard::GetBoolean("Also Shoot", mEnableSummerDemoShoot);
  mEnableSummerDemoRotateShoot = frc::SmartDashboard::GetBoolean("Also Rotate", mEnableSummerDemoRotateShoot);
  if (mEnableSummerDemo) {
    rPi->CheckForBall();
    frc::SmartDashboard::PutBoolean("Ball Ahead", rPi->mBallAhead);
    frc::SmartDashboard::PutNumber("Fetch State", mFetchState);
    FetchBall(ballAtFeeder, rPi);
  } else {
    Retract();
  }

#else
  bool deploy = pilot->GetRawButton(8) || pilot->GetRawButton(7);
  if (deploy) {
    // mSolenoid.Set(frc::DoubleSolenoid::kReverse);
    // mRoller.Set(kIntakeSpeed);
    Deploy();
  } else {
    // mRoller.Set(0.0);
    // mSolenoid.Set(frc::DoubleSolenoid::kForward);
    Retract();
  }
#endif

}

void Intake::RobotInit() {
		mRoller.ConfigFactoryDefault();
		mRoller.ConfigNominalOutputForward(0, kTimeoutMs);
		mRoller.ConfigNominalOutputReverse(0, kTimeoutMs);
    mRoller.SetInverted(true);

    mPIDController = new frc2::PIDController (kPtuned, kItuned, kDtuned);
    mPIDController->SetTolerance(kPIDTolerance);
    mPIDController->SetSetpoint(kIntakeSpeed);
    frc::SmartDashboard::PutData("Intake PID", mPIDController);

#ifdef SUMMER
  frc::SmartDashboard::PutBoolean("Ball Demo", mEnableSummerDemo);
  frc::SmartDashboard::PutBoolean("Also Shoot", mEnableSummerDemoShoot);
  frc::SmartDashboard::PutBoolean("Also Rotate", mEnableSummerDemoRotateShoot);
#endif  

}

void Intake::DoOnceInit() {
    mRoller.Set(0.0);
    mSolenoid.Set(frc::DoubleSolenoid::kForward);  // retract intake
}

void Intake::AutonomousPeriodic () {

}

void Intake::Deploy() {
  mSolenoid.Set(frc::DoubleSolenoid::kReverse);

  // mRoller.Set(kIntakeSpeed);
  double encoderSpeed = mEncoder.GetRate() ; // invert encoder, as in ManualElevate
  double power = kFtuned * kIntakeSpeed;  // basically feed forward
  power += mPIDController->Calculate(encoderSpeed); 
  mRoller.Set(power); 
  // frc::SmartDashboard::PutNumber("Intake Actual", encoderSpeed);
  // frc::SmartDashboard::PutNumber("Intake Power", power);
  // frc::SmartDashboard::PutNumber("Intake Error1", mPIDController->GetPositionError());
  // frc::SmartDashboard::PutNumber("Intake Error2", mPIDController->GetVelocityError());
  frc::SmartDashboard::PutBoolean("Intake Maxed", abs(power) >= 1.0);
}

void Intake::Retract() {
  mRoller.Set(0.0);
  mSolenoid.Set(frc::DoubleSolenoid::kForward);
}

void Intake::FetchBall (bool ballAtFeeder, RaspPi *rPi) {
  switch (mFetchState) {
    default:
    case kFBSWaitingForBall:
      Retract();
      // check ML for ball close enough
      if (rPi->mBallAhead) {
        mFetchState = kFBSBallAhead;
      } else if (ballAtFeeder) {
        mFetchState = kFBSBallAtFeeder;
        mTimer.Reset(); mTimer.Start(); // keep ball in feeder for period of time
      }
      break;
    case kFBSBallAhead:
      // lower intake and turn it on
      Deploy();
      // stay in this state until ball at feeder or ball no longer seen
      if (ballAtFeeder) {
        mFetchState = kFBSBallAtFeeder;
        mTimer.Reset();  mTimer.Start();// keep ball in feeder for period of time
      } else if (!rPi->mBallAhead) {
        mFetchState = kFBSBallGone;
        mTimer.Reset(); mTimer.Start(); // use timer to smooth out ball seen/not seen flickering
      }
      break;
    case kFBSBallAtFeeder:
      Retract();
      // for summer, stay in this state for 2 secs, then spit ball back out or shoot
      if (mTimer.Get() > 2.0_s) {
        if (mEnableSummerDemoShoot) { 
          if (mEnableSummerDemoRotateShoot) {mFetchState = kFBSRotating;}
          else {mFetchState = kFBSShooting;} 
        } 
        else { mFetchState = kFBSBallReturning; }
        mTimer.Reset(); mTimer.Start(); // return for a period of time
      }
      break;
    case kFBSBallReturning:
      // for summer, stay in this state for 2 secs
      if (mTimer.Get() > 2.0_s) {
        mFetchState = kFBSWaitingForBall;
      }
      break;
    case kFBSShooting:
      // for summer, stay in this state for longer
      if (mTimer.Get() > 4.0_s) {
        if (mEnableSummerDemoRotateShoot) {
          mFetchState = kFBSRotatingBack;
          mTimer.Reset(); mTimer.Start();
        } else {mFetchState = kFBSWaitingForBall;}
      }
      break;
    case kFBSRotating:
      // for summer, wait for robot to rotate 180, then shoot
      if (!ballAtFeeder) {
        // lost ball, so abort shooting
        mFetchState = kFBSRotatingBack;
        mTimer.Reset(); mTimer.Start();
      } else if (mTimer.Get() > 4.0_s) {
        mFetchState = kFBSShooting;
        mTimer.Reset(); mTimer.Start();
      }
      break;
    case kFBSRotatingBack:
      // for summer, wait for robot to rotate 180 back
      if (mTimer.Get() > 4.0_s) {
        mFetchState = kFBSWaitingForBall;
      }
      break;
    case kFBSBallGone:
      // same as ball ahead for 5 secs
      Deploy();
      if (ballAtFeeder) {
        mFetchState = kFBSBallAtFeeder;
        mTimer.Reset(); // keep ball in feeder for period of time
        mTimer.Start();
      } else if (mTimer.Get() > 5.0_s) { // if we haven't seen a ball for 5 secs
        mFetchState = kFBSWaitingForBall;
      }
      break;
  }
}

// bool Intake::DemoReturningBall() {
//   bool result = (mFetchState == kFBSBallReturning);
//   return result;
// }
