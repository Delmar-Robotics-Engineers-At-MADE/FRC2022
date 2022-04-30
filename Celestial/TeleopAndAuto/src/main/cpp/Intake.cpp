#include <Intake.h>
#include <Constants.h>
#include <RaspPi.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define SUMMER

static const double kIntakeSpeed = 0.8;

void Intake::TeleopInit() {
#ifdef SUMMER
  frc::SmartDashboard::PutBoolean("Ball Demo", mEnableSummerDemo);
#endif  
  
}


void Intake::TeleopPeriodic (frc::Joystick *pilot, bool ballAtFeeder, RaspPi *rPi) {

#ifdef SUMMER
  mEnableSummerDemo = frc::SmartDashboard::GetBoolean("Ball Demo", mEnableSummerDemo);
  if (mEnableSummerDemo) {
    rPi->CheckForBall();
    frc::SmartDashboard::PutBoolean("Ball Ahead", rPi->mBallAhead);
    frc::SmartDashboard::PutNumber("Fetch State", mFetchState);
    FetchBall(ballAtFeeder, rPi);
  }

#else
  bool deploy = pilot->GetRawButton(8) || pilot->GetRawButton(7);
  if (deploy) {
    mSolenoid.Set(frc::DoubleSolenoid::kReverse);
    mRoller.Set(kIntakeSpeed);
  } else {
    mRoller.Set(0.0);
    mSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
#endif

}

void Intake::RobotInit() {
		mRoller.ConfigFactoryDefault();
		mRoller.ConfigNominalOutputForward(0, kTimeoutMs);
		mRoller.ConfigNominalOutputReverse(0, kTimeoutMs);

#ifdef SUMMER
  frc::SmartDashboard::PutBoolean("Ball Demo", mEnableSummerDemo);
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
  mRoller.Set(kIntakeSpeed);
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
      }
      break;
    case kFBSBallAhead:
      // lower intake and turn it on
      Deploy();
      // stay in this state until ball at feeder or ball no longer seen
      if (ballAtFeeder) {
        mFetchState = kFBSBallAtFeeder;
        mTimer.Reset(); // keep ball in feeder for period of time
        mTimer.Start();
      } else if (!rPi->mBallAhead) {
        mFetchState = kFBSBallGone;
        mTimer.Reset();  // use timer to smooth out ball seen/not seen flickering
        mTimer.Start();
      }
      break;
    case kFBSBallAtFeeder:
      Retract();
      // for summer, stay in this state for 2 secs, then spit ball back out
      if (mTimer.Get() > 2.0_s) {
        mFetchState = kFBSBallReturning;
        mTimer.Reset(); // return for a period of time
        mTimer.Start();
      }
      break;
    case kFBSBallReturning:
      // for summer, stay in this state for 2 secs
      if (mTimer.Get() > 2.0_s) {
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

bool Intake::DemoReturningBall() {
  bool result = (mFetchState == kFBSBallReturning);
  return result;
}
