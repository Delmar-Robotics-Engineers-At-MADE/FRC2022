#include <Feeder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

// #define SUMMER

static const double kFeederSpeed = 0.9;
static const double kReverseFeederSpeed = -0.4;

bool Feeder::CargoAvailable() {
  return !mEyeFeeder.Get(); // after mods to funnel, added reflective tape and inverted
}

void Feeder::FeedCargo() {
  if (mManualFeeding) {
    // let driver control feeder
  } else {
    // std::cout << "feeding from FeedCargo: " << kFeederSpeed << std::endl;
    frc::SmartDashboard::PutBoolean("Feeding Cargo", true);
    mFeeder.Set(kFeederSpeed);
  }
}

void Feeder::ReverseFeed() {
  mFeeder.Set(kReverseFeederSpeed);
}

void Feeder::StopFeedingCargo() {
  if (mManualFeeding) {
    // let driver control feeder
  } else {
    frc::SmartDashboard::PutBoolean("Feeding Cargo", false);
    mFeeder.Set(0.0);
  }
}

#ifdef SUMMER

void Feeder::DemoReturnBall(Intake *intake) {
  if (intake->mFetchState == kFBSBallReturning) {
    mFeeder.Set(-kFeederSpeed);
  } else if (intake->mFetchState == kFBSShooting) {
    // don't do anything; shooter will control us
  } else {
    mFeeder.Set(0.0);
  }
}

#endif

void Feeder::TeleopPeriodic (Intake *intake){
  frc::SmartDashboard::PutBoolean("Cargo Present", CargoAvailable());

#ifdef SUMMER
  // for summer, return balls picked up by auto-intake, or maybe even shoot
  if (intake->mEnableSummerDemo) {
    DemoReturnBall(intake);
  } else {
    // let shooter tell us what to do
  }
#endif
}

void Feeder::RobotInit() {
  mFeeder.ConfigFactoryDefault();
  mFeeder.SetInverted(true);
  mFeeder.ConfigNominalOutputForward(0, kTimeoutMs);
  mFeeder.ConfigNominalOutputReverse(0, kTimeoutMs);
}

void Feeder::ManualFeed (frc::Joystick *pilot) {
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