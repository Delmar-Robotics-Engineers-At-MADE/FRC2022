#include <Intake.h>
#include <Constants.h>

static const double kIntakeSpeed = 1.0;

void Intake::TeleopPeriodic (frc::Joystick *pilot) {
  bool retract = pilot->GetRawButton(7);
  bool deploy = pilot->GetRawButton(8);
  if (retract && deploy) {
    // don't do anything when they're both pressed (driver confusion)
  } else if (retract) {
    mRoller.Set(0.0);
    mSolenoid.Set(frc::DoubleSolenoid::kForward);
  } else if (deploy) {
    mSolenoid.Set(frc::DoubleSolenoid::kReverse);
    mRoller.Set(kIntakeSpeed);
  }
}

void Intake::RobotInit() {
  
		mRoller.ConfigFactoryDefault();
		mRoller.ConfigNominalOutputForward(0, kTimeoutMs);
		mRoller.ConfigNominalOutputReverse(0, kTimeoutMs);
}