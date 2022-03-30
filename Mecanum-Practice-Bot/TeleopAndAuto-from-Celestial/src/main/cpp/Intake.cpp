#include <Intake.h>
#include <Constants.h>

static const double kIntakeSpeed = 0.9;

void Intake::TeleopPeriodic (frc::Joystick *pilot) {
  bool deploy = pilot->GetRawButton(8) || pilot->GetRawButton(7);
  if (deploy) {
    mSolenoid.Set(frc::DoubleSolenoid::kReverse);
    mRoller.Set(kIntakeSpeed);
  } else {
    mRoller.Set(0.0);
    mSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
}

void Intake::RobotInit() {
  
		mRoller.ConfigFactoryDefault();
		mRoller.ConfigNominalOutputForward(0, kTimeoutMs);
		mRoller.ConfigNominalOutputReverse(0, kTimeoutMs);
}

void Intake::DoOnceInit() {
    mRoller.Set(0.0);
    mSolenoid.Set(frc::DoubleSolenoid::kForward);
}