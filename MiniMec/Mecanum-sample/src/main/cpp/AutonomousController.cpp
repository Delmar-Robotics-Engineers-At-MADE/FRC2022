#include <AutonomousController.h>
#include <frc/smartdashboard/SmartDashboard.h>

AutonomousController::AutonomousController (VelocityController2 *vcontroller) { // constructor

  mVelocityController = vcontroller;

  mChooser.SetDefaultOption(kAutoNameDriveTurnAround, kAutoNameDriveTurnAround);
  mChooser.AddOption(kAutoNameDriveTurnAround, kAutoNameDriveTurnAround);
  mChooser.AddOption(kAutoNameDriveOnly, kAutoNameDriveOnly);
  mChooser.AddOption(kAutoNameJustInit, kAutoNameJustInit);

  mChooserOptionsDistance.SetDefaultOption(kAutoOption2Feet,kAutoOption2Feet);
  mChooserOptionsDistance.AddOption(kAutoOption2Feet, kAutoOption2Feet);
  mChooserOptionsDistance.AddOption(kAutoOption4Feet, kAutoOption4Feet);

  mChooserOptionsWait.SetDefaultOption(kAutoOptionNoWait,kAutoOptionNoWait);
  mChooserOptionsWait.AddOption(kAutoOptionWait, kAutoOptionWait);
  mChooserOptionsWait.AddOption(kAutoOptionNoWait, kAutoOptionNoWait);

  frc::SmartDashboard::PutData("A Modes", &mChooser);
  frc::SmartDashboard::PutData("A Move?", &mChooserOptionsDistance);
  frc::SmartDashboard::PutData("A Wait?", &mChooserOptionsWait);

}

void AutonomousController::AutonomousInit() {
  mAutoSelected = mChooser.GetSelected();
  mAutoSelectedOptionsDistance = mChooserOptionsDistance.GetSelected();
  mAutoSelectedOptionsWait = mChooserOptionsWait.GetSelected();
  if (mAutoSelected == kAutoNameDriveTurnAround) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
      mVelocityController->SetTrapezoidGoal(5.0_ft, 0_fps);
      mVelocityController->StartMotionTimer();
  } else if (mAutoSelected == kAutoNameJustInit) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  } else {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  }
}

void AutonomousController::AutonomousPeriodic() {
  if (mAutoSelected == kAutoNameDriveTurnAround) {
      mVelocityController->DriveTrapezoid();
  }
}