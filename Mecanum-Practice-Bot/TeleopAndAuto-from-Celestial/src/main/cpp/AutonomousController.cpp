#include <AutonomousController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

AutonomousController::AutonomousController (/*VelocityController2 *vcontroller*/) { // constructor

  // mVelocityController = vcontroller;

  mChooser.SetDefaultOption(kAutoNameDriveTurnAround, kAutoNameDriveTurnAround);
  mChooser.AddOption(kAutoNameDriveTurnAround, kAutoNameDriveTurnAround);
  mChooser.AddOption(kAutoNameDriveOnly, kAutoNameDriveOnly);
  mChooser.AddOption(kAutoNameTurnOnly, kAutoNameTurnOnly);
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
  frc::SmartDashboard::PutBoolean("Auto Complete", false);
  mAutoSelected = mChooser.GetSelected();
  mAutoSelectedOptionsDistance = mChooserOptionsDistance.GetSelected();
  mAutoSelectedOptionsWait = mChooserOptionsWait.GetSelected();
  if (mAutoSelected == kAutoNameDriveTurnAround) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
      // mVelocityController->SetTrapezoidGoal(5.0_ft, 0_fps);
      // mVelocityController->StartMotionTimer();
  } else if (mAutoSelected == kAutoNameTurnOnly) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
  } else if (mAutoSelected == kAutoNameJustInit) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  } else {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  }
  mdtsAutoState = kdtsBegin; 
  mtosAutoState = ktosBegin; 
  std::cout << "auto init: " << mAutoSelected << std::endl;
}

void AutonomousController::DriveTurnAround() {
  bool turningDone = false; bool drivingDone = false;
  switch (mdtsAutoState) {
    case kdtsBegin:
      mdtsAutoState = kdtsDriving;
      break;
    case kdtsDriving:
      // drivingDone = mVelocityController->DriveTrapezoid();
      if (drivingDone) {mdtsAutoState = kdtsStraightening;}
      break;
    case kdtsStraightening:
      // turningDone = mVelocityController->TurnRight(0);
      if (turningDone) { mdtsAutoState = kdtsTurning; }
      break;
    case kdtsTurning:
      // turningDone = mVelocityController->TurnRight(180);
      if (turningDone) {
        frc::SmartDashboard::PutBoolean("Auto Complete", true);
        mdtsAutoState = kdtsCompleted;
        }
      break;
    case kdtsCompleted:
      // mVelocityController->StopDriving();
      break;    
    case kdtsUnknownState:
    default:
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
      break;
  }
}

void AutonomousController::TurnOnly() {
  bool turningDone = false;
  switch (mtosAutoState) {
    case ktosBegin:
      mtosAutoState = ktosTurning;
      break;
    case ktosTurning:
      // turningDone = mVelocityController->TurnRight(90);
      if (turningDone) {
        frc::SmartDashboard::PutBoolean("Auto Complete", true);
        mtosAutoState = ktosCompleted;
        }
      break;
    case ktosCompleted:
      // mVelocityController->StopDriving();
      break;
    case ktosUnknownState:
    default:
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
      break;
  }
}

void AutonomousController::AutonomousPeriodic() {
  if (mAutoSelected == kAutoNameDriveTurnAround) {
    DriveTurnAround();
  } else if (mAutoSelected == kAutoNameTurnOnly) {
    TurnOnly();
  } else {
    // mVelocityController->StopDriving();
  }
}