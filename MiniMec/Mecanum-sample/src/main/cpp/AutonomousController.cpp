#include <AutonomousController.h>
#include <frc/smartdashboard/SmartDashboard.h>

AutonomousController::AutonomousController (VelocityController2 *vcontroller) { // constructor

  mVelocityController = vcontroller;

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
      mVelocityController->SetTrapezoidGoal(5.0_ft, 0_fps);
      mVelocityController->StartMotionTimer();
  } else if (mAutoSelected == kAutoNameTurnOnly) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
  } else if (mAutoSelected == kAutoNameJustInit) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  } else {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  }
}

void AutonomousController::AutonomousPeriodic() {
  bool turningDone = false; bool drivingDone = false;
  if (mAutoSelected == kAutoNameDriveTurnAround) {
    switch (mdtsAutoState) {
      case kdtsBegin:
        mdtsAutoState = kdtsDriving;
        break;
      case kdtsDriving:
        drivingDone = mVelocityController->DriveTrapezoid();
        if (drivingDone) {mdtsAutoState = kdtsTurning;}
        break;
      case kdtsTurning:
        turningDone = mVelocityController->TurnRight(180);
        if (turningDone) {
          frc::SmartDashboard::PutBoolean("Auto Complete", true);
          mdtsAutoState = kdtsCompleted;
          }
        break;
      case kdtsCompleted:
        mVelocityController->StopDriving();
        break;    
      case kdtsUnknownState:
      default:
        frc::SmartDashboard::PutBoolean("Auto Does Something", false);
        break;
    }
  } else if (mAutoSelected == kAutoNameTurnOnly) {
    switch (mtosAutoState) {
      case ktosBegin:
        mtosAutoState = ktosTurning;
        break;
      case ktosTurning:
        turningDone = mVelocityController->TurnRight(90);
        if (turningDone) {
          frc::SmartDashboard::PutBoolean("Auto Complete", true);
          mtosAutoState = ktosCompleted;
          }
        break;
      case ktosCompleted:
        mVelocityController->StopDriving();
        break;
      case ktosUnknownState:
      default:
        frc::SmartDashboard::PutBoolean("Auto Does Something", false);
        break;
    }
  }
}