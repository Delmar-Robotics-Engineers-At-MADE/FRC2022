  #include <AutonomousController.h>
  #include <frc/smartdashboard/SmartDashboard.h> 
  #include <iostream>
   
AutonomousController::AutonomousController (frc::MecanumDrive *drive) {
    
mDrive = drive;
}


void AutonomousController::AutonomousInit(){
    frc::SmartDashboard::PutBoolean("Auto complete", false);
    mAutoSelected = mChooser.GetSelected();
    mDriveStates = kBegin;
    mTimer.Reset();
    mTimer.Start();
    std::cout << "auto selected: " << mAutoSelected << std::endl;
}

void AutonomousController::AutonomousPeriodic(){
    if (mAutoSelected == kAutoNameDriveOnly) {
        Driving();
    } else {
        NotDriving();
    }
}

void AutonomousController::Driving () {
    switch (mDriveStates) {
        case kBegin:
             mDriveStates = kDriving;
            break;
        case kDriving:
              mDrive->DriveCartesian(0.3, 0.0, 0.0);
              if (mTimer.Get() > 1_s){
                mDriveStates = kCompleted;
              }
            break;
        case kCompleted:
                mDrive->DriveCartesian(0.0, 0.0, 0.0);
            break;
        default:
            break;
    }

}

void AutonomousController::NotDriving() {
    mDrive->DriveCartesian(0.0, 0.0, 0.0);
}