#include <AutonomousController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

constexpr units::time::second_t kAutoDriveTime = 2.0_s; // seconds
constexpr units::time::second_t kAutoShootTime = 8.0_s; // seconds

AutonomousController::AutonomousController (DriveSystem *drive, Shooter *shooter) { // constructor

  mDrive = drive;
  mShooter = shooter;

  mChooser.SetDefaultOption(kAutoNameDriveOnly, kAutoNameDriveOnly);
  mChooser.AddOption(kAutoNameDriveOnly, kAutoNameDriveOnly);
  mChooser.AddOption(kAutoNameDriveAndShoot, kAutoNameDriveAndShoot);
  mChooser.AddOption(kAutoNameJustInit, kAutoNameJustInit);

  mChooserOptionsDirection.SetDefaultOption(kAutoOptionDirStraight,kAutoOptionDirStraight);
  mChooserOptionsDirection.AddOption(kAutoOptionDirStraight, kAutoOptionDirStraight);
  mChooserOptionsDirection.AddOption(kAutoOptionDirLeft, kAutoOptionDirLeft);
  mChooserOptionsDirection.AddOption(kAutoOptionDirRight, kAutoOptionDirRight);

  mChooserOptionsWait.SetDefaultOption(kAutoOptionNoWait,kAutoOptionNoWait);
  mChooserOptionsWait.AddOption(kAutoOptionWait, kAutoOptionWait);
  mChooserOptionsWait.AddOption(kAutoOptionNoWait, kAutoOptionNoWait);

  frc::SmartDashboard::PutData("A Modes", &mChooser);
  frc::SmartDashboard::PutData("A Direction?", &mChooserOptionsDirection);
  frc::SmartDashboard::PutData("A Wait?", &mChooserOptionsWait);

}

void AutonomousController::AutonomousInit() {
  frc::SmartDashboard::PutBoolean("Auto Complete", false);
  mAutoSelected = mChooser.GetSelected();
  mAutoSelectedOptionsDirection = mChooserOptionsDirection.GetSelected();
  mAutoSelectedOptionsWait = mChooserOptionsWait.GetSelected();
  if (mAutoSelected == kAutoNameDriveOnly) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
  } else if (mAutoSelected == kAutoNameDriveAndShoot) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
  } else if (mAutoSelected == kAutoNameJustInit) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  } else {
      frc::SmartDashboard::PutBoolean("Auto Does Something", false);
  }
  mDOSAutoState = kDOSBegin; 
  std::cout << "auto init: " << mAutoSelected << std::endl;

  mTimer.Reset();
  mTimer.Start();
}

void AutonomousController::DriveOnly() {
  bool drivingDone = false;
  switch (mDOSAutoState) {
    default:
    case kDOSBegin:
      mDOSAutoState = kDOSDriving;
      break;
    case kDOSDriving:
      if (mAutoSelectedOptionsDirection == kAutoOptionDirStraight) {
        mDrive->DriveSlowForAuto(0, -1);
      } else if (mAutoSelectedOptionsDirection == kAutoOptionDirLeft) {
        mDrive->DriveSlowForAuto(-0.7071, -0.7071); // drive at 45 degrees
      } else if (mAutoSelectedOptionsDirection == kAutoOptionDirRight) {
        mDrive->DriveSlowForAuto(0.7071, -0.7071); // drive at 45 degrees
      }
      if (mTimer.Get() > kAutoDriveTime) {
        mDOSAutoState = kDOSCompleted;
      }
      break;
    case kDOSCompleted:
      mDrive->StopMotor();
      break;    
  }
}

void AutonomousController::DriveAndShoot() {
  bool drivingDone = false;
  switch (mDASAutoState) {
    default:
    case kDASBegin:
      mDASAutoState = kDASDriving;
      break;
    case kDASDriving:
      if (mAutoSelectedOptionsDirection == kAutoOptionDirStraight) {
        mDrive->DriveSlowForAuto(0, -1);
      } else if (mAutoSelectedOptionsDirection == kAutoOptionDirLeft) {
        mDrive->DriveSlowForAuto(-0.7071, -0.7071); // drive at 45 degrees
      } else if (mAutoSelectedOptionsDirection == kAutoOptionDirRight) {
        mDrive->DriveSlowForAuto(0.7071, -0.7071); // drive at 45 degrees
      }
      if (mTimer.Get() > kAutoDriveTime) {
        mDASAutoState = kDASShooting;
      }
      break;
    case kDASShooting:
      mDrive->StopMotor();
      mShooter->Shoot(true, kDriveOnTarget);
      if (mTimer.Get() > kAutoShootTime) {
        mDASAutoState = kDASCompleted;
      }
      break;
    case kDASCompleted:
      mDrive->StopMotor();
      mShooter->Idle();
      break;    
  }
}


// void AutonomousController::DriveTurnAround() {
//   bool turningDone = false; bool drivingDone = false;
//   switch (mdtsAutoState) {
//     case kdtsBegin:
//       mdtsAutoState = kdtsDriving;
//       break;
//     case kdtsDriving:
//       // drivingDone = mVelocityController->DriveTrapezoid();
//       if (drivingDone) {mdtsAutoState = kdtsStraightening;}
//       break;
//     case kdtsStraightening:
//       // turningDone = mVelocityController->TurnRight(0);
//       if (turningDone) { mdtsAutoState = kdtsTurning; }
//       break;
//     case kdtsTurning:
//       // turningDone = mVelocityController->TurnRight(180);
//       if (turningDone) {
//         frc::SmartDashboard::PutBoolean("Auto Complete", true);
//         mdtsAutoState = kdtsCompleted;
//         }
//       break;
//     case kdtsCompleted:
//       // mVelocityController->StopDriving();
//       break;    
//     case kdtsUnknownState:
//     default:
//       frc::SmartDashboard::PutBoolean("Auto Does Something", false);
//       break;
//   }
// }

// void AutonomousController::TurnOnly() {
//   bool turningDone = false;
//   switch (mtosAutoState) {
//     case ktosBegin:
//       mtosAutoState = ktosTurning;
//       break;
//     case ktosTurning:
//       // turningDone = mVelocityController->TurnRight(90);
//       if (turningDone) {
//         frc::SmartDashboard::PutBoolean("Auto Complete", true);
//         mtosAutoState = ktosCompleted;
//         }
//       break;
//     case ktosCompleted:
//       // mVelocityController->StopDriving();
//       break;
//     case ktosUnknownState:
//     default:
//       frc::SmartDashboard::PutBoolean("Auto Does Something", false);
//       break;
//   }
// }

void AutonomousController::AutonomousPeriodic() {
  if (mAutoSelected == kAutoNameDriveOnly) {
    DriveOnly();
  } else if (mAutoSelected == kAutoNameDriveAndShoot) {
    DriveAndShoot();
  } else {
    mDrive->StopMotor();
  }
}