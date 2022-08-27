#include <AutonomousController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
// #define SUMMER

constexpr units::time::second_t kAutoDriveTime = 2.0_s; // seconds
constexpr units::time::second_t kAutoShootTime = 5.0_s; // seconds
constexpr units::time::second_t kAutoShootTimeEnd = 8.0_s; // seconds

AutonomousController::AutonomousController (DriveSystem *drive, Shooter *shooter, Feeder *feeder) { // constructor

  mDrive = drive;
  mShooter = shooter;
  mFeeder = feeder;

  mChooser.SetDefaultOption(kAutoNameDemoDriving, kAutoNameDemoDriving);
  mChooser.AddOption(kAutoNameDemoDriving, kAutoNameDemoDriving);

#ifndef SUMMER
  mChooser.AddOption(kAutoNameDriveOnly, kAutoNameDriveOnly);
  mChooser.AddOption(kAutoNameDriveAndShoot, kAutoNameDriveAndShoot);
  mChooser.AddOption(kAutoNameJustInit, kAutoNameJustInit);
#endif

#ifndef SUMMER
  mChooserOptionsDirection.SetDefaultOption(kAutoOptionDirStraight,kAutoOptionDirStraight);
  mChooserOptionsDirection.AddOption(kAutoOptionDirStraight, kAutoOptionDirStraight);
  mChooserOptionsDirection.AddOption(kAutoOptionDirLeft, kAutoOptionDirLeft);
  mChooserOptionsDirection.AddOption(kAutoOptionDirRight, kAutoOptionDirRight);

  mChooserOptionsWait.SetDefaultOption(kAutoOptionNoWait,kAutoOptionNoWait);
  mChooserOptionsWait.AddOption(kAutoOptionWait, kAutoOptionWait);
  mChooserOptionsWait.AddOption(kAutoOptionNoWait, kAutoOptionNoWait);
#endif

  frc::SmartDashboard::PutData("A Modes", &mChooser);

#ifndef SUMMER
  frc::SmartDashboard::PutData("A Direction?", &mChooserOptionsDirection);
  frc::SmartDashboard::PutData("A Wait?", &mChooserOptionsWait);
#endif

}

void AutonomousController::AutonomousInit() {
  frc::SmartDashboard::PutBoolean("Auto Complete", false);
  mAutoSelected = mChooser.GetSelected();
  mAutoSelectedOptionsDirection = mChooserOptionsDirection.GetSelected();
  mAutoSelectedOptionsWait = mChooserOptionsWait.GetSelected();
  if (mAutoSelected == kAutoNameDemoDriving) {
      frc::SmartDashboard::PutBoolean("Auto Does Something", true);
  } else if (mAutoSelected == kAutoNameDriveOnly) {
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
        mDASAutoState = kDASReadying;
      }
      break;
    case kDASReadying:
      mDrive->StopMotor();
      mShooter->FixedSpeedForAuto();
      mShooter->FixedElevationForAuto(); // don't wait for this to be true
      // mShooter->CheckLimelight();
      // mShooter->Shoot(true, kDriveOnTarget);
      if (mTimer.Get() > kAutoShootTime) {
        mDASAutoState = kDASShooting;
      }
      break;
    case kDASShooting:
      mDrive->StopMotor();
      mShooter->ShootForAuto();
      if (mTimer.Get() > kAutoShootTimeEnd) {
        mDASAutoState = kDASCompleted;
      }
      break;
    case kDASCompleted:
      mDrive->StopMotor();
      mShooter->Idle();
      mFeeder->StopFeedingCargo();
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
  } else if (mAutoSelected == kAutoNameDemoDriving) {
    //DemoDriving(gamepad);  // this doesn't work because controllers are disabled in auto
  } else {
    mDrive->StopMotor();
  }
}
