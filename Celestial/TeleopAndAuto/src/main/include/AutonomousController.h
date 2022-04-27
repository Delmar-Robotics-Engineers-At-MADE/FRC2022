#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <DriveSystem.h>
#include <Shooter.h>

// enum DriveTurnStates {
// 		kdtsUnknownState = 0,
//     kdtsBegin,
// 		kdtsDriving,
//     kdtsStraightening,
// 		kdtsTurning,
// 		kdtsCompleted
// 	};

enum DriveOnlyStates {
		kDOSUnknownState = 0,
    kDOSBegin,
    kDOSDriving,
		kDOSCompleted
	};

enum DriveAndShootStates {
		kDASUnknownState = 0,
    kDASBegin,
    kDASDriving,
    kDASReadying,
    kDASShooting,
		kDASCompleted
	};

class AutonomousController {

public:
  AutonomousController (DriveSystem *drive, Shooter *shooter); // constructor
  void AutonomousInit();
  void AutonomousPeriodic();

private:
  DriveSystem *mDrive;
  Shooter *mShooter;
  
  frc::SendableChooser<std::string> mChooser;
  frc::SendableChooser<std::string> mChooserOptionsDirection;
  frc::SendableChooser<std::string> mChooserOptionsWait;
  const std::string kAutoNameDemoDriving = "Demo Driving";
  const std::string kAutoNameDriveAndShoot = "Drive And Shoot";
  const std::string kAutoNameDriveOnly = "Drive Only";
  const std::string kAutoNameJustInit = "Just Init";
  const std::string kAutoOptionDirStraight = "Straight";
  const std::string kAutoOptionDirLeft = "To Left";
  const std::string kAutoOptionDirRight = "To Right";
  const std::string kAutoOptionWait = "Wait";
  const std::string kAutoOptionNoWait = "No wait";
  std::string mAutoSelected;
  std::string mAutoSelectedOptionsDirection;
  std::string mAutoSelectedOptionsWait;

  // VelocityController2 *mVelocityController;
  // DriveTurnStates mdtsAutoState = kdtsUnknownState; 
  // TurnOnlyStates mtosAutoState = ktosUnknownState; 
  DriveOnlyStates mDOSAutoState = kDOSUnknownState;
  DriveAndShootStates mDASAutoState = kDASUnknownState;

  frc::Timer mTimer; 

  // void DriveTurnAround();
  // void TurnOnly();
  void DriveOnly();
  void DriveAndShoot();


};

