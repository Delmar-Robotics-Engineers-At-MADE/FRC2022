#ifndef AUTOCONTROLLER
#define AUTOCONTROLLER

#include <frc/smartdashboard/SendableChooser.h>
#include <VelocityController2.h>

enum DriveTurnStates {
		kdtsUnknownState = 0,
    kdtsBegin,
		kdtsDriving,
		kdtsTurning,
		kdtsCompleted
	};

enum TurnOnlyStates {
		ktosUnknownState = 0,
    ktosBegin,
    ktosTurning,
		ktosCompleted
	};
class AutonomousController {

public:
  AutonomousController (VelocityController2 *vcontroller); // constructor
  void AutonomousInit();
  void AutonomousPeriodic();

private:
  frc::SendableChooser<std::string> mChooser;
  frc::SendableChooser<std::string> mChooserOptionsDistance;
  frc::SendableChooser<std::string> mChooserOptionsWait;
  const std::string kAutoNameDriveTurnAround = "Drive, Stop, Turn Around";
  const std::string kAutoNameDriveOnly = "Drive Only";
  const std::string kAutoNameTurnOnly = "Turn Only";
  const std::string kAutoNameJustInit = "Just Init";
  const std::string kAutoOption2Feet = "2 Feet";
  const std::string kAutoOption4Feet = "4 Feet";
  const std::string kAutoOptionWait = "Wait";
  const std::string kAutoOptionNoWait = "No wait";
  std::string mAutoSelected;
  std::string mAutoSelectedOptionsDistance;
  std::string mAutoSelectedOptionsWait;

  VelocityController2 *mVelocityController;
  DriveTurnStates mdtsAutoState = kdtsUnknownState; 
  TurnOnlyStates mtosAutoState = ktosUnknownState; 

};

#endif