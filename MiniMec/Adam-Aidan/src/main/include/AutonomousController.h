#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/MecanumDrive.h>

enum DriveTurnStates {
    kUnkownState = 0,
    kBegin,
    kDriving,
    kCompleted
};

class AutonomousController {

public:
    AutonomousController (frc::MecanumDrive *drive);
    void AutonomousInit();
    void AutonomousPeriodic();

private:
frc::SendableChooser<std::string> mChooser;
const std::string kAutoNameDriveOnly = "Drive Only";
const std::string kAutoNameJustInit = "Just Init";
std::string mAutoSelected;
frc::MecanumDrive *mDrive;
DriveTurnStates mDriveStates = kUnkownState;
frc::Timer mTimer;

void Driving ();
void NotDriving();

};