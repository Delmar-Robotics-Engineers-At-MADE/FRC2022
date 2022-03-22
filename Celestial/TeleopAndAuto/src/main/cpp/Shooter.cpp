#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

static const double kMinTargetAreaPercent = 0.0;

double ConvertRadsToDegrees (double rads) {
    const static double conversion_factor = 180.0/3.141592653589793238463;
    return rads * conversion_factor;
}

double ConvertDegreesToRads (double degs) {
    const static double conversion_factor = 3.141592653589793238463/180.0;
    return degs * conversion_factor;
}

// h1 = height of limelight; h2 = height of target above limelight; 
// phi = angle of limelight from vertical; theta = vertical angle of target reported by limelight
// distance to target = h2 / TAN (theta + phi)

Shooter::Shooter () {  // constructor
  // get network table populated by LimeLight
  mLimeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Shooter::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot){
  mPhi = frc::SmartDashboard::GetNumber("Phi", mPhi); // angle of limelight from vertical
  mH2 = frc::SmartDashboard::GetNumber("Phi", mH2); // height of target above limelight

  double tv = mLimeTable->GetNumber("tv",0.0);  
  bool targetSeen = (tv != 0.0); // tv is non-zero if there is a target detected
  double targetArea = mLimeTable->GetNumber("ta",0.0);
  double targetAngleHorizontal = 0.0;
  double targetAngleVertical = 0.0;
  double targetDistance = 0.0;
  bool lightOn = false;

  if (copilot->GetRawButton(2)) { // shoot at high goal
    lightOn = true;
  } else if (copilot->GetRawButton(4)) { // shoot at low goal
    lightOn = true;
  } else { // no shooter buttons pressed
    lightOn = false;
  }

  if (lightOn) {
    mLimeTable->PutNumber("ledMode",3.0); // LED on bright
  } else {
    mLimeTable->PutNumber("ledMode",1.0); // LED off
  }

  if (targetSeen) {
    if (targetArea > kMinTargetAreaPercent) {  
      targetAngleHorizontal = mLimeTable->GetNumber("tx",0.0);
      targetAngleVertical = mLimeTable->GetNumber("ty",0.0);   
      targetDistance = mH2 / tan(ConvertDegreesToRads(targetAngleVertical));
    }
  }

  frc::SmartDashboard::PutBoolean("Target Seen", targetSeen);
  frc::SmartDashboard::PutNumber("Target Area", targetArea);
  frc::SmartDashboard::PutNumber("Angle Horiz", targetAngleHorizontal);
  frc::SmartDashboard::PutNumber("Angle Vert", targetAngleVertical);
  frc::SmartDashboard::PutNumber("Distance", targetDistance);
}

void Shooter::RobotInit() {
  frc::SmartDashboard::PutNumber("Phi", mPhi);
  frc::SmartDashboard::PutNumber("kH2", mPhi);

  mLimeTable->PutNumber("camMode",0.0); // camera in normal CV mode
  mLimeTable->PutNumber("ledMode",1.0); // LED off
  mLimeTable->PutNumber("stream",0.0);  // secondary camera side-by-side
}

