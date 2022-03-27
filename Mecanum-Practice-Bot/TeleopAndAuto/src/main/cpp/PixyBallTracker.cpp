#include <PixyBallTracker.h>

PixyBallTracker::PixyBallTracker (double p, double i, double d) {
      
  // get network table populated by raspberry pi using pixycam
  nt::NetworkTableInstance networkTables = nt::NetworkTableInstance::GetDefault();
  mPixyTable = networkTables.GetTable("PixyBlocks");
      
  // use pid for ball tracking
  mPIDController = new frc2::PIDController (p, i, d);
  mPIDController->SetTolerance(8, 8);  // pixy cam image coords, roughly 0 to 300
  mPIDController->SetSetpoint(150); // center of image
  }

bool PixyBallTracker::BallSeen(){
  return (mPixyTable->GetNumber("STATUS", 0) == 1);
}

double PixyBallTracker::CalculateResponse(){
  double x = mPixyTable->GetNumber("X", 150); // position of ball
  double speed = mPIDController->Calculate(x);
  return (speed);
}
