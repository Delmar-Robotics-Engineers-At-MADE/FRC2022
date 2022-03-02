#include <pixyBallTracker.h>

pixyBallTracker::pixyBallTracker (double p, double i, double d) {
    
    nt::NetworkTableInstance networkTables = nt::NetworkTableInstance::GetDefault();
    mPixyTable = networkTables.GetTable("PixyBlocks");

    mPidController = new frc2::PIDController (p, i, d);
    mPidController->SetTolerance(8, 8);
    mPidController->SetSetpoint(150);
}
bool pixyBallTracker::ballSeen() {
    return (mPixyTable->GetNumber("STATUS", 0) == 1);
}
double pixyBallTracker::calculateResponse(){
    double x = mPixyTable->GetNumber("X", 150);
    double speed = mPidController->Calculate(x);
    return (speed);
}
