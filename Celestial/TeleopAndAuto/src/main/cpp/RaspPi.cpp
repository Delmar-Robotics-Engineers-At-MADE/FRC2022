#include <RaspPi.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

static const double kMinAreaToBeNear = 0.02;

RaspPi::RaspPi () {  // constructor
  // get network table populated by python code on Pi
  mMLTable = nt::NetworkTableInstance::GetDefault().GetTable("ML");
}

void RaspPi::CheckForBall() {
  double index = mMLTable->GetNumber("nearestindex", -1.0);
  bool ballNear = false;
  if (index > -1.0) {
    mNearestBallArea = mMLTable->GetNumber("nearestarea", 0.0);
    if (mNearestBallArea > kMinAreaToBeNear) {
      ballNear = true;
    }
  }
  mBallAhead = ballNear;
  if (mBallAhead) {
    mNearestBallX = mMLTable->GetNumber("nearestx", 0.0);
  }
}
