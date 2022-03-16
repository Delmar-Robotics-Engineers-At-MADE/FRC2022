#ifndef PIXYBALLTRACKER
#define PIXYBALLTRACKER

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/controller/PIDController.h>


class PixyBallTracker  {

private:
  std::shared_ptr<nt::NetworkTable> mPixyTable;
  frc2::PIDController *mPIDController;


 public:
    PixyBallTracker (double p, double i, double d);  // constructor
    bool BallSeen();
    double CalculateResponse();
};

#endif