#pragma once

#include <networktables/NetworkTable.h>

class RaspPi {
public:
  RaspPi (); // constructor

  double mNearestBallArea;
  double mNearestBallX;
  bool mBallAhead;

  void CheckForBall();

private:
  std::shared_ptr<nt::NetworkTable> mMLTable; // Machine Learning stuff from python code on Pi
};