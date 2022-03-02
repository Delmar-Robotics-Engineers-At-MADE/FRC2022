#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/controller/PIDController.h>

const double kp = 0.012;
const double ki = 0.0;
const double kd = 0.00012;

class pixyBallTracker {
private:
    std::shared_ptr<nt::NetworkTable> mPixyTable;
    frc2::PIDController *mPidController;

    public:
    pixyBallTracker (double p, double i, double d);
    bool ballSeen();
    double calculateResponse();
};
  