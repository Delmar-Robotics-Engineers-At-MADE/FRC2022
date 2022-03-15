#include <frc/drive/MecanumDrive.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>


class VelocityController {

// 60 in/2sec at full speed
// 26 in/2sec at half speed

private:
  frc::MecanumDrive *mRobotDrive;
  frc::TrapezoidProfile<units::feet>::Constraints mConstraints{2.0_fps, 0.5_fps_sq};
  frc::TrapezoidProfile<units::feet>::State mGoal;
  frc::TrapezoidProfile<units::feet>::State mSetpoint;

public:
  VelocityController (frc::MecanumDrive *drive); // constructor
  double ForwardAtSpeed (units::feet_per_second_t feetPerSec);
  void SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps);
  bool DriveTrapezoid(); // returns true when done

};