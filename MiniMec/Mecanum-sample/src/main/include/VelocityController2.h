#include <frc/drive/MecanumDrive.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>


class VelocityController2 {

// 60 in/2sec at full speed
// 26 in/2sec at half speed

private:
  frc::MecanumDrive *mRobotDrive;
  frc::TrapezoidProfile<units::feet>::Constraints mConstraints{2.5_fps, 1_fps_sq};
  frc::TrapezoidProfile<units::feet>::State mGoal;
  frc::TrapezoidProfile<units::feet>::State mInitialState;
  frc::TrapezoidProfile<units::feet> *mProfile;
  frc::Timer mTimer;

public:
  VelocityController2 (frc::MecanumDrive *drive); // constructor
  double ForwardAtSpeed (units::feet_per_second_t feetPerSec);
  void SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps);
  bool DriveTrapezoid(); // returns true when done
  void StartMotionTimer();

};