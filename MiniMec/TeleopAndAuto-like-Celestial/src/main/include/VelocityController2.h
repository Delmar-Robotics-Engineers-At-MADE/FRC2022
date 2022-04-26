#ifdef DELETEME

#include <frc/drive/MecanumDrive.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <frc/Joystick.h>
#include <PixyBallTracker.h>

class VelocityController2 {

// 60 in/2sec at full speed
// 26 in/2sec at half speed

private:
  frc::MecanumDrive *mRobotDrive;
  AHRS *mAHRS;
  frc::TrapezoidProfile<units::feet>::Constraints mConstraints{2.5_fps, 1_fps_sq};
  frc::TrapezoidProfile<units::feet>::State mGoal;
  frc::TrapezoidProfile<units::feet>::State mInitialState;
  frc::TrapezoidProfile<units::feet> *mProfile;
  frc::Timer mTimer;  
  frc2::PIDController *mPIDControllerGyro; // for orienting robot with gyro

public:
  VelocityController2 (frc::MecanumDrive *drive, AHRS *gyro); // constructor
  double ForwardAtSpeed (units::feet_per_second_t feetPerSec, double gyroAngle);
  void SetTrapezoidGoal (units::foot_t distance, units::feet_per_second_t fps);
  bool DriveTrapezoid(); // returns true when done
  void StartMotionTimer();
  void StopDriving();
  bool TurnRight (double degrees);
  bool TurnStraight ();
  void TelopPeriodic (frc::Joystick *pilot);
  void TrackBall (PixyBallTracker *tracker);
};

#endif