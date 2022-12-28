

#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/Joystick.h>
#include "AHRS.h"
#include <PixyBallTracker.h>
#include <VelocityController.h>
#include <AutonomousController.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;

private:
  frc::Talon mFrontLeft{1};
  frc::Talon mRearLeft{0};
  frc::Talon mFrontRight{3};
  frc::Talon mRearRight{2};
  frc::MecanumDrive mRobotDrive{mFrontLeft, mRearLeft, mFrontRight, mRearRight};

  frc::Joystick mStick{0};

  AHRS *mAHRS;

  PixyBallTracker *mTracker;

  VelocityController2 *mVelocityController;
  AutonomousController *mAutoController;

  // frc::Timer mAutonomousTimer;

  bool mDoOnceInited = false;

  void DoOnceInit();
  void RepeatableInit();
};