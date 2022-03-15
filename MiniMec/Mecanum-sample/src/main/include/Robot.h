#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/Joystick.h>
#include "AHRS.h"
#include <PixyBallTracker.h>
#include <VelocityController2.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
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

  frc::Timer mAutonomousTimer;

  VelocityController2 *mVelocityController;

  bool mDoOnceInited = false;

  void DoOnceInit();
  void RepeatableInit();
};