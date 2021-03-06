#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/Joystick.h>
#include <PixyBallTracker.h>
#include <AutonomousController.h>
#include <Climber.h>
#include <Shooter.h>
#include <DriveSystem.h>
#include <Intake.h>

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
  DriveSystem mRobotDrive{mFrontLeft, mRearLeft, mFrontRight, mRearRight};

  frc::Joystick mPilot{0};
  frc::Joystick mCopilot{1};

  PixyBallTracker *mTracker;

  // VelocityController2 *mVelocityController;


  // frc::Timer mAutonomousTimer;

  bool mDoOnceInited = false;

  void DoOnceInit();
  void RepeatableInit();
};