#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/Joystick.h>
#include "AHRS.h"
#include <PixyBallTracker.h>
#include <AutonomousController.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override; 
  void AutonomousInit() override;
  void AutonomousPeriodic() override;

private:
  frc::Talon m_frontLeft{1};
  frc::Talon m_rearLeft{0};
  frc::Talon m_frontRight{3};
  frc::Talon m_rearRight{2};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};

  frc::Joystick m_stick{0};

  AHRS *m_ahrs;

  PixyBallTracker *mTracker;

  AutonomousController *mAutoController;
};