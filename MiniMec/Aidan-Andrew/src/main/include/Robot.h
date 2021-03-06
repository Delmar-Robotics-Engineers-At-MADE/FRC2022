#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include "../include/pixyBallTracker.h"

class Robot : public frc::TimedRobot{
    void RobotInit() override;
    void TeleopPeriodic() override;
private:
 static constexpr int kFrontLeftChannel = 0;
 static constexpr int kRearLeftChannel = 1;
 static constexpr int kFrontRightChannel = 2;
 static constexpr int kRearRightChannel = 3;

 static constexpr int kJoystickChannel = 0;
 frc::Talon m_frontLeft{kFrontLeftChannel};
 frc::Talon m_rearLeft{kRearLeftChannel};
 frc::Talon m_frontRight{kFrontRightChannel};
 frc::Talon m_rearRight{kRearRightChannel};
 frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};

 frc::Joystick m_stick{kJoystickChannel};

 pixyBallTracker *mTracker;
};