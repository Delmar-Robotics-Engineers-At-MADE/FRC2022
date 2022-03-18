#include <frc/Joystick.h>
#include "ctre/Phoenix.h"

class Climber {
public:

  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot);
  void RobotInit();
  void DoOnceInit();

private:
  // WPI_TalonSRX mClimberPort{3};
  // WPI_TalonSRX mClimberStar{12};
  WPI_TalonSRX mClimberStar{8};  // on pinoccio
};