#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "frc/DigitalInput.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Climber {
public:
  void TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot);
  void RobotInit();
  void DoOnceInit();

private:
  // WPI_TalonSRX mClimberPort{3};
  // WPI_TalonSRX mClimberStar{12};
  WPI_TalonSRX mClimberStar{8};  // on pinoccio
  frc::DigitalInput portLimit{2};
  bool smartClimber {false};
};