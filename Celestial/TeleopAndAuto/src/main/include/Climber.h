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
  frc::DigitalInput mPortLimitSwitch{2};
  bool mSmartClimberEnabled {false};

  void SmartClimber(int povPad);
  void ManualClimber(frc::Joystick *copilot);
};