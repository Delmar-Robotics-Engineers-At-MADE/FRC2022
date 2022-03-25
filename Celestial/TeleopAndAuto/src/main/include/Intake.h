#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>

class Intake {
 public:
    
    frc::DoubleSolenoid *mDoubleSolenoid;


 private:
  frc::Joystick m_stick{0};

  // Solenoid corresponds to a single solenoid.
  frc::Solenoid m_solenoid{frc::PneumaticsModuleType::CTREPCM, 0};

  // DoubleSolenoid corresponds to a double solenoid.
  frc::DoubleSolenoid m_doubleSolenoid{frc::PneumaticsModuleType::CTREPCM, 4,
                                       5};

  static constexpr int kSolenoidButton = 1;
  static constexpr int kDoubleSolenoidForward = 2;
  static constexpr int kDoubleSolenoidReverse = 3;

  void TeleopPeriodic();
};