#include <Intake.h>

  void TeleopPeriodic() override {
    /* The output of GetRawButton is true/false depending on whether the button
     * is pressed; Set takes a boolean for for whether to use the default
     * (false) channel or the other (true).
     */
    m_solenoid.Set(m_stick.GetRawButton(kSolenoidButton));

    /* In order to set the double solenoid, we will say that if neither button
     * is pressed, it is off, if just one button is pressed, set the solenoid to
     * correspond to that button, and if both are pressed, set the solenoid to
     * Forwards.
     */
    if (m_stick.GetRawButton(kDoubleSolenoidForward)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick.GetRawButton(kDoubleSolenoidReverse)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
    }
  }