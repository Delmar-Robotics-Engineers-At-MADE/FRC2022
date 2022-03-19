#include <Climber.h>
#include <iostream>

enum Constants {
  kTimeoutMs = 30
};
const double kClimberPosMiddle = 100000;
const double kClimberPosLow = 50000;
const double kClimberPosRetracted = 20;

void Climber::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot){


  int povPad = copilot->GetPOV();
  double target = 0;
  std::string msg;

  // TODO: untested
  if (true == smartClimber) {
    switch(povPad) {
      case 0:
        mClimberStar.Set(ControlMode::Position, kClimberPosMiddle);
        break;
      case 90:
      case 270:
        mClimberStar.Set(ControlMode::Position, kClimberPosLow);
        break;
      case 180:
        mClimberStar.Set(ControlMode::Position, kClimberPosRetracted);
        break;
      default:
        // nothing pressed, stop motors
        mClimberStar.Set(ControlMode::PercentOutput, 0);
        break;
    }
  } else {

  }

  // TODO: add code to check home sensor position and re-enable smart
  // control if moved manually back to home.
  
  if (mClimberStar.GetControlMode() == ControlMode::Position) { std::cout << "Position "; }
  std::cout << "target: " << target << " -- ";
  std::cout << "pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;

  // bool middleBarButtonPressed = copilot->GetRawButton(16);
  // bool lowBarButtonPressed = copilot->GetRawButton(13);
  // bool retractButtonPressed = copilot->GetRawButton(14);

  // if (retractButtonPressed) {
  //   mClimberStar.Set(ControlMode::Position, kClimberPosRetracted);
  // } else if (lowBarButtonPressed) {
  //   mClimberStar.Set(ControlMode::Position, kClimberPosLow);
  // } else if (middleBarButtonPressed) {
  //   mClimberStar.Set(ControlMode::Position, kClimberPosMiddle);
  // } else { // stop the motors
  //   mClimberStar.Set(ControlMode::PercentOutput, 0);
  // }
}

void Climber::RobotInit(){
  mClimberStar.ConfigFactoryDefault();

  /**
   * Grab the 360 degree position of the MagEncoder's absolute
   * position, and intitally set the relative sensor to match.
   */
  int absolutePosition = mClimberStar.GetSensorCollection().GetPulseWidthPosition();
  /* use the low level API to set the quad encoder signal */
  mClimberStar.SetSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);

  /* choose the sensor and sensor direction */
  mClimberStar.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
  mClimberStar.SetSensorPhase(false);

  /* set the peak and nominal outputs, 12V means full */
  mClimberStar.ConfigNominalOutputForward(0, kTimeoutMs);
  mClimberStar.ConfigNominalOutputReverse(0, kTimeoutMs);
  mClimberStar.ConfigPeakOutputForward(1, kTimeoutMs);
  mClimberStar.ConfigPeakOutputReverse(-1, kTimeoutMs);

  /* set closed loop gains in slot0 */
  mClimberStar.Config_kF(0, 0.0, kTimeoutMs);
  mClimberStar.Config_kP(0, 0.5, kTimeoutMs);
  mClimberStar.Config_kI(0, 0.0, kTimeoutMs);
  mClimberStar.Config_kD(0, 0.0, kTimeoutMs);

  mClimberStar.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  
  std::cout << "RobotInit pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;

}

void Climber::DoOnceInit() {
  mClimberStar.SetSelectedSensorPosition (0.0 , 0);
  std::cout << "DoOnce pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;

  // TODO: This does not work correctly yet
  if (portLimit.Get()) {
    frc::SmartDashboard::PutBoolean("Port Homed", false);
  } else {
    frc::SmartDashboard::PutBoolean("Port Homed", true);
    smartClimber = true;
  }

}
