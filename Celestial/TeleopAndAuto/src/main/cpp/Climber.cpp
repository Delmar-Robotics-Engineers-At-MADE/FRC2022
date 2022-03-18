#include <Climber.h>
#include <iostream>

enum Constants {
  kTimeoutMs = 30
};
const double kClimberPosMiddle = 500;
const double kClimberPosLow = 200;
const double kClimberPosRetracted = 20;

void Climber::TelopPeriodic (frc::Joystick *pilot, frc::Joystick *copilot){

  int povPad = copilot->GetPOV();

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
  mClimberStar.Config_kP(0, 0.1, kTimeoutMs);
  mClimberStar.Config_kI(0, 0.0, kTimeoutMs);
  mClimberStar.Config_kD(0, 0.0, kTimeoutMs);

  mClimberStar.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

}

void Climber::DoOnceInit() {
  mClimberStar.SetSelectedSensorPosition (0.0 , 0);
}
