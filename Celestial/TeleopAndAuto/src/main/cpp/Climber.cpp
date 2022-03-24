#include <Climber.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

enum Constants {
  kTimeoutMs = 30
};

// One rotation is 4096 encoder counts, and encoder is at the output of the gearbox
const double kClimberPosMiddle = 10 * 4096;
const double kClimberPosLow = 6 * 4096;
const double kClimberPosRetracted = 1 * 4096;

void Climber::SmartClimber(int povPad){
  double target = 0.0;
  switch(povPad) {
    case 0:
      target = kClimberPosMiddle;
      break;
    case 90:
    case 270:
      target = kClimberPosLow;
      break;
    case 180:
      target = kClimberPosRetracted;
      break;
    case -1:
    default:
      // nothing pressed, stop motors
      mClimberPort.Set(ControlMode::PercentOutput, 0);
      mClimberStar.Set(ControlMode::PercentOutput, 0);
      break;
    }
  if (target > 0.0) {
    mClimberPort.Set(ControlMode::Position, target);
    mClimberStar.Set(ControlMode::Position, target);
  }

  std::cout << "target: " << target << " -- ";
  std::cout << "pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;
}

void Climber::ManualClimber(frc::Joystick *copilot){
  bool climberIsOnStopPort = !mLimitSwitchPort.Get();
  bool climberIsOnStopStar = !mLimitSwitchStar.Get();
  bool climberIsAtTopSmartLimitPort = (mSmartClimberEnabled && mClimberPort.GetSelectedSensorPosition(0) >= kClimberPosMiddle);
  double powerPort = -copilot->GetRawAxis(0);
  double powerStar = -copilot->GetRawAxis(3);

  // don't allow operator to lower climber if it's on the stop or beyond the smart setpoints
  if (!climberIsOnStopPort || powerPort > 0.0) {mClimberPort.Set(ControlMode::PercentOutput, powerPort);}
  if (!climberIsOnStopStar || powerStar > 0.0) {mClimberStar.Set(ControlMode::PercentOutput, powerStar);}

  std::cout << "output: " << mClimberStar.GetMotorOutputPercent()  << " -- ";
  std::cout << "pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;
}

void Climber::TeleopInit(){
  mTimer.Reset();
  mTimer.Start();
}

void Climber::TelopPeriodic (frc::Joystick *copilot){

  bool endgameOverridePressed = copilot->GetRawButton(1);
  int povPad = copilot->GetPOV();
  bool smartControl = false;
  bool manualControl = false;
  if (!mSmartClimberEnabled) {
    // motor not homed, so cannot use smart climber positions
    manualControl = true;
  } else if (povPad == -1) {
    // nothing pressed on POV, so manual control is ok
    manualControl = true;
  } else {
    // pov takes priority over manual control
    smartControl = true;
  }

  if (endgameOverridePressed || mTimer.Get() > 2.5_s) { // don't let any controls work until endgame, unless override button pressed
    if (smartControl) {
      SmartClimber(povPad);
    } else {
      ManualClimber(copilot);
      CheckHomePositions();  // if operator needs to home a climber to enable smart-climber, check here
    }
  }

}

void Climber::RobotInit(){
  mClimberStar.ConfigFactoryDefault();

  /**
   * Grab the 360 degree position of the MagEncoder's absolute
   * position, and intitally set the relative sensor to match.
   */
  int absolutePosition = mClimberStar.GetSensorCollection().GetPulseWidthPosition();
  mClimberStar.SetSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);
  absolutePosition = mClimberPort.GetSensorCollection().GetPulseWidthPosition();
  mClimberPort.SetSelectedSensorPosition(absolutePosition, 0, kTimeoutMs);

  /* choose the sensor and sensor direction */
  mClimberStar.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
  mClimberStar.SetSensorPhase(false);
  mClimberPort.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
  mClimberPort.SetSensorPhase(false);

  /* set the peak and nominal outputs, 12V means full */
  mClimberStar.ConfigNominalOutputForward(0, kTimeoutMs);
  mClimberStar.ConfigNominalOutputReverse(0, kTimeoutMs);
  mClimberStar.ConfigPeakOutputForward(1, kTimeoutMs);
  mClimberStar.ConfigPeakOutputReverse(-1, kTimeoutMs);
  mClimberPort.ConfigNominalOutputForward(0, kTimeoutMs);
  mClimberPort.ConfigNominalOutputReverse(0, kTimeoutMs);
  mClimberPort.ConfigPeakOutputForward(1, kTimeoutMs);
  mClimberPort.ConfigPeakOutputReverse(-1, kTimeoutMs);

  /* set closed loop gains in slot0 */
  mClimberStar.Config_kF(0, 0.0, kTimeoutMs);
  mClimberStar.Config_kP(0, 0.5, kTimeoutMs);
  mClimberStar.Config_kI(0, 0.0, kTimeoutMs);
  mClimberStar.Config_kD(0, 0.0, kTimeoutMs);
  mClimberPort.Config_kF(0, 0.0, kTimeoutMs);
  mClimberPort.Config_kP(0, 0.5, kTimeoutMs);
  mClimberPort.Config_kI(0, 0.0, kTimeoutMs);
  mClimberPort.Config_kD(0, 0.0, kTimeoutMs);

  mClimberStar.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  mClimberStar.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  
  std::cout << "RobotInit pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;

}

void Climber::CheckHomePositions() {
  if (!mSmartClimberEnabled) { // no need to do this check if this is already set true
    bool climberIsOnStopPort = !mLimitSwitchPort.Get();
    bool climberIsOnStopStar = !mLimitSwitchStar.Get();
    frc::SmartDashboard::PutBoolean("Port Homed", climberIsOnStopPort);
    frc::SmartDashboard::PutBoolean("Starboard Homed", climberIsOnStopStar);
    if (climberIsOnStopPort && climberIsOnStopStar) {
      mSmartClimberEnabled = true;
    }
  }
}

void Climber::DoOnceInit() {
  mClimberStar.SetSelectedSensorPosition (0.0 , 0);
  std::cout << "DoOnce pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;
  CheckHomePositions();
}
