#include <Climber.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

static const double kPowerUp = 0.3;//1.0;
static const double kPowerDown = -0.3;//-1.0;
constexpr units::time::second_t kRatchetTimeDelay = 1.0_s; // seconds

// enum Constants {
//   kTimeoutMs = 30
// };

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

void Climber::StopClimbers() {
  mClimberPort.StopMotor();
  mClimberStar.StopMotor();
  mRatchetState = kRatchetIdle;
  mSolenoid.Set(frc::DoubleSolenoid::kReverse); // reverse releases wire, allowing ratchet to engage
}

void Climber::OpenRatchetIfExtending (double powerPort, double powerStar) {
  // actually open ratchet if moving in either direction

  // std::cout << "climber power port: " << powerPort << ", power star: " << powerStar << std::endl;

  // if (powerPort > 0.0 || powerStar > 0.0) {
  //   // positive means climbers extending, which is when ratchet needs to be open
  //   mSolenoid.Set(frc::DoubleSolenoid::kForward);
  // } else { // not extending, so let ratchet close
  //   mSolenoid.Set(frc::DoubleSolenoid::kReverse);
  // }
  if (powerPort == 0.0 && powerStar == 0.0) {
    // no delay needed when stopping
    mRatchetState = kRatchetIdle;
    mSolenoid.Set(frc::DoubleSolenoid::kReverse); // reverse releases wire, allowing ratchet to engage
  } else { // wanting to move climbers
    switch (mRatchetState) {
      default:
      case kRatchetIdle:
        // need to move ratchet and delay a bit before moving climber
        mSolenoid.Set(frc::DoubleSolenoid::kForward); // forward pulls wire to open ratchet
        mRatchetState = kRatchetMoving;
        mRatchetTimer.Reset();
        mRatchetTimer.Start();
        break;
      case kRatchetMoving:
        if (mRatchetTimer.Get() > kRatchetTimeDelay) {
          mRatchetState = kRatchetMoved;
        }
        break;
      case kRatchetMoved:
        // now ok to move motor
        break;
    }
  }
}

void Climber::ManualClimber(frc::Joystick *copilot){
  bool TODO_Add_Dashboard_Displays = false;
  double powerPort = 0.0;
  double powerStar = 0.0;

  bool climberIsOnStopPort = false; // !mLimitSwitchPort.Get();
  bool climberIsOnStopStar = false; // !mLimitSwitchStar.Get();
  bool climberIsAtTopSmartLimitPort = (mSmartClimberEnabled && mClimberPort.GetSelectedSensorPosition(0) >= kClimberPosMiddle);
  bool climberIsAtTopSmartLimitStar = (mSmartClimberEnabled && mClimberStar.GetSelectedSensorPosition(0) >= kClimberPosMiddle);

  // check controls
  if (copilot->GetRawButton(5)) {
    powerPort = kPowerUp;
  } else if (copilot->GetRawButton(7)) {
    powerPort = kPowerDown;
  }
  if (copilot->GetRawButton(6)) {
    powerStar = kPowerUp;
  } else if (copilot->GetRawButton(8)) {
    powerStar = kPowerDown;
  }

  // don't allow operator to lower climber if it's on the stop or beyond the smart setpoints
  if (climberIsOnStopPort && powerPort < 0.0) {powerPort = 0.0;}
  if (climberIsOnStopStar && powerStar < 0.0) {powerStar = 0.0;}
  if (climberIsAtTopSmartLimitPort && powerPort > 0.0) {powerPort = 0.0;}
  if (climberIsAtTopSmartLimitStar && powerStar > 0.0) {powerStar = 0.0;}

  OpenRatchetIfExtending (powerPort, powerStar);

  // std::cout << "ratchet state: " << mRatchetState << std::endl;
  if (mRatchetState != kRatchetMoving) {
    // std::cout << "powering climbers" << std::endl;
    mClimberPort.Set(ControlMode::PercentOutput, powerPort);
    mClimberStar.Set(ControlMode::PercentOutput, powerStar);
    if (powerPort == 0.0) {
      // std::cout << "stopping port" << std::endl;
      mClimberPort.StopMotor(); // above power set should be same but isn't
    }
    if (powerStar == 0.0) {
      // std::cout << "stopping star" << std::endl;
      mClimberStar.StopMotor(); // above power set should be same but isn't
    }
  }

  // std::cout << "power port: " << powerPort << std::endl;
  // std::cout << "power star: " << powerStar << std::endl;
  // std::cout << "smart climber " << mSmartClimberEnabled << std::endl;
  // std::cout << "output: " << mClimberStar.GetMotorOutputPercent()  << " -- ";
  // std::cout << "pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;
}

void Climber::TeleopInit(){
  mEndgameTimer.Reset();
  mEndgameTimer.Start();
}

void Climber::TelopPeriodic (frc::Joystick *copilot){

  bool endgameOverridePressed = copilot->GetRawButton(1);
  int povPad = copilot->GetPOV();
  bool smartControl = false;
  // bool manualControl = false;
  if (!mSmartClimberEnabled) {
    // motor not homed, so cannot use smart climber positions
    // manualControl = true;
  } else if (povPad == -1) {
    // nothing pressed on POV, so manual control is ok
    // manualControl = true;
  } else {
    // pov takes priority over manual control
    smartControl = true;
  }

  if (endgameOverridePressed || mEndgameTimer.Get() > 2.0_min) { // don't let any controls work until endgame, unless override button pressed
    //std::cout << "endgame timer: " << (mEndgameTimer.Get() > 2_min) << std::endl;
    if (smartControl) {
      SmartClimber(povPad);
    } else {
      ManualClimber(copilot);
      CheckHomePositions();  // if operator needs to home a climber to enable smart-climber, check here
    }
  } else {
    // possibly just let up on blue button
    StopClimbers();
  }

}

void Climber::RobotInit(){

  // std::cout << "RobotInit of climber" << std::endl;

  mClimberStar.ConfigFactoryDefault();
  mClimberPort.ConfigFactoryDefault();

  mClimberStar.SetInverted(true);
  mClimberPort.SetInverted(false);

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
  mClimberPort.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  
  // std::cout << "RobotInit pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;

}

void Climber::CheckHomePositions() {
  bool TODO_Set_Smart_Climber_Positions = false;
  if (!mSmartClimberEnabled) { // no need to do this check if this is already set true
    bool climberIsOnStopPort = !mLimitSwitchPort.Get();
    bool climberIsOnStopStar = !mLimitSwitchStar.Get();
    frc::SmartDashboard::PutBoolean("Port Homed", climberIsOnStopPort);
    frc::SmartDashboard::PutBoolean("Starboard Homed", climberIsOnStopStar);
    if (climberIsOnStopPort && climberIsOnStopStar) {
      mSmartClimberEnabled = true;
    }
  }
  mSmartClimberEnabled = false;  // until we have numbers and limit switches
  frc::SmartDashboard::PutNumber("Port Climber", mClimberPort.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Star Climber", mClimberStar.GetSelectedSensorPosition());

}

void Climber::DoOnceInit() {
  mClimberPort.SetSelectedSensorPosition (0.0 , 0);
  mClimberStar.SetSelectedSensorPosition (0.0 , 0);
  // std::cout << "DoOnce pos: " << mClimberStar.GetSelectedSensorPosition(0) << std::endl;
  CheckHomePositions();
}

void Climber::RepeatableInit() {
  // OpenRatchetIfExtending(0,0); // release ratchet to engage
  // mClimberPort.StopMotor();
  // mClimberStar.StopMotor();
  StopClimbers();
}