#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/Joystick.h>
#include <PixyBallTracker.h>
#include <AutonomousController.h>
#include <Climber.h>
#include <Shooter.h>
#include <DriveSystem.h>
#include <Intake.h>
#include <RaspPi.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void RobotPeriodic() override;

private:
  rev::CANSparkMax mFrontLeft{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax mRearLeft{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax mFrontRight{14, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax mRearRight{13, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController mPIDFrontLeft  = mFrontLeft .GetPIDController();
  rev::SparkMaxPIDController mPIDRearLeft   = mRearLeft  .GetPIDController();
  rev::SparkMaxPIDController mPIDFrontRight = mFrontRight.GetPIDController();
  rev::SparkMaxPIDController mPIDRearRight  = mRearRight .GetPIDController();

  rev::SparkMaxRelativeEncoder mEncoderFrontLeft  = mFrontLeft .GetEncoder();
  rev::SparkMaxRelativeEncoder mEncoderRearLeft   = mRearLeft  .GetEncoder();
  rev::SparkMaxRelativeEncoder mEncoderFrontRight = mFrontRight.GetEncoder();
  rev::SparkMaxRelativeEncoder mEncoderRearRight  = mRearRight .GetEncoder();
  
  DriveSystem mRobotDrive{mFrontLeft, mRearLeft, mFrontRight, mRearRight};

  frc::Joystick mPilot{0};
  frc::Joystick mCopilot{1};

  PixyBallTracker *mTracker;
  RaspPi mRaspPi;

  // VelocityController2 *mVelocityController;
  AutonomousController *mAutoController;
  Climber mClimber;
  Shooter *mShooter;
  Intake mIntake;

  //frc::DoubleSolenoid mTempSolenoid{frc::PneumaticsModuleType::CTREPCM, 4, 5};

  bool mDoOnceInited = false;

  void DoOnceInit();
  void RepeatableInit();
};