#include <DriveSystem.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/smartdashboard/SmartDashboard.h>

// constructor
DriveSystem::DriveSystem(frc::SpeedController& frontLeftMotor, frc::SpeedController& rearLeftMotor,
                         frc::SpeedController& frontRightMotor, frc::SpeedController& rearRightMotor) :

  MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor) {
                
    // merely call parent class constructor

  }

void DriveSystem::TelopPeriodic (frc::Joystick *pilot){

  if (pilot->GetRawButton(6) && pilot->GetRawButton(4)) {
    mAHRS->ZeroYaw();   // use current robot orientation as field forward
  } else {
    DriveCartesian(pilot->GetY(), -pilot->GetX(), -pilot->GetZ(), mAHRS->GetAngle());
  }
  double IR = mColorSensor.GetIR();
  frc::SmartDashboard::PutNumber("Rev Color IR", IR);
}

void DriveSystem::RobotInit() {

  try{
      mAHRS = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception &ex) {
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char *p_err_msg = err_msg.c_str();
      FRC_ReportError(0, "{}", p_err_msg); // frc::DriverStation::ReportError(p_err_msg);
    }
}

void DriveSystem::DoOnceInit()  {
    mAHRS->ZeroYaw();   // use current robot orientation as field forward
}