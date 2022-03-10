/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class Robot : public frc::TimedRobot {
  // initialize motor
  static const int deviceID = 62;
  rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};

  /**
   * In order to use PID functionality for a controller, a SparkMaxPIDController object
   * is constructed by calling the GetPIDController() method on an existing
   * CANSparkMax object
   */
  rev::SparkMaxPIDController m_pidController = m_motor.GetPIDController();

  // Encoder object created to display velocity values
  rev::SparkMaxRelativeEncoder m_encoder = m_motor.GetEncoder();

  frc::Joystick m_stick{0};

  // default PID coefficients
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  // motor max RPM
  const double MaxRPM = 5700;

  // for trapezoidal
  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{1.75_mps, 0.75_mps_sq};
  frc::TrapezoidProfile<units::meters>::State m_goal;
  frc::TrapezoidProfile<units::meters>::State m_setpoint;

  frc::SimpleMotorFeedforward<units::meters> m_feedforward{
    // Note: These gains are fake, and will have to be tuned for your robot.
    1_V, 1.5_V * 1_s / 1_m};

  static constexpr units::second_t kDt = 20_ms;

 public:
  void RobotInit() override {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.RestoreFactoryDefaults();
    
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }


  void TeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // read setpoint from joystick and scale by max rpm
    double SetPoint = 0.0;// = MaxRPM*m_stick.GetY();

    if (m_stick.GetRawButton(2)) {
      m_goal = {5_m, 0_mps};
    } else if (m_stick.GetRawButton(3)) {
      m_goal = {0_m, 0_mps};
    } else {
      m_goal = {0_m, 0_mps};
    }

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    frc::TrapezoidProfile<units::meters> profile{m_constraints, m_goal, m_setpoint};

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.Calculate(kDt);

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  rev::ControlType::kDutyCycle
     *  rev::ControlType::kPosition
     *  rev::ControlType::kVelocity
     *  rev::ControlType::kVoltage
     */
    
    // Send setpoint to motor, example is this:
    // m_motor.SetSetpoint(ExampleSmartMotorController::PIDMode::kPosition,
    //                     m_setpoint.position.value(),
    //                     m_feedforward.Calculate(m_setpoint.velocity) / 12_V);
    m_pidController.SetReference(m_setpoint.position.value(), rev::ControlType::kPosition, 0,  m_feedforward.Calculate(m_setpoint.velocity) / 12_V);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetVelocity());
    
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
