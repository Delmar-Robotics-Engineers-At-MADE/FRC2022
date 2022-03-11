// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "ExampleSmartMotorController.h"
#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {
 public:
  static constexpr units::second_t kDt = 20_ms;

  rev::SparkMaxPIDController m_pidController = m_motor.GetPIDController();
  rev::SparkMaxRelativeEncoder m_encoder = m_motor.GetEncoder();

  // default PID coefficients
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  Robot() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    // m_motor.SetPID(1.3, 0.0, 0.7);
  }

  void RobotInit() override 
  {
    // following example
    m_motor.RestoreFactoryDefaults();
    
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
  }

  void TeleopPeriodic() override {
    if (m_joystick.GetRawButtonPressed(2)) {
      m_goal = {5_m, 0_mps};
    } else if (m_joystick.GetRawButtonPressed(3)) {
      m_goal = {0_m, 0_mps};
    }

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    frc::TrapezoidProfile<units::meters> profile{m_constraints, m_goal,
                                                 m_setpoint};

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.Calculate(kDt);

    // Send setpoint to offboard controller PID
    //m_motor.SetSetpoint(ExampleSmartMotorController::PIDMode::kPosition,
    //                    m_setpoint.position.value(),
    //                    m_feedforward.Calculate(m_setpoint.velocity) / 12_V);
    m_pidController.SetReference(100, rev::ControlType::kVelocity);
  }

 private:
  frc::Joystick m_joystick{1};
  rev::CANSparkMax m_motor{62, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot.
      1_V, 1.5_V * 1_s / 1_m};

  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{1.75_mps,
                                                                  0.75_mps_sq};
  frc::TrapezoidProfile<units::meters>::State m_goal;
  frc::TrapezoidProfile<units::meters>::State m_setpoint;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
