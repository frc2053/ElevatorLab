// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/sendable/Sendable.h>

struct ElevatorGains
{
  double kP{0.0};
  double kI{0.0};
  double kD{0.0};
  double kV{0.0};
  double kA{0.0};
  double kS{0.0};
  double kG{0.0};
};

class ElevatorSubsystem : public frc2::SubsystemBase
{
public:
  ElevatorSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void GoToHeight(units::meter_t height);
  units::meter_t GetCurrentHeight();

private:
  void ConfigureMotors();
  void InitSendable(wpi::SendableBuilder &builder) override;
  void SetGains(const ElevatorGains &newGains);
  ElevatorGains GetGains();

  ctre::phoenix6::hardware::TalonFX elevatorLeftMotor{ElevatorConstants::leftMotorCANId};
  ctre::phoenix6::hardware::TalonFX elevatorRightMotor{ElevatorConstants::rightMotorCANId};

  ctre::phoenix6::controls::MotionMagicVoltage positionControl{0_deg, true, 0_V, 0, false};
  ctre::phoenix6::StatusSignal<units::turn_t> elevatorPositionSignal{elevatorLeftMotor.GetPosition()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> elevatorVelocitySignal{elevatorLeftMotor.GetVelocity()};

  units::meter_t currentSetpoint{0};
  units::meter_t currentPosition{0};
  units::meters_per_second_t currentVelocity{0};

  ElevatorGains currentGains{};
};
