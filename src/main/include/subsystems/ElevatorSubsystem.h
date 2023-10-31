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
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/XboxController.h>

struct ElevatorGains
{
  double kP{10.0};
  double kI{0.0};
  double kD{0.0};
  double kV{3.0};
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

  units::meter_t ConvertMotorPositionToElevatorPositon(units::radian_t position);
  units::meters_per_second_t ConvertMotorVelToElevatorVel(units::radians_per_second_t vel);

  units::radian_t ConvertElevatorPositionToMotorPosition(units::meter_t position);
  units::radians_per_second_t ConvertElevatorVelToMotorVel(units::meters_per_second_t vel);

  ctre::phoenix6::hardware::TalonFX elevatorLeftMotor{ElevatorConstants::leftMotorCANId};
  ctre::phoenix6::hardware::TalonFX elevatorRightMotor{ElevatorConstants::rightMotorCANId};

  ctre::phoenix6::controls::PositionVoltage positionControl{0_rad, 0_rad_per_s, true, 0_V, 0, false};
  ctre::phoenix6::StatusSignal<units::turn_t> elevatorPositionSignal{elevatorLeftMotor.GetPosition()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> elevatorVelocitySignal{elevatorLeftMotor.GetVelocity()};

  units::meter_t currentSetpoint{0};
  units::meter_t currentPosition{0};
  units::meters_per_second_t currentVelocity{0};

  ElevatorGains currentGains{};

  frc::DCMotor elevatorGearbox{frc::DCMotor::Falcon500FOC(2)};
  frc::sim::ElevatorSim elevatorSim{
      elevatorGearbox,
      ElevatorConstants::elevatorGearRatio,
      ElevatorConstants::elevatorCarriageMass,
      ElevatorConstants::elevatorDrumRadius,
      ElevatorConstants::elevatorMinHeight,
      ElevatorConstants::elevatorMaxHeight,
      true,
      0_m,
      {0.005}};

  ctre::phoenix6::sim::TalonFXSimState &elevatorSimState{elevatorLeftMotor.GetSimState()};

  frc::XboxController joystick{1};
};
