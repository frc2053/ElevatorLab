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
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/Commands.h>
#include <units/time.h>

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

  bool IsElevatorAtSetpoint();

  frc2::CommandPtr GoToHeightCommand(std::function<units::meter_t()> height) {
    return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([height, this] {
        GoToHeight(height());
      }, {this}),
      frc2::cmd::WaitUntil([this] {
        return IsElevatorAtSetpoint();
      })
    );
  };

  frc2::CommandPtr GoToHeightCommand2(std::function<units::meter_t()> height, std::function<units::meter_t()> height2) {
    return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([height, this] {
        GoToHeight(height());
      }, {this}),
      frc2::cmd::WaitUntil([this] {
        return IsElevatorAtSetpoint();
      }),
      frc2::cmd::Wait(2_s), 
      frc2::cmd::RunOnce([height2, this] {
        GoToHeight(height2());
      }, {this}),
      frc2::cmd::WaitUntil([this] {
        return IsElevatorAtSetpoint();
      }),
      frc2::cmd::Wait(2_s)
    );
  };

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

  // Create a Mechanism2d display of an elevator
  frc::Mechanism2d mech2d{10_in / 1_m, 73_in / 1_m};
  frc::MechanismRoot2d *elevatorRoot =
      mech2d.GetRoot("Elevator Root", 5_in / 1_m, 0.0_in / 1_m);
  frc::MechanismLigament2d *elevatorMech2d =
      elevatorRoot->Append<frc::MechanismLigament2d>(
          "Elevator", elevatorSim.GetPosition().value(), 90_deg);

  
};
