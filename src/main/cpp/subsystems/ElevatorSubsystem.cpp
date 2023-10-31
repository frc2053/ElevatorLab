// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorSubsystem::ElevatorSubsystem()
{
  ConfigureMotors();
  frc::SmartDashboard::PutData("Elevator Telemetry", this);
  frc::SmartDashboard::PutData("Elevator Vis", &mech2d);
}

frc2::CommandPtr ElevatorSubsystem::ExampleMethodCommand()
{
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ElevatorSubsystem::ExampleCondition()
{
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ElevatorSubsystem::Periodic()
{
  // Refresh our signals from the motor controller
  ctre::phoenix6::BaseStatusSignal::RefreshAll(elevatorPositionSignal, elevatorVelocitySignal);

  units::turn_t motorPostion = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(elevatorPositionSignal, elevatorVelocitySignal);

  currentPosition = ConvertMotorPositionToElevatorPositon(motorPostion);
  currentVelocity = ConvertMotorVelToElevatorVel(elevatorVelocitySignal.GetValue());

  elevatorMech2d->SetLength(currentPosition.value());
}

void ElevatorSubsystem::SimulationPeriodic()
{
  // Our elevator simulator takes in the applied motor voltage as input in a 1x1 matrix
  elevatorSim.SetInput(frc::Vectord<1>{elevatorSimState.GetMotorVoltage().value()});
  // Advance the simulation by 20 milliseconds. This is the same update rate as the periodic functions.
  elevatorSim.Update(20_ms);
  // Finally, update our simulated falcons encoders from the sim.
  elevatorSimState.SetRawRotorPosition(ConvertElevatorPositionToMotorPosition(elevatorSim.GetPosition()));
  elevatorSimState.SetRotorVelocity(ConvertElevatorVelToMotorVel(elevatorSim.GetVelocity()));
}

void ElevatorSubsystem::ConfigureMotors()
{
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  mainConfig.Slot0.kP = currentGains.kP;
  mainConfig.Slot0.kI = currentGains.kI;
  mainConfig.Slot0.kD = currentGains.kD;
  mainConfig.Slot0.kV = currentGains.kV;
  mainConfig.Slot0.kA = currentGains.kA;
  mainConfig.Slot0.kS = currentGains.kS;
  mainConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  mainConfig.Slot0.kG = currentGains.kG;

  elevatorLeftMotor.GetConfigurator().Apply(mainConfig);
  elevatorRightMotor.GetConfigurator().Apply(mainConfig);

  // Because the other elevator motor is facing the opposite direction in our imaginary
  // elevator, we want to make sure it always follows the left motor but in the opposite direction
  elevatorRightMotor.SetControl(ctre::phoenix6::controls::Follower{elevatorLeftMotor.GetDeviceID(), true});

  // Super fast refresh rate!
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(1000_Hz, elevatorPositionSignal, elevatorVelocitySignal);
  // Disable all other signals we dont care about
  elevatorLeftMotor.OptimizeBusUtilization();
  elevatorRightMotor.OptimizeBusUtilization();
}

void ElevatorSubsystem::GoToHeight(units::meter_t height)
{
  currentSetpoint = height;
  // This "withX" style of code is called the builder pattern.
  units::radian_t motorSetpoint = ConvertElevatorPositionToMotorPosition(height);
  elevatorLeftMotor.SetControl(positionControl.WithPosition(motorSetpoint).WithEnableFOC(true).WithSlot(0));
}

units::meter_t ElevatorSubsystem::GetCurrentHeight()
{
  return currentPosition;
}

units::meter_t ElevatorSubsystem::ConvertMotorPositionToElevatorPositon(units::radian_t position)
{
  units::radian_t drumRadialPosition = position / ElevatorConstants::elevatorGearRatio;
  return units::meter_t{drumRadialPosition.value() * ElevatorConstants::elevatorDrumRadius};
}

units::meters_per_second_t ElevatorSubsystem::ConvertMotorVelToElevatorVel(units::radians_per_second_t vel)
{
  units::radians_per_second_t drumRadialVelocity = vel / ElevatorConstants::elevatorGearRatio;
  return units::meters_per_second_t{drumRadialVelocity.value() * ElevatorConstants::elevatorDrumRadius.value()};
}

units::radian_t ElevatorSubsystem::ConvertElevatorPositionToMotorPosition(units::meter_t position)
{
  units::radian_t drumRadialPosition{position.value() / ElevatorConstants::elevatorDrumRadius.value()};
  return drumRadialPosition * ElevatorConstants::elevatorGearRatio;
}

units::radians_per_second_t ElevatorSubsystem::ConvertElevatorVelToMotorVel(units::meters_per_second_t vel)
{
  units::radians_per_second_t drumRadialVel{vel.value() / ElevatorConstants::elevatorDrumRadius.value()};
  return drumRadialVel * ElevatorConstants::elevatorGearRatio;
}

void ElevatorSubsystem::SetGains(const ElevatorGains &newGains)
{
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kP = currentGains.kP;
  newConfig.kI = currentGains.kI;
  newConfig.kD = currentGains.kD;
  newConfig.kV = currentGains.kV;
  newConfig.kA = currentGains.kA;
  newConfig.kS = currentGains.kS;
  newConfig.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  newConfig.kG = currentGains.kG;
  elevatorLeftMotor.GetConfigurator().Apply(newConfig);
}

ElevatorGains ElevatorSubsystem::GetGains()
{
  return currentGains;
}

void ElevatorSubsystem::InitSendable(wpi::SendableBuilder &builder)
{
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Position Setpoint (ft)",
      [this]
      { return currentSetpoint.convert<units::feet>().value(); },
      [this](double newSetpointFt)
      { GoToHeight(units::foot_t{newSetpointFt}); });
  builder.AddDoubleProperty(
      "Current Position (ft)",
      [this]
      { return GetCurrentHeight().convert<units::feet>().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Current Velocity (ft per sec)",
      [this]
      { return currentVelocity.convert<units::feet_per_second>().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this]
      { return currentGains.kP; },
      [this](double newKp)
      {
        ElevatorGains newGains = GetGains();
        newGains.kP = newKp;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this]
      { return currentGains.kI; },
      [this](double newKi)
      {
        ElevatorGains newGains = GetGains();
        newGains.kI = newKi;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this]
      { return currentGains.kD; },
      [this](double newKd)
      {
        ElevatorGains newGains = GetGains();
        newGains.kD = newKd;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this]
      { return currentGains.kV; },
      [this](double newKv)
      {
        ElevatorGains newGains = GetGains();
        newGains.kV = newKv;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this]
      { return currentGains.kA; },
      [this](double newKa)
      {
        ElevatorGains newGains = GetGains();
        newGains.kA = newKa;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this]
      { return currentGains.kS; },
      [this](double newKs)
      {
        ElevatorGains newGains = GetGains();
        newGains.kS = newKs;
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kG", [this]
      { return currentGains.kG; },
      [this](double newKg)
      {
        ElevatorGains newGains = GetGains();
        newGains.kG = newKg;
        SetGains(newGains);
      });
}