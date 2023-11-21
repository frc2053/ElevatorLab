// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return elevatorSubsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&elevatorSubsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(elevatorSubsystem.ExampleMethodCommand());

  m_driverController.A().OnTrue(elevatorSubsystem.GoToHeightCommand([] {
    return 30_in;
  }));

  m_driverController.X().OnTrue(elevatorSubsystem.GoToHeightCommand([] {
    return 0_in;
  }));

  m_driverController.Y().OnTrue(elevatorSubsystem.GoToHeightCommand2([] {return 45_in;}, []{return 15_in;}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&elevatorSubsystem);
}
