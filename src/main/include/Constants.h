// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/mass.h>
#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants
{

    constexpr int kDriverControllerPort = 0;

} // namespace OperatorConstants

namespace ElevatorConstants
{
    static constexpr int leftMotorCANId{2};
    static constexpr int rightMotorCANId{3};
    static constexpr double elevatorGearRatio{5.5};
    static constexpr units::kilogram_t elevatorCarriageMass{25_lb};
    static constexpr units::meter_t elevatorDrumRadius{1_in};
    static constexpr units::meter_t elevatorMinHeight{0_ft};
    static constexpr units::meter_t elevatorMaxHeight{6_ft};
}
