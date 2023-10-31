# Simulating some motors using PHYSICS

To simulate our elevator mechanism, we can actually use wpilib's built in ElevatorSim class. So lets add it to our subsystem.

Add the below variables to our constants file in the elevator namespace. These define the physical characteristics of our fake elevator.

```cpp
// Constants.h
static constexpr units::kilogram_t elevatorCarriageMass{25_lb};
static constexpr units::meter_t elevatorDrumRadius{1_in};
static constexpr units::meter_t elevatorMinHeight{0_ft};
static constexpr units::meter_t elevatorMaxHeight{6_ft};
```

We need a DCMotor object as well to pass into our sim object. This describes the physical characteristics of our motors we are using.

```cpp
// top of ElevatorSubsystem.h
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
private:
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

```

Lets add this code to the SimulationPeriodic function in ElevatorSubsystem.cpp

Take a look through the comments to understand what is going on.

```cpp
// Our elevator simulator takes in the applied motor voltage as input in a 1x1 matrix
elevatorSim.SetInput(frc::Vectord<1>{elevatorSimState.GetMotorVoltage().value()});
// Advance the simulation by 20 milliseconds. This is the same update rate as the periodic functions.
elevatorSim.Update(20_ms);
// Finally, update our simulated falcons encoders from the sim.
elevatorSimState.SetRawRotorPosition(units::turn_t{elevatorSim.GetPosition().value()});
elevatorSimState.SetRotorVelocity(units::turns_per_second_t{elevatorSim.GetVelocity().value()});
```