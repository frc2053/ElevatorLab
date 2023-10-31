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
  elevatorSimState.SetRawRotorPosition(ConvertElevatorPositionToMotorPosition(elevatorSim.GetPosition()));
  elevatorSimState.SetRotorVelocity(ConvertElevatorVelToMotorVel(elevatorSim.GetVelocity()));
```

Finally, lets add a little visualizer so we arent just look at numbers on the dashboard

```cpp
// Add these variables to the private member variables in ElevatorSubsystem.h

// Create a Mechanism2d display of an elevator
frc::Mechanism2d mech2d{10_in / 1_m, 73_in / 1_m};
frc::MechanismRoot2d* elevatorRoot =
    mech2d.GetRoot("Elevator Root", 5_in / 1_m, 0.5_in / 1_m);
frc::MechanismLigament2d* elevatorMech2d =
    elevatorRoot->Append<frc::MechanismLigament2d>(
        "Elevator", elevatorSim.GetPosition().value(), 90_deg);
```

In the constructor we can put the visualizer on the dashboard..

```cpp
ElevatorSubsystem::ElevatorSubsystem()
{
  //....
  frc::SmartDashboard::PutData("Elevator Vis", &mech2d);
}
```

Then in the periodic function we need to update the length of our elevator vis.. 

```cpp
void ElevatorSubsystem::Periodic()
{
  //....
  elevatorMech2d->SetLength(currentPosition.value());
}
```

Finally, lets try out the complete simulation!

https://github.com/frc2053/ElevatorLab/assets/6174102/a7d08376-8fda-405f-a6ff-482984865d1f

Mess with the gains and see how that effects the simulation!


