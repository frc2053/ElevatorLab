# Implementing our Elevator Subsystem Part 2

Because we want to control how the users of the ElevatorSubsystem interface with our motors, we need to write some functions that make it easy for the Command. Lets make a quick list of what might be useful as a Command to be able to do: 

- We need a way to tell the elevator to go to certain height
- We want to be able to get the height of the elevator at any time

Thats about it :) Lets think about what our subsystem might have to make debugging easier for our future selves.

- We want a function to configure the motors on robot startup
- We want to be able to simulate the elevator motors without the user knowing the difference between the two
- We want a way to visualize the elevator state on the simulator and dashboard

Lets declare these functions in the h file then work on implementing each one.

In ElevatorSubsystem.h add these functions and class variables
```cpp
// under the public section...
  void GoToHeight(units::meter_t height);
  units::meter_t GetCurrentHeight();
// under the private section...  
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

  ctre::phoenix6::controls::MotionMagicVoltage positionControl{0_deg, true, 0_V, 0, false};
  ctre::phoenix6::StatusSignal<units::turn_t> elevatorPositionSignal{elevatorLeftMotor.GetPosition()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> elevatorVelocitySignal{elevatorLeftMotor.GetVelocity()};

  //Lets keep track of these for when we want to output them 
  //on the dashboard
  units::meter_t currentSetpoint{0};
  units::meter_t currentPosition{0};
  units::meters_per_second_t currentVelocity{0};
```

You don't have to worry too much about the signal and control variables for now, we are just going to use them to get the position and velocity of the motors.

You'll also have to put `#include <units/length.h>` and `#include <units/velocity.h>` at the top of the file as we are going to use the units::meter_t type. This is a super cool library that lets us use and and convert units in a safe way. For example you can pass in 2_ft (2 feet) into GoToHeight(units::meter_t height) and it will automatically convert 2 feet into meters. Less math mistakes is always good!

Ok lets move to our cpp file and actually write these functions.

Lets do the motor configuration first. Don't worry about a lot of this stuff as it is specific to the Phoenix library and not important for this lesson. 

Dont forget to add this to your constants file in the elevator namespace!

```cpp
static constexpr double elevatorGearRatio{5.5};
```

In ElevatorSubsystem.cpp

```cpp
ElevatorSubsystem::ElevatorSubsystem() {
  // Add me!
  ConfigureMotors();
  // This will add myself to the dashboard
  frc::SmartDashboard::PutData("Elevator Telemetry", this);
}

void ElevatorSubsystem::ConfigureMotors()
{
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  mainConfig.Slot0.kP = kP;
  mainConfig.Slot0.kI = kI;
  mainConfig.Slot0.kD = kD;
  mainConfig.Slot0.kV = kV;
  mainConfig.Slot0.kA = kA;
  mainConfig.Slot0.kS = kS;
  mainConfig.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  mainConfig.Slot0.kG = kG;

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
```

Next, lets write our functions to get and set the height of our elevator. Since we set up the gear ratio of our elevator before, it should be trival. 

Read through the comments for more details!

```cpp
void ElevatorSubsystem::Periodic()
{
  // Refresh our signals from the motor controller
  ctre::phoenix6::BaseStatusSignal::RefreshAll(elevatorPositionSignal, elevatorVelocitySignal);

  units::turn_t motorPostion = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(elevatorPositionSignal, elevatorVelocitySignal);

  currentPosition = ConvertMotorPositionToElevatorPositon(motorPostion);
  currentVelocity = ConvertMotorVelToElevatorVel(elevatorVelocitySignal.GetValue());
}

void ElevatorSubsystem::GoToHeight(units::meter_t height)
{
  currentSetpoint = height;
  // This "withX" style of code is called the builder pattern.
  units::radian_t motorSetpoint = ConvertElevatorPositionToMotorPosition(height);
  elevatorLeftMotor.SetControl(positionControl.WithPosition(motorSetpoint).WithEnableFOC(true).WithSlot(0););
}

units::meter_t ElevatorSubsystem::GetCurrentHeight()
{
  return currentPosition;
}
```

Ok that wasn't too bad. Lets add our final couple functions for now, and add some logging so we can see the state of the elevator on the SmartDashboard. Lets write a quick struct to hold all of our PID gains to make it easy to keep track of what they are

```cpp
// Before the class in ElevatorSubsystem.h...
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
//In private area of ElevatorSubsytem...
ElevatorGains currentGains{};
```

```cpp
// And some helper functions... 
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
      { return currentPosition.convert<units::feet>().value(); },
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
```

This is a super long function that basically logs some variables to the dashboard. Lets take a look at once section of it with some expanded formatting..

```cpp
  builder.AddDoubleProperty(
    "Position Setpoint (ft)",
    [this] { 
      return currentSetpoint.convert<units::feet>().value(); 
    },
    [this](double newSetpointFt) { 
      GoToHeight(units::foot_t{newSetpointFt}); 
    }
  );
```
This AddDoubleProperty function looks kinda weird right? Well thats becase the 2nd and 3rd parameters are actually lambda functions. 

A lambda function is basically an "inline function" that doesnt have a name. The uses for these will become apparent if you think about what AddDoubleProperty is doing. 

AddDoubleProperty basically says:

Here is a key being the string parameter, and I want something I can CALL to get the data to put on the dashboard and something I can CALL to set the data in the robot program.

If I just passed in a variable instead of a function, the variable wouldn't be able to be updated easily.

This might seem convoluted for now, but will click eventually. 

Next steps is adding some actually physics simulation. Take a snack break and move onto the next step: [CLICK HERE](simulation.md)