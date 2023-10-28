# Command Based Framework

The command based framework is a style of writing robot code to allow us to easily chain together actions we want our robot to do.

Think about this. In a regular program, you would need a big loop if you want to have your program respond to joysticks or sensor more than once, or else as soon as the code was done the program would end. 

For example:

```cpp
while(robotIsEnabled) {
    double joystickValue = joystick.GetLeftX();
    motor.Set(joystickValue);
    //blah blah blah
}
```

This can work for small programs like how I demoed the motor moving program, but imagine you had 10 motors all doing different things depending on sensors, driver input and more conditions. This giant loop would become a giant mess, and keeping track of all of your variables state would make fixing bugs super difficult!

This is where the command based framework comes in. It allows us to split up our robot functionality into "Subsystems" and "Commands".

You can think of a subsystem as single part of our robot, for example, an elevator or drivetrain or intake. You can think of each subsystem being able to do things completely independently from eachother. Each subsystem can only run one "Command" at a time. 

Commands are pieces of code that act on one or more subsystems. They represet actions a robot can take. For example you could have a command called "MoveElevatorToTop" which would use the Elevator subsystem, and the command would run until the elevator on the robot reached the maximum height. 

If you want more information about the command based system check out [this](https://docs.wpilib.org/en/latest/docs/software/commandbased/what-is-command-based.html) link.

## Looking through code
Take a quick look through the .h and .cpp files in the project. You can see there are folders for commands and subsystems.

Here are some key things to take note:

Any function name that ends in "Periodic" means that function will run every 20 milliseconds over and over until the robot program command ends.

Any function name that ends in "Init" means that the function will run before the Periodics function is called.

For example, in Robot.cpp, you can see AutonomousInit() which will run once when you change the Robot State to Autonomous and then AutonomousPeriodic() will start to run every 20 milliseconds, until you disable the robot or change to Teleop mode.

## Enough of the boring stuff I wanna code!
Ok lets get started coding our elevator! [CLICK ME TO START](subsystem.md)