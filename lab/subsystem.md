# Implementing our ElevatorSubsystem

First thing we want to do is import the library to control the motor controllers we want to use. Since we are going to use [Falcon500's](https://www.vexrobotics.com/pro/falcon-500), we need to use the Phoenix library. 

![Falcon 500](img/FalconV3.avif)

To add this library to our project:
    1. Ctrl+Shift+P and search for "Manage Vendor Libraries" and click enter.
    2. Then arrow down until you get to "Install new libraries (online)" and click enter.
    3. Paste this url into the text box https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-alpha-latest.json and click enter
    4. This will place a file into the vendordeps folder and 