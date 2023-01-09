2023SwerveSim
This repository contains code examples on how to implement a working swerve sim using the 2023 WPILib libraries and the CTRE/Rev libraries. This can operate the robot using both Autonomous and Teleop Code. The only limitation this model has is that you cannot define a trajectory that utilizes a moving heading for the swerve bot to use (e.g. strafe/orbit a target). Otherwise, for most teams attempting to develop swerve code, this should cover 95% of their needs.

More details on how this was originally developed can be found in the [ChiefDelphi post](https://www.chiefdelphi.com/t/simulating-a-swerve-drive-with-the-2021-wpilib-libraries/393534) we made.

## 2023SwerveControllerCommand
This is an example template that uses the WPILib SwerveControllerCommand as the baseline to implement a swerve sim. This tries to make as little changes as possible while also following the WPILib style of making the code as generic as possible to make it easier to adapt for teams using hardware/libraries from different vendors.

## 2023FalconSwerve
This is an example template that uses the 2023 CTRE Tuner 5 (Non-pro) libraries to implement a swerve sim.

## 2023RevSwerve
This is an example template that uses a mix of the 2023 RevLib libraries for NEO Motors and the CTRE Tuner 5 (Non-pro) libraries to implement the CANCoders/Pigeon2. Note that the RevLib sim doesn't work completely, so we implemented a workaround to ensure that the code logic is working properly.

### Swerve Hardware Specs
* SDS MKIV Modules with Falcon 500/Neo motors
* Using L3 configuration geared for 16 ft/s (NEO) or 18 ft/s (Falcon)  (6.12:1)
* Steering Gear ratio is 12.8:1 (See Patrick's post on [ChiefDelphi](https://www.chiefdelphi.com/t/sds-mk3-swerve-module/388331/60)
* Assumes you are using CTRE CANCoders to reset the module absolute position and a CTRE Pigeon2 as your gyroscope.
