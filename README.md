# FRC-2024 Project Overview

## Project Structure

The project is structured into several directories, each serving a specific purpose in the robot's software architecture:

- `commands`: Contains command classes that define operations for the robot.
- `subsystems`: Houses subsystem classes that represent different parts of the robot, such as the elevator and intake shooter.
- `utils`: Includes utility classes and enums for various functionalities.
- `vision`: Contains classes related to vision processing and interaction with the Limelight camera.

## Commands Directory

### `elevatorPosition.java`
Controls the elevator's position using a PID controller.

### `elevatorRollerCommand.java`
Manages the elevator's roller speed based on beam break sensor input.

### `IntakeCommandFactory.java`
Factory class for creating intake and shooter commands.

### `IntakeRollersCommand.java`
Sets the power for the intake rollers and stow motors.

### `IntakeShooterPosition.java`
Positions the intake shooter pivot using a PID controller.

### `IntakeShooterPositionTimeOut.java`
Similar to `IntakeShooterPosition` but with a timeout feature.

### `DriveAlignAndRangeVisionCommand.java`
Aligns and ranges the robot using vision data.

### `driveAlignVisionCommand.java`
Aligns the robot with a vision target.

## Subsystems Directory

### `Elevator.java`
Represents the elevator subsystem, including motors and sensors.

### `IntakePivot.java`
Controls the pivot mechanism of the intake shooter.

### `IntakeRollers.java`
Manages the rollers for intaking and shooting balls.

### `SwerveDrive.java`
Implements the swerve drive functionality for the robot.

### `Vision.java`
Handles vision processing and interaction with the Limelight camera.

## Tools and Technologies

- **SmartDashboard/Shuffleboard**: Used for real-time data visualization and control interface.
- **Limelight Camera**: Provides vision processing capabilities for target detection and alignment.

