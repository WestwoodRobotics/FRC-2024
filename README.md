# FRC-2024 Project Overview

Welcome to the FRC-2024 project repository! This project is designed to provide a comprehensive software solution for our competitive robot, incorporating advanced control systems, vision processing, strategic gameplay functionalities, and utilizing the command-based framework for organizing robot operations.

## Project Structure

The project is structured into several directories, each serving a specific purpose in the robot's software architecture:

- `commands`: Contains command classes that define operations for the robot.
- `subsystems`: Houses subsystem classes that represent different parts of the robot, such as the elevator and intake shooter.
- `utils`: Includes utility classes and enums for various functionalities.
- `vision`: Contains classes related to vision processing and interaction with the Limelight camera.

## Setup Instructions

To get started with the FRC-2024 project, follow these steps:

1. Clone the repository to your local machine.
2. Install the required software dependencies, including WPILib and vendor libraries.
3. Configure your development environment according to the WPILib documentation.
4. Build the project using Gradle to verify the setup.

For comprehensive documentation on WPILib and the command-based framework, refer to [WPILib Documentation](https://docs.wpilib.org/en/stable/index.html).

## Contribution Guidelines

We welcome contributions from all team members! To contribute, please adhere to our coding standards and review process, detailed in [CONTRIBUTING.md](CONTRIBUTING.md).

## Dependencies

This project relies on several external libraries and tools:

- WPILib Suite
- PathPlanner for autonomous path planning
- REVLib for interfacing with REV Hardware

Refer to the `build.gradle` file for specific version requirements.

## Tools and Technologies

- **SmartDashboard/Shuffleboard**: Used for real-time data visualization and control interface.
- **Limelight Camera**: Provides vision processing capabilities for target detection and alignment.
- **PathPlanner**: Path visualization software for autonomous path planning.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for more information.
