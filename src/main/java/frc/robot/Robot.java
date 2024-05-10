// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The main Robot class that extends TimedRobot. It is the central hub for the robot's operations.
 * This class is responsible for initializing all robot subsystems and commands, as well as handling
 * the different robot states (disabled, autonomous, teleop, etc.).
 */
public class Robot extends TimedRobot {
  // The command to run in autonomous mode
  private Command autonomousCommand;

  // Container for robot subsystems and commands
  private RobotContainer robotContainer;

  /**
   * This method is called when the robot is first started up and initializes the robot components.
   */
  @Override
  public void robotInit() {
    // Instantiate the RobotContainer. This will perform all our button bindings and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.elevator.resetEncoder();
    robotContainer.robotDrive.resetGyro();

    // Initialize the camera server for vision processing
    //CameraServer.startAutomaticCapture();
  }

  /**
   * This method is called every robot packet, no matter the mode. It is used for items like
   * diagnostics that you want run during disabled, autonomous, teleoperated, and test modes.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This method is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This method is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.robotDrive.resetGyro();

    // Schedule the autonomous command if it is present
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This method is called periodically during Autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {}

  /**
   * This method is called once each time the robot enters Teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This method is called periodically during Teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {}

  /**
   * This method is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This method is called periodically during Test mode.
   */
  @Override
  public void testPeriodic() {}
}
