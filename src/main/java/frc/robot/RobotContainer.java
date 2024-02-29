
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import javax.naming.OperationNotSupportedException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.elevator.elevatorPositionNoPID;
import frc.robot.commands.intakeShooter.ShootAtRPM;
//import frc.robot.commands.Intake.IntakeCommand;
//import frc.robot.commands.elevator.ElevatorCommand;
//import frc.robot.commands.elevator.ElevatorPosSet;
import frc.robot.commands.swerve.driveCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
//import frc.robot.commands.wrist.WristCommand;
//import frc.robot.commands.wrist.WristPosSet;
//import frc.robot.subsystems.elevator.ElevatorModule;
//import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.SwerveDrive;
//import frc.robot.subsystems.wrist.WristModule;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_robotDrive = new SwerveDrive();
  private final Elevator m_elevator = new Elevator();
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  //private final Test test = new Test();

  // LED for indicating robot state, not implemented in hardware.

  // private final IntakeModule m_intakeModule = new IntakeModule();
  // private final ElevatorModule m_elevatorModule = new ElevatorModule();
  // private final WristModule m_wristModule = new WristModule();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(PortConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);
  
  // private final JoystickButton yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton DriverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton DriverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton DriverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton DriverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);

  private final POVButton DriverDPadUp = new POVButton(m_driverController, 0);
  private final POVButton DriverDPadRight = new POVButton(m_driverController, 90);
  private final POVButton DriverDPadDown = new POVButton(m_driverController, 180);
  private final POVButton DriverDPadLeft = new POVButton(m_driverController, 270);
  
  private final JoystickButton DriverRightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton DriverLeftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);


  private final JoystickButton OperatorAButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton OperatorBButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton OperatorXButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  private final JoystickButton OperatorYButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);

  private final POVButton OperatorDPadUp = new POVButton(m_operatorController, 0);
  private final POVButton OperatorDPadRight = new POVButton(m_operatorController, 90);  
  private final POVButton OperatorDPadDown = new POVButton(m_operatorController, 180);
  private final POVButton OperatorDPadLeft = new POVButton(m_operatorController, 270);

    private final JoystickButton OperatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
  private final JoystickButton OperatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);



  // private final JoystickButton y2Button = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  // private final JoystickButton a2Button = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  // private final JoystickButton b2Button = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  // private final JoystickButton x2Button = new JoystickButton(m_operatorController, XboxController.Button.kX.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    m_IntakeShooter.setDefaultCommand(new InstantCommand(() -> m_IntakeShooter.setPivotPower((Math.abs(m_operatorController.getLeftY())) > 0.1 ? m_operatorController.getLeftY() : 0.00), m_IntakeShooter));
    
    //test.setDefaultCommand(new testCommand(test, m_driverController));
    NamedCommands.registerCommand("Shoot", new ShootAtRPM(m_IntakeShooter, 3000, 500));

  }

  /*
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /*new JoystickButton(m_driverController, Button.kR1.value) // if R1 is pressed wheels should go into x formation
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX()0,
            m_robotDrive));*/
   
    // review button mapping
    DriverAButton.whileTrue(new InstantCommand(
            () -> m_robotDrive.resetGyro(),
            m_robotDrive));

    // reference for future command mapping

    DriverDPadUp.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(1.0)))
    .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0)));
    DriverDPadDown.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(-1)))
    .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0)));
    DriverDPadLeft.onTrue(new InstantCommand(() -> m_elevator.setPivotPower(-0.25)))
    .onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0)));
    DriverDPadRight.onTrue(new InstantCommand(() -> m_elevator.setPivotPower(0.25)))
    .onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0)));
    
    

    Trigger operatorRightY = new Trigger(() -> m_operatorController.getRightY() > 0.10);
    operatorRightY.whileTrue(new InstantCommand(() -> m_elevator.setElevatorPower(m_operatorController.getRightY()))).onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0)));

    Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.75);
    Trigger operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.75);


    operatorLeftTrigger.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(1), m_IntakeShooter)).onFalse(new InstantCommand(() -> m_IntakeShooter.setRollerPower(0), m_IntakeShooter));
    operatorRightTrigger.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(-1), m_IntakeShooter)).onFalse(new InstantCommand(() -> m_IntakeShooter.setRollerPower(0), m_IntakeShooter));
    
    //Operator Intake
    OperatorBButton.onTrue(new InstantCommand(() -> m_IntakeShooter.setStowPower(1), m_IntakeShooter)).onFalse(new InstantCommand(() -> m_IntakeShooter.setStowPower(0), m_IntakeShooter));
    //Operator Outtake
    OperatorXButton.onTrue(new InstantCommand(() -> m_IntakeShooter.setStowPower(-1), m_IntakeShooter)).onFalse(new InstantCommand(() -> m_IntakeShooter.setStowPower(0), m_IntakeShooter));

    OperatorAButton.onTrue(new elevatorPositionNoPID(m_elevator, ElevatorPositions.AMP));



    DriverLeftBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(-0.85)))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0)));
    DriverRightBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(0.85)))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0)));

    // aButton.onTrue(new ElevatorPosSet(m_elevatorModule, "cube_pickup")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));
    // yButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cone_high/cube_mid")));
    // bButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));
    // xButton.onTrue(new ElevatorPosSet(m_elevatorModule, "elevator_init")
    //           .andThen(new WristPosSet(m_wristModule, "cube_high")));


    //a2Button.onTrue(new ElevatorPosSet(m_elevatorModule, "home/low_cube")
    //           .andThen(new WristPosSet(m_wristModule, "home/low_cube")));  

    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("NewAutoy");
  }
}