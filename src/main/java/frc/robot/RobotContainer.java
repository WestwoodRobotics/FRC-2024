
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import javax.naming.OperationNotSupportedException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.elevator.elevatorPosition;
import frc.robot.commands.intakeShooter.IntakeBeamBreakCommand;
import frc.robot.commands.intakeShooter.IntakeCommand;
import frc.robot.commands.intakeShooter.IntakeShooterPosition;
import frc.robot.commands.intakeShooter.ShootAtRPM;
import frc.robot.commands.intakeShooter.ShootForTimeCommand;
//import frc.robot.commands.Intake.IntakeCommand;
//import frc.robot.commands.elevator.ElevatorCommand;
//import frc.robot.commands.elevator.ElevatorPosSet;
import frc.robot.commands.swerve.driveCommand;
import frc.robot.commands.utils.StopAllRollersCommand;
import frc.robot.commands.vision.LEDCommand;
import frc.robot.subsystems.vision.BeamBreak;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
//import frc.robot.commands.wrist.WristCommand;
//import frc.robot.commands.wrist.WristPosSet;
//import frc.robot.subsystems.elevator.ElevatorModule;
//import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.SwerveDrive;
//import frc.robot.subsystems.wrist.WristModule;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LED;
import frc.robot.subsystems.vision.LimitSwitch;
import pabeles.concurrency.ConcurrencyOps.NewInstance;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveDrive m_robotDrive = new SwerveDrive();
  public final Elevator m_elevator = new Elevator();
  //private LED led = new LED(9);
  private BeamBreak intakeShooterBeamBreak = new BeamBreak(9);
  private BeamBreak elevatorPivotBeamBreak = new BeamBreak(8);
  private LimitSwitch limitSwitch = new LimitSwitch(6);

  private final IntakeShooter m_IntakeShooter = new IntakeShooter(limitSwitch);
  
  

  // LED for indicating robot state, not implemented in hardware.

  // The driver's controller
  XboxController m_driverController = new XboxController(PortConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);

  private final JoystickButton DriverAButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton DriverBButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton DriverXButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton DriverYButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton DriverGyroButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton OperatorStartButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);


  private final POVButton DriverDPadUp = new POVButton(m_driverController, 0);
  private final POVButton DriverDPadRight = new POVButton(m_driverController, 90);
  private final POVButton DriverDPadDown = new POVButton(m_driverController, 180);
  private final POVButton DriverDPadLeft = new POVButton(m_driverController, 270);

  private final JoystickButton DriverRightBumper = new JoystickButton(m_driverController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton DriverLeftBumper = new JoystickButton(m_driverController,
      XboxController.Button.kLeftBumper.value);
      
  private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
  private final Trigger driverRightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorAButton = new JoystickButton(m_operatorController,
      XboxController.Button.kA.value);
  private final JoystickButton OperatorBButton = new JoystickButton(m_operatorController,
      XboxController.Button.kB.value);
  private final JoystickButton OperatorXButton = new JoystickButton(m_operatorController,
      XboxController.Button.kX.value);
  private final JoystickButton OperatorYButton = new JoystickButton(m_operatorController,
      XboxController.Button.kY.value);

  private final POVButton OperatorDPadUp = new POVButton(m_operatorController, 0);
  private final POVButton OperatorDPadRight = new POVButton(m_operatorController, 90);
  private final POVButton OperatorDPadDown = new POVButton(m_operatorController, 180);
  private final POVButton OperatorDPadLeft = new POVButton(m_operatorController, 270);

  private Trigger operatorRightYTrigger = new Trigger(() -> Math.abs(m_operatorController.getRightY()) > 0.10);

  private final Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5);
  private final Trigger operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorRightBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton OperatorLeftBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kLeftBumper.value);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    m_IntakeShooter.setDefaultCommand(new InstantCommand(() -> m_IntakeShooter.setPivotPower((Math.abs(m_operatorController.getLeftY())) > 0.1 ? -1*m_operatorController.getLeftY() : 0.00), m_IntakeShooter));
    //led.setDefaultCommand(new LEDCommand(led, intakeShooterBeamBreak, elevatorPivotBeamBreak));

    m_chooser.setDefaultOption("Two note", twoNoteAuto());
    m_chooser.addOption("get out of the way", getOutOfTheWay());
    m_chooser.addOption("shoot and move", shootAndMobility());
    //m_chooser.addOption("Shoot and leave", new PathPlannerAuto("GetOutOfTheWay1Auton"));
    //m_chooser.addOption("Test One Meter", new PathPlannerAuto("MeterTestPathAuton"));
    //m_chooser.addOption("Get Middle Notes Out" , new PathPlannerAuto("MoveCenterNotesAwayAuton"));
    //m_chooser.addOption("Parallel Commands", new PathPlannerAuto("ParallelAuton"));
    //m_chooser.addOption("two note take two", twoNoteAuto());
    
    SmartDashboard.putData(m_chooser);

    //test.setDefaultCommand(new testCommand(test, m_driverController));
    NamedCommands.registerCommand("Shoot", new InstantCommand(()-> m_IntakeShooter.setRollerPower(-1)));
    NamedCommands.registerCommand("IntakeShooterGoHome", new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.HOME, limitSwitch));
    NamedCommands.registerCommand("GetElevatorOutOfWay", new elevatorPosition(m_elevator, ElevatorPositions.AUTO_SHOOT));
    NamedCommands.registerCommand("GoToIntakePosition", new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.INTAKE, limitSwitch));
    NamedCommands.registerCommand("ReleasePreRoller", new InstantCommand(() -> m_IntakeShooter.setStowPower(1)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> m_IntakeShooter.setRollerPower(1)).alongWith(new InstantCommand(() -> m_IntakeShooter.setStowPower(-1))));
    NamedCommands.registerCommand("GoToShootPosition", new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.AUTON_SHOOT, limitSwitch));
    NamedCommands.registerCommand("StopAllShooters", new StopAllRollersCommand(m_IntakeShooter, m_elevator));
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
    /*
     * DRIVER BUTTON MAPPINGS
     */

    DriverGyroButton.whileTrue(new InstantCommand(
        () -> m_robotDrive.resetGyro(),
        m_robotDrive));

    DriverDPadUp.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(1.0), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0), m_elevator));
    DriverDPadDown.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(-1), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0), m_elevator));
    DriverDPadLeft.onTrue(new InstantCommand(() -> m_elevator.setPivotPower(-0.25), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0), m_elevator));
    DriverDPadRight.onTrue(new InstantCommand(() -> m_elevator.setPivotPower(0.25), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0), m_elevator));

    DriverAButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER, limitSwitch));
    //DriverAButton.onTrue(new ShootForTimeCommand(m_IntakeShooter, 2.0).andThen(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER)));


    DriverBButton.onTrue(new elevatorPosition(m_elevator, ElevatorPositions.AMP));
    // TODO: Probably need to change greater/less than signs in the implementation
    // of the method the command calls
    DriverXButton.onTrue(new elevatorPosition(m_elevator, ElevatorPositions.SOURCE)).onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.AUTON_SHOOT, limitSwitch));

    DriverAButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER, limitSwitch)).onTrue(new ShootAtRPM(m_IntakeShooter, 55000, -55000, 1));//.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(-1), m_IntakeShooter));
    DriverYButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.AMP, limitSwitch));

    DriverRightBumper.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.HOME, limitSwitch))
      .onTrue(new elevatorPosition(m_elevator, ElevatorPositions.HOME));//.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(0), m_IntakeShooter));

    DriverLeftBumper.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.INTAKE, limitSwitch));

    driverLeftTrigger.whileTrue(new InstantCommand(() -> {
      m_elevator.setRollerPower(-0.85);
      m_IntakeShooter.setStowPower(1);
      m_IntakeShooter.setRollerPower(-1);

    }, m_elevator, m_IntakeShooter))
        .onFalse(new StopAllRollersCommand(m_IntakeShooter, m_elevator));

    //driverLeftTrigger.onTrue(new ShootAtRPM(m_IntakeShooter, 3500, -3500, 1));
        //.onFalse(new StopAllRollersCommand(m_IntakeShooter, m_elevator));
// 
    driverRightTrigger.whileTrue(new InstantCommand(() -> {
      // Intake
      m_elevator.setRollerPower(0.85);
      m_IntakeShooter.setRollerPower(1);
      m_IntakeShooter.setStowPower(-1);
    }, m_elevator, m_IntakeShooter))
        .onFalse(new StopAllRollersCommand(m_IntakeShooter, m_elevator));

    /*
     * OPERATOR BUTTON MAPPING
     */
    operatorRightYTrigger
        .onTrue(new InstantCommand(
            () -> m_elevator.setPivotPower(
                (Math.abs(m_operatorController.getRightY()) > 0.1) ? m_operatorController.getRightY()*2 : 0.0),
            m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setPivotPower(0), m_elevator));

    OperatorDPadUp.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(1.0), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0), m_elevator));
    OperatorDPadDown.onTrue(new InstantCommand(() -> m_elevator.setElevatorPower(-1), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setElevatorPower(0), m_elevator));
    OperatorDPadLeft.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(1.0), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));
    OperatorDPadRight.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(-1), m_elevator))
        .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));

    operatorLeftTrigger.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(1)).alongWith(new InstantCommand(() -> m_IntakeShooter.setStowPower(-1))))
        .onFalse(new InstantCommand(() -> m_IntakeShooter.stopAllIntakeShooterRollers(), m_IntakeShooter));

    operatorRightTrigger.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(-1), m_IntakeShooter))
        .onFalse(new InstantCommand(() -> m_IntakeShooter.setRollerPower(0), m_IntakeShooter));

    OperatorBButton.onTrue(new InstantCommand(() -> m_IntakeShooter.setStowPower(1), m_IntakeShooter))
        .onFalse(new InstantCommand(() -> m_IntakeShooter.setStowPower(0), m_IntakeShooter));
    OperatorXButton.onTrue(new InstantCommand(() -> m_IntakeShooter.setStowPower(-1), m_IntakeShooter))
        .onFalse(new InstantCommand(() -> m_IntakeShooter.setStowPower(0), m_IntakeShooter));
    OperatorYButton.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(.5)))
        .onFalse(new InstantCommand(() -> m_IntakeShooter.setStowPower(0), m_IntakeShooter));
    OperatorAButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.AUTON_SHOOT, false, limitSwitch));
    OperatorStartButton.onTrue(new InstantCommand(() -> m_elevator.resetEncoder()));

    OperatorRightBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(-0.85), m_elevator))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));
    
    OperatorLeftBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(0.85), m_elevator))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));
  }

//------------- autonomous modes -------------
  public Command twoNoteAuto(){
    PathPlannerPath intakeAndDrive = PathPlannerPath.fromPathFile("IntakeAndDrive");
    PathPlannerPath goBackToStart = PathPlannerPath.fromPathFile("GoBackToStart");
    return new SequentialCommandGroup(
        NamedCommands.getCommand("Shoot"),
        NamedCommands.getCommand("GetElevatorOutOfWay"),
        NamedCommands.getCommand("ReleasePreRoller"),
        new WaitCommand(0.5),
        NamedCommands.getCommand("GoToIntakePosition"),
        NamedCommands.getCommand("Intake"),
        new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(3.35,5.58, new Rotation2d(0)))),
        AutoBuilder.followPath(intakeAndDrive),
        NamedCommands.getCommand("GoToShootPosition"),
        AutoBuilder.followPath(goBackToStart),
        new WaitCommand(0.5),
        NamedCommands.getCommand("ReleasePreRoller"),
        NamedCommands.getCommand("StopAllShooters"),
        new WaitCommand(100)
    );
  }

    public Command getOutOfTheWay(){
        PathPlannerPath getOutOfTheWay = PathPlannerPath.fromPathFile("GetOutOfTheWay1");
        PathPlannerPath goBackToStart = PathPlannerPath.fromPathFile("GoBackToStart");
        return new SequentialCommandGroup(
            NamedCommands.getCommand("Shoot"),
            NamedCommands.getCommand("GetElevatorOutOfWay"),
            NamedCommands.getCommand("ReleasePreRoller"),
            new WaitCommand(0.5),
            NamedCommands.getCommand("GoToIntakePosition"),
            NamedCommands.getCommand("Intake"),
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(-Math.PI/3)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(-60)),
            AutoBuilder.followPath(getOutOfTheWay)
            
            /*NamedCommands.getCommand("GoToShootPosition"),
            AutoBuilder.followPath(goBackToStart),
            new WaitCommand(0.5),
            NamedCommands.getCommand("ReleasePreRoller"),
            NamedCommands.getCommand("StopAllShooters"),
            new WaitCommand(100)*/
        );
    }

    public Command shootAndMobility(){
        PathPlannerPath getOutOfTheWay = PathPlannerPath.fromPathFile("GetOutOfTheWay2");
        PathPlannerPath goBackToStart = PathPlannerPath.fromPathFile("GoBackToStart");
        return new SequentialCommandGroup(
            NamedCommands.getCommand("Shoot"),
            NamedCommands.getCommand("GetElevatorOutOfWay"),
            NamedCommands.getCommand("ReleasePreRoller"),
            new WaitCommand(0.5),
            //NamedCommands.getCommand("GoToIntakePosition"),
            //NamedCommands.getCommand("Intake"),
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.77,6.84, new Rotation2d(0)))),
            AutoBuilder.followPath(getOutOfTheWay)
        
        /*NamedCommands.getCommand("GoToShootPosition"),
        AutoBuilder.followPath(goBackToStart),
        new WaitCommand(0.5),
        NamedCommands.getCommand("ReleasePreRoller"),
        NamedCommands.getCommand("StopAllShooters"),
        new WaitCommand(100)*/
        );
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SmartDashboard.putData("selected auto", m_chooser.getSelected());
    //return m_chooser.getSelected();
    /*return new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(-Math.PI/3)))),
                                      new InstantCommand(() -> m_robotDrive.setGyroYawOffset(-60)),
                                      new PathPlannerAuto("GetOutOfTheWay1Auton"))*/
    return twoNoteAuto();
  }
}