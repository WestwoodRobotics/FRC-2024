package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.elevator.elevatorPosition;
import frc.robot.commands.elevator.elevatorRollerCommand;
import frc.robot.commands.intakeShooter.IntakeRollersCommand;
import frc.robot.commands.intakeShooter.IntakeCommandFactory;
import frc.robot.commands.intakeShooter.IntakeShooterPosition;
import frc.robot.commands.intakeShooter.JuggernautFreeze;
import frc.robot.commands.swerve.DriveAlignAndRangeVisionCommand;
import frc.robot.commands.swerve.driveAlignVisionCommand;
import frc.robot.commands.swerve.driveCommand;
import frc.robot.commands.utils.StopAllRollersCommand;
import frc.robot.commands.vision.LEDCommand;
import frc.robot.subsystems.vision.BeamBreak;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.intakeShooter.IntakeRollers;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LED;
import frc.robot.subsystems.vision.Vision;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveDrive m_robotDrive = new SwerveDrive();
  private LED led = new LED(9);
  private BeamBreak intakeShooterBeamBreak = new BeamBreak(8);
  private BeamBreak elevatorPivotBeamBreak = new BeamBreak(9);
  public final Elevator m_elevator = new Elevator();
  private Vision vision = new Vision();

  private final IntakePivot m_IntakeShooterPivot = new IntakePivot();
  private final IntakeRollers m_IntakeShooterRollers = new IntakeRollers();

  private final IntakeCommandFactory intakeCommandFactory = new IntakeCommandFactory();
  
  

  // LED for indicating robot state, not implemented in hardware.

  // The driver's controller
  XboxController m_driverController = new XboxController(PortConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(PortConstants.kOperatorControllerPort);
  XboxController m_programmerController = new XboxController(PortConstants.kProgrammerControllerPort);

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
      
  private final Trigger driverRightTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
  private final Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);

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
  private Trigger operatorLeftYTrigger = new Trigger(() -> Math.abs(m_operatorController.getLeftY()) > 0.10);

  private final Trigger operatorLeftTrigger = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.5);
  private final Trigger operatorRightTrigger = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5);

  private final JoystickButton OperatorRightBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton OperatorLeftBumper = new JoystickButton(m_operatorController,
      XboxController.Button.kLeftBumper.value);


  private final JoystickButton programmerAButton = new JoystickButton(m_programmerController, XboxController.Button.kA.value);
    private final JoystickButton programmerBButton = new JoystickButton(m_programmerController, XboxController.Button.kB.value);
    private final JoystickButton programmerXButton = new JoystickButton(m_programmerController, XboxController.Button.kX.value);
    private final JoystickButton programmerYButton = new JoystickButton(m_programmerController, XboxController.Button.kY.value);
    private final JoystickButton programmerRightBumper = new JoystickButton(m_programmerController,
        XboxController.Button.kRightBumper.value);
    private final JoystickButton programmerLeftBumper = new JoystickButton(m_programmerController,
        XboxController.Button.kLeftBumper.value);
    private final Trigger programmerRightTrigger = new Trigger(() -> m_programmerController.getRightTriggerAxis() > 0.5);
    private final Trigger programmerLeftTrigger = new Trigger(() -> m_programmerController.getLeftTriggerAxis() > 0.5);
    private final POVButton programmerDPadUp = new POVButton(m_programmerController, 0);
    private final POVButton programmerDPadRight = new POVButton(m_programmerController, 90);
    private final POVButton programmerDPadDown = new POVButton(m_programmerController, 180);
    private final POVButton programmerDPadLeft = new POVButton(m_programmerController, 270);


    private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure default commands 
    // m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    
    m_robotDrive.setDefaultCommand(new driveCommand(m_robotDrive, m_driverController));
    //m_IntakeShooterPivot.setDefaultCommand(new InstantCommand(() -> m_IntakeShooterPivot.setPivotPower((Math.abs(m_operatorController.getLeftY())) > 0.1 ? -1*m_operatorController.getLeftY() : 0.00), m_IntakeShooterPivot));
    //
    //m_IntakeShooterPivot.setDefaultCommand(new JuggernautFreeze(m_IntakeShooterPivot));
    //led.setDefaultCommand(new LEDCommand(led, intakeShooterBeamBreak, elevatorPivotBeamBreak));
    led.setDefaultCommand(new LEDCommand(led, intakeShooterBeamBreak, elevatorPivotBeamBreak));


    //m_chooser.addOption("meterTest" , meterTest());
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //test.setDefaultCommand(new testCommand(test, m_driverController));
    NamedCommands.registerCommand("Shoot", intakeCommandFactory.goToShootPositionAndShoot(m_IntakeShooterPivot, m_IntakeShooterRollers));

    NamedCommands.registerCommand("IntakeShooterGoHome", new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME));

    NamedCommands.registerCommand("GetElevatorOutOfWay", new elevatorPosition(m_elevator, ElevatorPositions.AUTO_SHOOT));

    NamedCommands.registerCommand("ReleasePreRoller", new IntakeRollersCommand(m_IntakeShooterRollers, -1, 1));

    NamedCommands.registerCommand("IntakePose", new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.AUTON_INTAKE));

    NamedCommands.registerCommand("StopAllShooters", new StopAllRollersCommand(m_IntakeShooterRollers, m_elevator));

    NamedCommands.registerCommand("StopIntakePivot", new InstantCommand(() -> m_IntakeShooterPivot.setPivotPower(0)));

    NamedCommands.registerCommand("IntakeAndPosition", intakeCommandFactory.goToIntakePositionAndIntake(m_IntakeShooterPivot, m_IntakeShooterRollers));

    NamedCommands.registerCommand("Intake", new IntakeRollersCommand(m_IntakeShooterRollers, 1, -1));

    NamedCommands.registerCommand("GoToHomePosition", new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME));
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

    //DriverAButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER, limitSwitch));
    //DriverAButton.onTrue(new ShootForTimeCommand(m_IntakeShooter, 2.0).andThen(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER)));


    DriverBButton.onTrue(new elevatorPosition(m_elevator, ElevatorPositions.AMP));
    // TODO: Probably need to change greater/less than signs in the implementation
    // of the method the command calls
    DriverXButton.onTrue(new elevatorPosition(m_elevator, ElevatorPositions.SOURCE))
    .onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.AUTON_SHOOT).andThen(new JuggernautFreeze(m_IntakeShooterPivot)));

    //DriverAButton.onTrue(new IntakeShooterPosition(m_IntakeShooter, IntakeShooterPositions.SHOOT_NEAR_SPEAKER, limitSwitch)).onTrue(new ShootAtRPM(m_IntakeShooter, 55000, -55000, 1));//.onTrue(new InstantCommand(() -> m_IntakeShooter.setRollerPower(-1), m_IntakeShooter));
    DriverAButton.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS).andThen(new JuggernautFreeze(m_IntakeShooterPivot)).alongWith(new IntakeRollersCommand(m_IntakeShooterRollers, -1, 0))).onTrue(new elevatorPosition(m_elevator, ElevatorPositions.AUTO_SHOOT));

    //DriverYButton.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS, limitSwitch).alongWith(new InstantCommand(() -> m_IntakeShooterRollers.setRollerPower(1), m_IntakeShooterRollers).andThen(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(-1),m_IntakeShooterRollers))));
    DriverYButton.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.PODIUM_SHOT).alongWith(new IntakeRollersCommand(m_IntakeShooterRollers, -1, 0))).onFalse(new JuggernautFreeze(m_IntakeShooterPivot));
    DriverRightBumper.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME))
      .onTrue(new elevatorPosition(m_elevator, ElevatorPositions.HOME));


    DriverLeftBumper.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.INTAKE));

    // driverLeftTrigger.whileTrue(new InstantCommand(() -> {
    //   m_elevator.setRollerPower(-1);
    //   m_IntakeShooterRollers.setStowPower(1);
    //   m_IntakeShooterRollers.setRollerPower(-1);
    //   //m_IntakeShooter.setRollerRPM(55000, -55000);
    // }, m_elevator, m_IntakeShooterRollers))
    //     .onFalse(new StopAllRollersCommand(m_IntakeShooterRollers, m_elevator)).onFalse(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME)).onFalse(new elevatorPosition(m_elevator, ElevatorPositions.HOME));



    driverLeftTrigger.whileTrue(new InstantCommand(()-> m_elevator.setRollerPower(-1), m_elevator).alongWith(new IntakeRollersCommand(m_IntakeShooterRollers, -1, 1)))
        .onFalse(new StopAllRollersCommand(m_IntakeShooterRollers, m_elevator)).onFalse(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME)).onFalse(new elevatorPosition(m_elevator, ElevatorPositions.HOME));
    //driverLeftTrigger.onTrue(new ShootAtRPM(m_IntakeShooter, 3500, -3500, 1));
        //.onFalse(new StopAllRollersCommand(m_IntakeShooter, m_elevator));
// 
    // driverRightTrigger.whileTrue(new elevatorRollerCommand(m_elevator, 1, elevatorPivotBeamBreak)).whileTrue(new InstantCommand(() -> {
    //   // Intake
    //   m_IntakeShooterRollers.setRollerPower(1);
    //   m_IntakeShooterRollers.setStowPower(-1);
    // }, m_elevator, m_IntakeShooterRollers))
    //     .onFalse(new StopAllRollersCommand(m_IntakeShooterRollers, m_elevator));

    driverRightTrigger.whileTrue(new elevatorRollerCommand(m_elevator, 1, elevatorPivotBeamBreak).alongWith(new IntakeRollersCommand(m_IntakeShooterRollers, 1, -1)))
    .onFalse(new StopAllRollersCommand(m_IntakeShooterRollers, m_elevator));
    /*
     * OPERATOR BUTTON MAPPING
     */
    operatorRightYTrigger
        .onTrue(new InstantCommand(
            () -> m_elevator.setPivotPower(
                (Math.abs(m_operatorController.getRightY()) > 0.1) ? m_operatorController.getRightY()/2 : 0.0),
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

    operatorLeftTrigger.onTrue(new InstantCommand(() -> m_IntakeShooterRollers.setRollerPower(-1)).alongWith(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(-1))))
        .onFalse(new InstantCommand(() -> m_IntakeShooterRollers.stopAllIntakeShooterRollers(), m_IntakeShooterPivot));

    operatorRightTrigger.onTrue(new InstantCommand(() -> m_IntakeShooterRollers.setRollerPower(1), m_IntakeShooterPivot))
        .onFalse(new InstantCommand(() -> m_IntakeShooterRollers.setRollerPower(0), m_IntakeShooterPivot));

    OperatorBButton.onTrue(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(1), m_IntakeShooterPivot))
        .onFalse(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(0), m_IntakeShooterPivot));
    OperatorXButton.onTrue(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(-1), m_IntakeShooterPivot))
        .onFalse(new InstantCommand(() -> m_IntakeShooterRollers.setStowPower(0), m_IntakeShooterPivot));

    OperatorYButton.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS).alongWith(new IntakeRollersCommand(m_IntakeShooterRollers, 1, 0)).andThen(new JuggernautFreeze(m_IntakeShooterPivot)));

    OperatorAButton.onTrue(new elevatorPosition(m_elevator, ElevatorPositions.HANDOFF));
    OperatorStartButton.onTrue(new InstantCommand(() -> m_elevator.resetEncoder()));

    OperatorRightBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(-0.85), m_elevator))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));
    
    OperatorLeftBumper.onTrue(new InstantCommand(() -> m_elevator.setRollerPower(0.85), m_elevator))
    .onFalse(new InstantCommand(() -> m_elevator.setRollerPower(0), m_elevator));

    operatorLeftYTrigger.whileTrue(new InstantCommand(() -> m_IntakeShooterPivot.setPivotPower((Math.abs(m_operatorController.getLeftY())) > 0.1 ? -1*m_operatorController.getLeftY() : 0.00), m_IntakeShooterPivot)).onFalse(new JuggernautFreeze(m_IntakeShooterPivot));



    programmerLeftBumper.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.INTAKE));
    programmerRightBumper.onTrue(new IntakeShooterPosition(m_IntakeShooterPivot, IntakeShooterPositions.HOME));

    programmerDPadLeft.onTrue(new InstantCommand(() -> m_IntakeShooterPivot.setPivotPower(0.5), m_IntakeShooterPivot)).onFalse(
        new JuggernautFreeze(m_IntakeShooterPivot));
    
    programmerDPadRight.whileTrue(new InstantCommand(() -> m_IntakeShooterPivot.setPivotPower(-0.5), m_IntakeShooterPivot)).onFalse(new JuggernautFreeze(m_IntakeShooterPivot));



    
  }

//------------------------------------------- autonomous modes -------------------------------------------

    public Command twoNoteAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.35,5.58, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0)),
            new PathPlannerAuto("TwoNoteAuton")
        );
        toReturn.setName("two note");
        return toReturn;
    }

    // public Command getOutOfTheWay(){
    //     PathPlannerPath getOutOfTheWay = PathPlannerPath.fromPathFile("GetOutOfTheWay1");
    //     PathPlannerPath goBackToStart = PathPlannerPath.fromPathFile("GoBackToStart");
    //     return new SequentialCommandGroup(
    //         NamedCommands.getCommand("Shoot"),
    //         NamedCommands.getCommand("GetElevatorOutOfWay"),
    //         NamedCommands.getCommand("ReleasePreRoller"),
    //         new WaitCommand(0.5),
    //         NamedCommands.getCommand("GoToIntakePosition"),
    //         NamedCommands.getCommand("Intake"),
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(-Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(-60)),
    //         AutoBuilder.followPath(getOutOfTheWay)
            
    //         /*NamedCommands.getCommand("GoToShootPosition"),
    //         AutoBuilder.followPath(goBackToStart),
    //         new WaitCommand(0.5),
    //         NamedCommands.getCommand("ReleasePreRoller"),
    //         NamedCommands.getCommand("StopAllShooters"),
    //         new WaitCommand(100)*/
    //     );
    // }

    // public Command meterTest(){
    //    return new PathPlannerAuto("MeterTestPathAuton");
    // }

    // public Command shootAndPickup(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(2*Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(120)),
    //         new PathPlannerAuto("ShootAndPickupAuton")
    //     );
    // }

    public Command shootPickupShoot(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(2*Math.PI/3)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(120)),
            new PathPlannerAuto("ShootPickupShootAuton")
        );
        toReturn.setName("shoot pickup from middle shoot");
        return toReturn;
    }

    // public Command ampSide(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.68,4.38, new Rotation2d(-Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(60)),
    //         new PathPlannerAuto("AmpSideAuton")
    //     );
    // }

    // public Command testingOtherSideSubwoofer(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.74,6.68, new Rotation2d(Math.PI/3)))),
    //         new InstantCommand(() -> m_robotDrive.setGyroYawOffset(60)),
    //         new PathPlannerAuto("TestingOtherSideSubwooferAuton")
    //     );
    // }

	public Command centerNoteTopAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.09,5.56, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0)),
            new PathPlannerAuto("CenterTopNotesAuton")
           );
        toReturn.setName("Three Notes (Preload, Amp & Middle)");
        return toReturn;
    }

    public Command centerNoteBottomAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(1.09,5.56, new Rotation2d(0)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(0)),
            new PathPlannerAuto("CenterBottomNotesAuton")
        );
        toReturn.setName("Three Notes (Preload, Source & Middle)");
        return toReturn;
    }

    public Command messUpNotesAuto(){
        Command toReturn = new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.resetPose(new Pose2d(0.57,4.56, new Rotation2d(2*Math.PI/3)))),
            new InstantCommand(() -> m_robotDrive.setGyroYawOffset(120)),
            new PathPlannerAuto("MoveCenterNotesAwayAuton")
        );
        toReturn.setName("get middle notes out");
        return toReturn;
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
