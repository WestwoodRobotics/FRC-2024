package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Command for driving the robot using a swerve drive system.
 * This command reads inputs from an XboxController and translates them into swerve drive movements.
 */
public class driveCommand extends Command {

  private final SwerveDrive swerveDriveSubsystem;
  private XboxController xboxController;
  private boolean isSlowModeEnabled;

  /**
   * Constructs a new driveCommand.
   *
   * @param swerveDrive The swerve drive subsystem to control.
   * @param controller  The Xbox controller to use for input.
   */
  public driveCommand(SwerveDrive swerveDrive, XboxController controller) {
    swerveDriveSubsystem = swerveDrive;
    this.xboxController = controller;
    addRequirements(swerveDrive);
  }

  /**
   * Initializes the command.
   */
  @Override
  public void initialize() {
    isSlowModeEnabled = false;
  }

  /**
   * Executes the command.
   * Reads the joystick values from the controller and drives the robot.
   */
  @Override
  public void execute() {
    double leftXAxis, leftYAxis, rightXAxis;
    if (xboxController.getBackButtonPressed()) {
      isSlowModeEnabled = !isSlowModeEnabled;
    }

    leftXAxis = -MathUtil.applyDeadband(xboxController.getLeftX(), ControllerConstants.kDriveDeadband);
    leftYAxis = -MathUtil.applyDeadband(xboxController.getLeftY(), ControllerConstants.kDriveDeadband);
    rightXAxis = -MathUtil.applyDeadband(xboxController.getRightX(), ControllerConstants.kDriveDeadband);

    // Apply non-linear input (squaring the input)
    leftXAxis = Math.copySign(Math.pow(leftXAxis, 2), leftXAxis);
    leftYAxis = Math.copySign(Math.pow(leftYAxis, 2), leftYAxis);
    rightXAxis = Math.copySign(Math.pow(rightXAxis, 2), rightXAxis);

    if (isSlowModeEnabled) {
      leftXAxis *= Constants.DriveConstants.slowModeMultiplier;
      leftYAxis *= Constants.DriveConstants.slowModeMultiplier;
      rightXAxis *= Constants.DriveConstants.slowModeMultiplier;
    }
    swerveDriveSubsystem.drive(leftYAxis, leftXAxis, rightXAxis, true, false);
  }

  /**
   * Ends the command.
   *
   * @param interrupted Whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns whether the command has finished.
   *
   * @return false to keep the command running.
   */
  @Override
  public boolean isFinished() {
    return false; // should never end in teleop
  }
}
