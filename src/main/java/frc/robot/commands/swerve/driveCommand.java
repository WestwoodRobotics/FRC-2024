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

  private final SwerveDrive m_swerveDrive;
  private XboxController controller;
  private boolean slowMode;

  /**
   * Constructs a new driveCommand.
   *
   * @param swerveDrive The swerve drive subsystem to control.
   * @param controller  The Xbox controller to use for input.
   */
  public driveCommand(SwerveDrive swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;
    addRequirements(swerveDrive);
  }

  /**
   * Initializes the command.
   */
  @Override
  public void initialize() {
    slowMode = false;
  }

  /**
   * Executes the command.
   * Reads the joystick values from the controller and drives the robot.
   */
  @Override
  public void execute() {
    double leftX, leftY, rightX;
    if (controller.getBackButtonPressed()) {
      slowMode = !slowMode;
    }

    leftX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
    leftY = -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDriveDeadband);
    rightX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);

    // Apply non-linear input (squaring the input)
    leftX = Math.copySign(Math.pow(leftX, 2), leftX);
    leftY = Math.copySign(Math.pow(leftY, 2), leftY);
    rightX = Math.copySign(Math.pow(rightX, 2), rightX);

    if (slowMode) {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }
    m_swerveDrive.drive(leftY, leftX, rightX, true, false);
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
