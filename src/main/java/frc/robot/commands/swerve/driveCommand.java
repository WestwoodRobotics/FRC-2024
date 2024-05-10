package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Command for driving the swerve drive subsystem using an Xbox controller.
 */
public class driveCommand extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController controller;
  private boolean slowMode;

  /**
   * Constructs a new driveCommand.
   * 
   * @param swerveDrive The SwerveDrive subsystem this command will run on.
   * @param controller The XboxController to use for driving input.
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
   * Executes the command at each scheduler run.
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

    if (slowMode) {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }
    m_swerveDrive.drive(leftY, leftX, rightX, true, false);
  }

  /**
   * Ends the command when interrupted or finished.
   * 
   * @param interrupted Whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Specifies whether the command has finished.
   * 
   * @return false to keep the command running.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
