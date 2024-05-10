package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Command for driving the robot using the swerve drive subsystem.
 * This command utilizes the XboxController inputs to control the robot's movement.
 */
public class driveCommand extends Command {

  private final SwerveDrive swerveDriveSubsystem;

  private XboxController driverController;
  private boolean isSlowModeEnabled;

  public driveCommand(SwerveDrive swerveDriveSubsystem, XboxController driverController) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.driverController = driverController;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    isSlowModeEnabled = false;
  }

  @Override
  public void execute() {
    double leftJoystickX, leftJoystickY, rightJoystickRotation;
    if (driverController.getBackButtonPressed()) {
      isSlowModeEnabled = !isSlowModeEnabled;
    }

    leftJoystickX = -MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.kDriveDeadband);
    leftJoystickY = -MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.kDriveDeadband);
    rightJoystickRotation = -MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.kDriveDeadband);

    leftJoystickX = Math.copySign(Math.pow(leftJoystickX, 2), leftJoystickX);
    leftJoystickY = Math.copySign(Math.pow(leftJoystickY, 2), leftJoystickY);
    rightJoystickRotation = Math.copySign(Math.pow(rightJoystickRotation, 2), rightJoystickRotation);

    if (isSlowModeEnabled) {
      leftJoystickX *= Constants.DriveConstants.slowModeMultiplier;
      leftJoystickY *= Constants.DriveConstants.slowModeMultiplier;
      rightJoystickRotation *= Constants.DriveConstants.slowModeMultiplier;
    }
    swerveDriveSubsystem.drive(leftJoystickY, leftJoystickX, rightJoystickRotation, true, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
