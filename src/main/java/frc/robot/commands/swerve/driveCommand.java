package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.DriveSpeed;

// import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
// import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

public class driveCommand extends Command {

  private final SwerveDrive m_swerveDrive;

  private XboxController controller;
  private boolean slowMode;




  public driveCommand(SwerveDrive swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;
    addRequirements(swerveDrive);
  }


  @Override
  public void initialize() 
  {
    slowMode = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX, leftY, rightX;
    if (controller.getBackButtonPressed())
    {
      slowMode = !slowMode;
    }

    leftX = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.kDriveDeadband);
    leftY = -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDriveDeadband);
    rightX = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband);


    // Apply non-linear input (squaring the input)
    // leftX = Math.copySign(Math.pow(leftX, 2), leftX);
    // leftY = Math.copySign(Math.pow(leftY, 2), leftY);
    // rightX = Math.copySign(Math.pow(rightX, 2), rightX);


    if (slowMode)
    {
      leftX *= Constants.DriveConstants.slowModeMultiplier;
      leftY *= Constants.DriveConstants.slowModeMultiplier;
      rightX *= Constants.DriveConstants.slowModeMultiplier;
    }
    m_swerveDrive.drive(leftY, leftX, rightX, true, false);

  }


  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should never end in teleop
    return false;
  }
}