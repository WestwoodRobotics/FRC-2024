package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command to align and range the robot to a vision target using PID control.
 * This command utilizes vision data to align the robot's orientation and position
 * with respect to a detected target, typically an AprilTag.
 */
public class DriveAlignAndRangeVisionCommand extends CommandBase {

    private Vision vision;
    private SwerveDrive swerveDriveSubsystem;
    private boolean aprilTagDetected;
    private double horizontalOffset;
    private double verticalOffset;
    private double targetDistance;
    private PIDController pidController;
    private double scaledRotation;
    private double scaledForwardSpeed;
    private double scaledStrafeSpeed;
    private double rotationOffset;
    private double proportionalGain = .035; // Proportional gain for aiming
    private double rangeProportionalGain = .05; // Proportional gain for ranging
    private double strafeProportionalGain = .1; // Proportional gain for strafing
    private double desiredTargetDistance = 1.0; // Desired distance from the April tag

    public DriveAlignAndRangeVisionCommand(Vision vision, SwerveDrive swerveDriveSubsystem){
        this.vision = vision;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(vision, swerveDriveSubsystem);
    }
    
    @Override
    public void execute() {
        // Get the current state of the AprilTag
        aprilTagDetected = vision.found();
        if (aprilTagDetected) {
            rotationOffset = vision.getHorizontalDiff();
            verticalOffset = vision.getVerticalDiff();

            // Aiming
            pidController = new PIDController(proportionalGain, 0, 0);
            double pidOutput = pidController.calculate(rotationOffset);
            scaledRotation = MathUtil.clamp(pidOutput, -1, 1); // Clamp the value between -1 and 1

            // Ranging
            pidController = new PIDController(rangeProportionalGain, 0, 0);
            pidOutput = pidController.calculate(targetDistance - desiredTargetDistance);
            scaledForwardSpeed = MathUtil.clamp(pidOutput, -1, 1); // Clamp the value between -1 and 1

            // Drive the robot
            swerveDriveSubsystem.drive(scaledForwardSpeed, 0, scaledRotation, true, false);
        } else {
            swerveDriveSubsystem.drive(0, 0, 0, false, false);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
