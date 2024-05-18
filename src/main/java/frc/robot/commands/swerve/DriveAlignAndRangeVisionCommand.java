package frc.robot.commands.swerve;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command for aligning and ranging the robot using vision.
 * This command uses vision data to align the robot with a target and maintain a specific distance from it.
 */
public class DriveAlignAndRangeVisionCommand extends Command {

    private Vision visionSubsystem;
    private SwerveDrive swerveDriveSubsystem;
    private boolean isTargetFound;
    private double targetHorizontalDifference;
    private double targetVerticalDifference;
    private double targetDistance;
    private PIDController pidController;
    private double scaledRotationValue;
    private double scaledForwardMovementValue;
    private double scaledStrafeMovementValue;
    private double rotationDistanceToTarget;
    private double kP = .035; // constant of proportionality for aiming
    private double kP_range = .05; // constant of proportionality for ranging
    private double kP_strafe = .1; // constant of proportionality for strafing
    private double desiredDistance = 1.0; // desired distance from the April tag

    /**
     * Constructs a DriveAlignAndRangeVisionCommand.
     * 
     * @param vision The vision subsystem used to detect targets.
     * @param swerveDrive The swerve drive subsystem used to control the robot's movement.
     */
    public DriveAlignAndRangeVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.visionSubsystem = vision;
        this.swerveDriveSubsystem = swerveDrive;
        addRequirements(vision, swerveDrive);
    }
    
    @Override
    /*
     * From a top-down perspective
     */
    public void execute() {
        // Get the current state of the AprilTag
        isTargetFound = visionSubsystem.found();
        if (isTargetFound) {
            rotationDistanceToTarget = visionSubsystem.getHorizontalDiff();
            targetVerticalDifference = visionSubsystem.getVerticalDiff();

            // Aiming
            pidController = new PIDController(kP, 0, 0);
            double pidOutput = pidController.calculate(rotationDistanceToTarget);
            scaledRotationValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1

            // Ranging
            pidController = new PIDController(kP_range, 0, 0);
            pidOutput = pidController.calculate(targetDistance - desiredDistance);
            scaledForwardMovementValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1

            // Drive the robot
            swerveDriveSubsystem.drive(scaledForwardMovementValue, 0, scaledRotationValue, true, false);
        } else {
            swerveDriveSubsystem.drive(0, 0, 0, false, false);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
