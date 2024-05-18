package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command for aligning the robot with a vision target using a PID controller.
 * This command uses vision data to rotate the robot to align with a target.
 */
public class driveAlignVisionCommand extends Command {

    private Vision visionSubsystem;
    private SwerveDrive swerveDriveSubsystem;
    private boolean isTargetFound;
    private double targetHorizontalDifference;
    private PIDController pidController;
    private double scaledRotationValue;

    /**
     * Constructs a new driveAlignVisionCommand.
     * 
     * @param vision The vision subsystem used to detect targets.
     * @param swerveDrive The swerve drive subsystem used to control the robot's movement.
     */
    public driveAlignVisionCommand(Vision vision, SwerveDrive swerveDrive){
        this.visionSubsystem = vision;
        targetHorizontalDifference = vision.getHorizontalDiff();
        this.swerveDriveSubsystem = swerveDrive;
        this.visionSubsystem = vision;
        addRequirements(vision, swerveDrive);
    }
    
    /**
     * Executes the command to align the robot with the vision target.
     */
    @Override
    public void execute(){
        isTargetFound = visionSubsystem.found();
        targetHorizontalDifference = visionSubsystem.getHorizontalDiff();
        if (isTargetFound){
            pidController = new PIDController(0.0325, 0, 0);
            double pidOutput = pidController.calculate(targetHorizontalDifference);
            scaledRotationValue = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1
            System.out.println(scaledRotationValue);
            swerveDriveSubsystem.drive(0,0, scaledRotationValue, false, false);
        }
        else{
            swerveDriveSubsystem.drive(0,0,Math.copySign(0.2, scaledRotationValue),false,false);
        }
    }

    /**
     * Determines if the command is finished.
     * 
     * @return false to keep the command running.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
}
