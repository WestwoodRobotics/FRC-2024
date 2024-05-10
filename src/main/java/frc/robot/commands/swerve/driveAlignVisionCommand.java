package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command to align the robot with a vision target using PID control.
 */
public class driveAlignVisionCommand extends Command {

    private Vision visionSubsystem;
    private SwerveDrive swerveDriveSubsystem;

    private boolean isAprilTagFound;
    private double horizontalDifference;
    PIDController pidController;
    double scaledRotateValue;

    /**
     * Constructs a driveAlignVisionCommand.
     * 
     * @param visionSubsystem The vision subsystem used to detect targets.
     * @param swerveDriveSubsystem The swerve drive subsystem used to control the robot's movement.
     */
    public driveAlignVisionCommand(Vision visionSubsystem, SwerveDrive swerveDriveSubsystem){
        this.visionSubsystem = visionSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        horizontalDifference = visionSubsystem.getHorizontalDiff();
    }
    
    /**
     * Executes the vision alignment command.
     */
    @Override
    public void execute(){
    
        if ((horizontalDifference > 1) && (isAprilTagFound)){
            pidController = new PIDController(1, 0, 0);
            scaledRotateValue = pidController.calculate(horizontalDifference);
            swerveDriveSubsystem.drive(0,0,scaledRotateValue, false, false);
        }
    }

    /**
     * Determines if the command has finished.
     * 
     * @return true if the target is aligned, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return (isAprilTagFound ? (horizontalDifference < 1) : (false));
    }
}
