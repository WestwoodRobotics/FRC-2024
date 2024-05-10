package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * Command to align the robot with a vision target using PID control.
 */
public class driveAlignVisionCommand extends CommandBase {

    private final Vision vision;
    private final SwerveDrive swerveDriveSubsystem;
    private boolean aprilTagDetected;
    private double horizontalOffset;
    private final PIDController pidController;
    private double scaledRotation;

    /**
     * Constructs a new driveAlignVisionCommand.
     * 
     * @param vision The vision subsystem used to detect targets.
     * @param swerveDriveSubsystem The swerve drive subsystem used to control the robot's movement.
     */
    public driveAlignVisionCommand(Vision vision, SwerveDrive swerveDriveSubsystem){
        this.vision = vision;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(vision, swerveDriveSubsystem);
        pidController = new PIDController(0.0325, 0, 0);
    }
    
    @Override
    public void execute(){
        aprilTagDetected = vision.found();
        horizontalOffset = vision.getHorizontalDiff();
        if (aprilTagDetected){
            double pidOutput = pidController.calculate(horizontalOffset);
            scaledRotation = MathUtil.clamp(pidOutput, -1, 1); // clamp the value between -1 and 1
            System.out.println(scaledRotation);
            swerveDriveSubsystem.drive(0,0, scaledRotation, false, false);
        }
        else{
            swerveDriveSubsystem.drive(0,0,Math.copySign(0.2, scaledRotation),false,false);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
